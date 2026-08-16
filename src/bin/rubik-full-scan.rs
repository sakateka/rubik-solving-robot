//! Interactive end-to-end scan: stand choreography → camera → TPU → facelets.

use anyhow::{bail, Context, Result};
use clap::Parser;
use rubik_scan::{
    camera::Camera,
    cube::{CubeState, Face, LogicalFace, QuarterTurns, ScanPose},
    grid,
    model::Detector,
    pca9685::Pca9685,
    postprocess, preprocess,
    stand::StandCalibration,
    stand_runtime::{CommandedStandState, ScanFace, StandRuntime},
    tpu::CviTpuDetector,
};
use std::{
    ffi::CString,
    io::{self, BufRead, Write},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::Duration,
};

#[derive(Parser)]
#[command(about = "Scan all six Rubik faces with the stand, camera, and TPU")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Optional TOML file overriding built-in stand calibration
    #[arg(long)]
    config: Option<PathBuf>,

    /// Servo PWM frequency
    #[arg(long, default_value_t = 50.0)]
    pwm_hz: f64,

    /// Path to vendor sensor_cfg.ini
    #[arg(long, default_value = "/mnt/data/sensor_cfg.ini")]
    sensor_config: PathBuf,

    /// VPSS frames discarded once after camera start
    #[arg(long, default_value_t = 10)]
    warmup_frames: u32,

    /// Path to the production CV181X TPU model
    #[arg(long)]
    cvimodel: PathBuf,

    /// Detections below this confidence are discarded
    #[arg(long, default_value_t = 0.5)]
    conf: f32,

    /// IoU threshold for class-agnostic NMS
    #[arg(long, default_value_t = 0.5)]
    iou: f32,

    /// Required acknowledgement that startup and the `scan` command move the stand
    #[arg(long)]
    confirm_stand_motion: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_stand_motion {
        bail!("refusing to start motion runtime; pass --confirm-stand-motion after checking the stand");
    }

    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let mut output = Pca9685::open(&cli.i2c_device, cli.address)?;
    let pwm = output.initialize_safe_pwm(cli.pwm_hz)?;
    let mut stand = StandRuntime::new(output, calibration);
    stand.reset()?;
    stand.safe_open()?;
    println!("stand=safe-open pwm_hz={:.3}", pwm.pwm_hz());

    let sensor_config = CString::new(cli.sensor_config.to_string_lossy().as_bytes())?;
    let camera = Camera::open(&sensor_config)?;
    camera.warmup_vpss(cli.warmup_frames)?;
    let detector = CviTpuDetector::load(&cli.cvimodel)?;
    println!("scanner ready; commands: scan, state, off, quit");

    command_loop(&mut stand, &camera, &detector, cli.conf, cli.iou)
}

fn command_loop(
    stand: &mut StandRuntime<Pca9685>,
    camera: &Camera,
    detector: &CviTpuDetector,
    confidence: f32,
    iou: f32,
) -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))
        .context("failed to install Ctrl-C shutdown handler")?;

    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            if sender.send(line).is_err() {
                break;
            }
        }
    });

    let mut stdout = io::stdout();
    prompt(&mut stdout)?;
    while running.load(Ordering::SeqCst) {
        let line = match receiver.recv_timeout(Duration::from_millis(100)) {
            Ok(line) => line.context("failed to read scanner command from stdin")?,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        match line.trim() {
            "" => {}
            "help" => println!("commands: scan, state, off, quit"),
            "state" => println!("state={}", state_name(stand.state())),
            "scan" => match full_scan(stand, camera, detector, confidence, iou) {
                Ok(result) => print_scan_result(&result)?,
                Err(error) => eprintln!("scan stopped: {error:#}"),
            },
            "off" => {
                stand.off()?;
                println!("outputs=all_off");
            }
            "quit" | "exit" => {
                stand.off()?;
                println!("outputs=all_off; scanner stopped");
                return Ok(());
            }
            command => eprintln!("unknown command {command:?}; type help"),
        }
        prompt(&mut stdout)?;
    }

    stand.off()?;
    println!("outputs=all_off; scanner stopped");
    Ok(())
}

fn full_scan(
    stand: &mut StandRuntime<Pca9685>,
    camera: &Camera,
    detector: &CviTpuDetector,
    confidence: f32,
    iou: f32,
) -> Result<ScanResult> {
    if stand.state() != CommandedStandState::SafeOpen {
        bail!(
            "cannot start full scan from {}; use off, physically recover the cube if necessary, then restart",
            state_name(stand.state())
        );
    }

    let mut result = ScanResult::default();
    stand.grip()?;

    stand.scan_pose(ScanFace::Left)?;
    record_face(
        &mut result,
        ScanFace::Left,
        capture_face(camera, detector, confidence, iou)?,
    )?;

    for face in [
        ScanFace::Right,
        ScanFace::Down,
        ScanFace::Up,
        ScanFace::Front,
        ScanFace::Back,
    ] {
        stand.scan_next(face)?;
        record_face(
            &mut result,
            face,
            capture_face(camera, detector, confidence, iou)?,
        )?;
    }
    Ok(result)
}

fn capture_face(
    camera: &Camera,
    detector: &CviTpuDetector,
    confidence: f32,
    iou: f32,
) -> Result<Face> {
    let (_, rgb) = camera.capture_vpss_rgb()?;
    let input = preprocess::cube_roi_vpss_rgb(&rgb)?;
    let detections = detector.detect(&input)?;
    let detections = postprocess::filter_confidence(detections, confidence);
    let detections = postprocess::filter_center_window(detections, 320.0, 0.05, 0.90, 0.05, 0.95);
    let detections = postprocess::nms(detections, iou);
    let grid = grid::build_grid(&detections)?;
    let face = grid.to_face()?;
    println!("camera face: {}", face.compact());
    Ok(face)
}

#[derive(Default)]
struct ScanResult {
    state: CubeState,
    faces: [Option<Face>; 6],
}

fn record_face(result: &mut ScanResult, scan_face: ScanFace, face: Face) -> Result<()> {
    let logical_face = logical_face(scan_face);
    result.state.record_scan(
        ScanPose {
            face: logical_face,
            camera_to_face: QuarterTurns::Zero,
        },
        face,
    )?;
    result.faces[logical_face as usize] = Some(face);
    println!("scan {}: {}", logical_face.symbol(), face.compact());
    Ok(())
}

fn logical_face(face: ScanFace) -> LogicalFace {
    match face {
        ScanFace::Front => LogicalFace::Front,
        ScanFace::Left => LogicalFace::Left,
        ScanFace::Right => LogicalFace::Right,
        ScanFace::Up => LogicalFace::Up,
        ScanFace::Down => LogicalFace::Down,
        ScanFace::Back => LogicalFace::Back,
    }
}

fn print_scan_result(result: &ScanResult) -> Result<()> {
    let face = |logical: LogicalFace| -> Result<Face> {
        result.faces[logical as usize].with_context(|| format!("missing {} scan", logical.symbol()))
    };
    let up = face(LogicalFace::Up)?;
    let right = face(LogicalFace::Right)?;
    let front = face(LogicalFace::Front)?;
    let down = face(LogicalFace::Down)?;
    let left = face(LogicalFace::Left)?;
    let back = face(LogicalFace::Back)?;

    println!("color net (physical sticker colors):");
    print_single_face("         ", up);
    for row in 0..3 {
        println!(
            "{} | {} | {} | {}",
            face_row(left, row),
            face_row(front, row),
            face_row(right, row),
            face_row(back, row)
        );
    }
    print_single_face("         ", down);

    let color_facelet = [up, right, front, down, left, back]
        .into_iter()
        .map(Face::compact)
        .collect::<String>();
    println!("color facelet (URFDLB order): {color_facelet}");
    println!(
        "solver facelet (centers mapped to URFDLB): {}",
        result.state.facelet_string()?
    );
    Ok(())
}

fn print_single_face(indent: &str, face: Face) {
    for row in 0..3 {
        println!("{indent}{}", face_row(face, row));
    }
}

fn face_row(face: Face, row: usize) -> String {
    face.stickers()[row * 3..row * 3 + 3]
        .iter()
        .map(|color| color.symbol().to_string())
        .collect::<Vec<_>>()
        .join(" ")
}

fn prompt(stdout: &mut io::Stdout) -> Result<()> {
    write!(stdout, "> ").context("failed to write scanner prompt")?;
    stdout.flush().context("failed to flush scanner prompt")
}

fn state_name(state: CommandedStandState) -> &'static str {
    match state {
        CommandedStandState::Unknown => "unknown",
        CommandedStandState::OutputsOff => "outputs-off",
        CommandedStandState::SafeOpen => "safe-open",
        CommandedStandState::Gripped => "gripped",
        CommandedStandState::ScanHold(face) => face.name(),
        CommandedStandState::Faulted => "faulted",
    }
}

fn parse_address(value: &str) -> Result<u16, String> {
    let value = value.trim();
    let address = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .map_or_else(|| value.parse(), |hex| u16::from_str_radix(hex, 16))
        .map_err(|error| format!("invalid I²C address {value:?}: {error}"))?;
    if address > 0x7f {
        return Err(format!("I²C address must be 7-bit, got 0x{address:x}"));
    }
    Ok(address)
}
