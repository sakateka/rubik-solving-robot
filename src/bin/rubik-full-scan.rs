//! Interactive end-to-end scan: stand choreography → camera → TPU → facelets.

use anyhow::{bail, Context, Result};
use clap::Parser;
use rubik_scan::{
    camera::Camera,
    cube::{CubeState, Face, LogicalFace, QuarterTurns, ScanPose, StickerColor},
    grid,
    model::Detector,
    pca9685::Pca9685,
    postprocess,
    postprocess::Detection,
    preprocess,
    stand::StandCalibration,
    stand_runtime::{CommandedStandState, ScanFace, StandRuntime},
    tpu::CviTpuDetector,
};
use std::{
    ffi::CString,
    fmt::Write as _,
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

    /// Directory where each full scan stores its six 320x320 ROI frames and predictions
    #[arg(long, default_value = "/mnt/storage/rubik-scan-records")]
    record_dir: PathBuf,

    /// Detections below this confidence are discarded
    #[arg(long, default_value_t = 0.5)]
    conf: f32,

    /// IoU threshold for class-agnostic NMS
    #[arg(long, default_value_t = 0.5)]
    iou: f32,

    /// Maximum number of moves requested from min2phase
    #[arg(long, default_value_t = 21)]
    max_moves: u8,

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
    println!("scanner ready; commands: scan, solve, state, off, quit");

    command_loop(
        &mut stand,
        &camera,
        &detector,
        cli.conf,
        cli.iou,
        cli.max_moves,
        &cli.record_dir,
    )
}

fn command_loop(
    stand: &mut StandRuntime<Pca9685>,
    camera: &Camera,
    detector: &CviTpuDetector,
    confidence: f32,
    iou: f32,
    max_moves: u8,
    record_dir: &PathBuf,
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

    let mut last_scan: Option<ScanResult> = None;
    let mut scan_number = 0_u32;
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
            "help" => println!("commands: scan, solve, state, off, quit"),
            "state" => println!("state={}", state_name(stand.state())),
            "scan" => {
                scan_number += 1;
                let mut recorder = match ScanRecorder::start(record_dir, scan_number) {
                    Ok(recorder) => Some(recorder),
                    Err(error) => {
                        eprintln!("scan recording disabled: {error:#}");
                        None
                    }
                };
                match full_scan(stand, camera, detector, confidence, iou, recorder.as_mut()) {
                    Ok(result) => match print_scan_result(&result) {
                        Ok(facelet) => {
                            if let Some(recorder) = recorder.as_mut() {
                                if let Err(error) = recorder.save_result(&result, Ok(&facelet)) {
                                    eprintln!("could not write scan result: {error:#}");
                                }
                            }
                            println!(
                                "scan record: {}",
                                recorder.as_ref().map_or_else(
                                    || "unavailable".to_owned(),
                                    |recorder| recorder.path.display().to_string()
                                )
                            );
                            last_scan = Some(result);
                            println!("scan accepted; enter solve to run min2phase");
                        }
                        Err(error) => {
                            if let Some(recorder) = recorder.as_mut() {
                                if let Err(write_error) = recorder.save_result(&result, Err(&error))
                                {
                                    eprintln!("could not write scan failure: {write_error:#}");
                                }
                            }
                            eprintln!("scan completed but facelet is invalid: {error:#}");
                        }
                    },
                    Err(error) => {
                        if let Some(recorder) = recorder.as_mut() {
                            if let Err(write_error) = recorder.save_failure(&error) {
                                eprintln!("could not write scan failure: {write_error:#}");
                            }
                        }
                        eprintln!("scan stopped: {error:#}");
                    }
                }
            }
            "solve" => match last_scan.as_ref() {
                Some(scan) => match scan.state.solve(max_moves) {
                    Ok(solution) => println!("solution ({max_moves} moves max): {solution}"),
                    Err(error) => eprintln!("min2phase rejected the saved facelet: {error:#}"),
                },
                None => eprintln!("no validated full scan is available; run scan first"),
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
    mut recorder: Option<&mut ScanRecorder>,
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
    let captured = capture_face(camera, detector, confidence, iou)?;
    record_face(
        &mut result,
        ScanFace::Left,
        captured,
        recorder.as_deref_mut(),
    )?;

    for face in [
        ScanFace::Right,
        ScanFace::Down,
        ScanFace::Up,
        ScanFace::Front,
        ScanFace::Back,
    ] {
        stand.scan_next(face)?;
        let captured = capture_face(camera, detector, confidence, iou)?;
        record_face(&mut result, face, captured, recorder.as_deref_mut())?;
    }
    stand.finish_scan()?;
    println!("stand=safe-open; cube returned to front-facing orientation");
    Ok(result)
}

fn capture_face(
    camera: &Camera,
    detector: &CviTpuDetector,
    confidence: f32,
    iou: f32,
) -> Result<CapturedFace> {
    let (_, rgb) = camera.capture_vpss_rgb()?;
    let input = preprocess::cube_roi_vpss_rgb(&rgb)?;
    let detections = detector.detect(&input)?;
    let detections = postprocess::filter_confidence(detections, confidence);
    let detections = postprocess::filter_center_window(detections, 320.0, 0.05, 0.90, 0.05, 0.95);
    let detections = postprocess::nms(detections, iou);
    let grid = grid::build_grid(&detections)?;
    let face = grid.to_face()?;
    println!("camera face: {}", face.compact());
    Ok(CapturedFace {
        face,
        rgb,
        detections,
    })
}

struct CapturedFace {
    face: Face,
    rgb: Vec<u8>,
    detections: Vec<Detection>,
}

#[derive(Default)]
struct ScanResult {
    state: CubeState,
    faces: [Option<Face>; 6],
}

struct ScanRecorder {
    path: PathBuf,
}

impl ScanRecorder {
    fn start(base: &std::path::Path, scan_number: u32) -> Result<Self> {
        std::fs::create_dir_all(base).with_context(|| {
            format!("could not create scan record directory {}", base.display())
        })?;
        let path = base.join(format!("scan-{}-{scan_number:03}", utc_timestamp()?));
        std::fs::create_dir(&path)
            .with_context(|| format!("could not create scan record session {}", path.display()))?;
        Ok(Self { path })
    }

    fn save_face(&self, logical_face: LogicalFace, captured: &CapturedFace) -> Result<()> {
        const PIXELS: usize = 320 * 320;
        if captured.rgb.len() != PIXELS * 3 {
            bail!(
                "unexpected RGB frame length {}; expected {}",
                captured.rgb.len(),
                PIXELS * 3
            );
        }
        let mut interleaved = Vec::with_capacity(captured.rgb.len());
        for pixel in 0..PIXELS {
            interleaved.extend_from_slice(&[
                captured.rgb[pixel],
                captured.rgb[PIXELS + pixel],
                captured.rgb[PIXELS * 2 + pixel],
            ]);
        }
        let image = image::RgbImage::from_raw(320, 320, interleaved)
            .context("could not construct RGB scan image")?;
        let name = logical_face.symbol();
        let image_path = self.path.join(format!("{name}.png"));
        image
            .save(&image_path)
            .with_context(|| format!("could not save {}", image_path.display()))?;

        let mut labels = String::new();
        for detection in &captured.detections {
            writeln!(
                labels,
                "{} {:.6} {:.6} {:.6} {:.6}",
                detection.class_id,
                detection.x / 320.0,
                detection.y / 320.0,
                detection.w / 320.0,
                detection.h / 320.0,
            )?;
        }
        let label_path = self.path.join(format!("{name}.yolo.txt"));
        std::fs::write(&label_path, labels)
            .with_context(|| format!("could not save {}", label_path.display()))?;
        let face_path = self.path.join(format!("{name}.face.txt"));
        std::fs::write(&face_path, format!("{}\n", captured.face.compact()))
            .with_context(|| format!("could not save {}", face_path.display()))
    }

    fn save_result(
        &self,
        result: &ScanResult,
        facelet: Result<&str, &anyhow::Error>,
    ) -> Result<()> {
        let mut report = String::from("predicted camera grids:\n");
        for logical_face in LogicalFace::ALL {
            let face = result.faces[logical_face as usize]
                .with_context(|| format!("missing {} scan", logical_face.symbol()))?;
            writeln!(report, "{}={}", logical_face.symbol(), face.compact())?;
        }
        match facelet {
            Ok(facelet) => writeln!(report, "solver_facelet={facelet}")?,
            Err(error) => writeln!(report, "validation_error={error:#}")?,
        }
        let path = self.path.join("result.txt");
        std::fs::write(&path, report).with_context(|| format!("could not save {}", path.display()))
    }

    fn save_failure(&self, error: &anyhow::Error) -> Result<()> {
        let path = self.path.join("failure.txt");
        std::fs::write(&path, format!("{error:#}\n"))
            .with_context(|| format!("could not save {}", path.display()))
    }
}

fn utc_timestamp() -> Result<String> {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .context("system clock is before the Unix epoch")?
        .as_secs();
    let days = (seconds / 86_400) as i64;
    let day_seconds = seconds % 86_400;
    let (year, month, day) = civil_date_from_unix_days(days);
    Ok(format!(
        "{year:04}-{month:02}-{day:02}_{:02}-{:02}-{:02}",
        day_seconds / 3_600,
        (day_seconds % 3_600) / 60,
        day_seconds % 60,
    ))
}

/// Gregorian calendar date for days since 1970-01-01, in UTC.
const fn civil_date_from_unix_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let day_of_era = z - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = month_prime + if month_prime < 10 { 3 } else { -9 };
    let year = year + if month <= 2 { 1 } else { 0 };
    (year, month as u32, day as u32)
}

fn record_face(
    result: &mut ScanResult,
    scan_face: ScanFace,
    captured: CapturedFace,
    recorder: Option<&mut ScanRecorder>,
) -> Result<()> {
    let logical_face = logical_face(scan_face);
    if let Some(recorder) = recorder {
        if let Err(error) = recorder.save_face(logical_face, &captured) {
            eprintln!("could not record {} scan: {error:#}", logical_face.symbol());
        }
    }
    result.state.record_scan(
        ScanPose {
            face: logical_face,
            camera_to_face: QuarterTurns::Zero,
        },
        captured.face,
    )?;
    result.faces[logical_face as usize] = Some(captured.face);
    println!(
        "scan {}: {}",
        logical_face.symbol(),
        captured.face.compact()
    );
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

fn print_scan_result(result: &ScanResult) -> Result<String> {
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
    print_color_counts([up, right, front, down, left, back]);
    let solver_facelet = result.state.facelet_string()?;
    println!("solver facelet (centers mapped to URFDLB): {solver_facelet}");
    Ok(solver_facelet)
}

fn print_color_counts(faces: [Face; 6]) {
    let mut counts = [0_u8; 6];
    for face in faces {
        for color in face.stickers() {
            counts[color_index(color)] += 1;
        }
    }
    println!(
        "color counts: W={} Y={} R={} O={} G={} B={}",
        counts[color_index(StickerColor::White)],
        counts[color_index(StickerColor::Yellow)],
        counts[color_index(StickerColor::Red)],
        counts[color_index(StickerColor::Orange)],
        counts[color_index(StickerColor::Green)],
        counts[color_index(StickerColor::Blue)],
    );
}

const fn color_index(color: StickerColor) -> usize {
    match color {
        StickerColor::White => 0,
        StickerColor::Yellow => 1,
        StickerColor::Red => 2,
        StickerColor::Orange => 3,
        StickerColor::Green => 4,
        StickerColor::Blue => 5,
    }
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
