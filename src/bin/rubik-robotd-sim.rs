//! Simulated robot daemon: the full protocol stack of the hardware daemon,
//! but the stand is a 3D web UI instead of PCA9685 servos and the camera is
//! replaced by a scanner that always recognizes a solved cube. An optional
//! UART can still be attached for protocol-level testing.

use anyhow::Result;
use rubik_scan::{
    pca9685::PwmOutput,
    robot_daemon::{run_uart_daemon, DaemonHub, UartDaemonOptions},
    robot_service::{FaceScanner, RobotService},
    sim_server::{run_sim_server, SimEngine, SimUpdate},
    stand::StandCalibration,
};
use rubik_link_protocol as link;
use std::path::Path;
use std::sync::mpsc;

/// No-op PWM backend; the web UI renders stand motion from status snapshots.
#[derive(Default)]
struct SimPwmOutput;

impl PwmOutput for SimPwmOutput {
    fn set_channels(&mut self, _channels: &[(u8, u16)]) -> Result<()> {
        Ok(())
    }

    fn disable_channels(&mut self, _channels: &[u8]) -> Result<()> {
        Ok(())
    }

    fn all_off(&mut self) -> Result<()> {
        Ok(())
    }
}

/// Always recognizes a solved cube face with perfect confidence.
struct SolvedCubeScanner;

const SOLVED_FACES: [[link::StickerColor; 9]; 6] = [
    [link::StickerColor::White; 9],    // Up
    [link::StickerColor::Red; 9],      // Right
    [link::StickerColor::Green; 9],    // Front
    [link::StickerColor::Yellow; 9],   // Down
    [link::StickerColor::Orange; 9],   // Left
    [link::StickerColor::Blue; 9],     // Back
];

impl FaceScanner for SolvedCubeScanner {
    fn capture(&mut self, face: link::CubeFace) -> Result<link::RecognizedFace> {
        Ok(link::RecognizedFace {
            colors: SOLVED_FACES[face as usize],
            confidence: [255; 9],
        })
    }
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut addr = "127.0.0.1:8080".to_owned();
    let mut uart_device = None;
    let mut skip_uart_config = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--addr" => {
                addr = args.next().expect("--addr requires a value");
            }
            "--uart" => {
                uart_device = Some(args.next().expect("--uart requires a device path"));
            }
            "--skip-uart-config" => skip_uart_config = true,
            other => anyhow::bail!("unknown argument {other:?}"),
        }
    }

    let calibration = StandCalibration::default();
    let service = RobotService::with_scanner(SimPwmOutput, calibration.clone(), SolvedCubeScanner);

    let (update_tx, update_rx) = mpsc::channel::<SimUpdate>();
    let (hub, inbound) = DaemonHub::new(Box::new(SimEngine::new(update_tx)));

    let server_addr = addr.clone();
    let server_thread = std::thread::spawn(move || {
        if let Err(error) = run_sim_server(&server_addr, update_rx, inbound, &calibration) {
            eprintln!("simulation server failed: {error:#}");
            std::process::exit(1);
        }
    });

    let result = run_uart_daemon(
        UartDaemonOptions {
            process_name: "rubik-robotd-sim",
            uart_device: uart_device.as_deref().map(Path::new),
            skip_uart_config,
            hub: Some(hub),
        },
        service,
    );
    let _ = server_thread.join();
    result
}
