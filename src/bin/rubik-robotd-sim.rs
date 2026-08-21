//! Multi-session simulated robot daemon. Each browser tab owns an independent
//! protocol service, stand, scanner, cube state and safety monitor.

use anyhow::Result;
use rubik_scan::{sim_server::run_sim_server, stand::StandCalibration};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut addr = "127.0.0.1:8080".to_owned();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--addr" => {
                addr = args.next().expect("--addr requires a value");
            }
            other => anyhow::bail!("unknown argument {other:?}"),
        }
    }

    let calibration = StandCalibration::default();
    eprintln!("rubik-robotd-sim ready; isolated server-side browser sessions enabled");
    run_sim_server(&addr, calibration)
}
