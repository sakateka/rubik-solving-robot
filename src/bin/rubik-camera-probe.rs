//! Minimal verification of the Milk-V Duo camera path: GC2083 → VI → ISP → VPSS.

#[cfg(feature = "cvi-camera")]
#[path = "../camera.rs"]
mod camera;

#[cfg(feature = "cvi-camera")]
use anyhow::{bail, Result};
#[cfg(feature = "cvi-camera")]
use camera::Camera;
#[cfg(feature = "cvi-camera")]
use clap::Parser;
#[cfg(feature = "cvi-camera")]
use std::{ffi::CString, fs::File, io::Write, path::PathBuf};

#[cfg(feature = "cvi-camera")]
#[derive(Parser)]
#[command(about = "Probe GC2083 through the CVI VI/ISP media stack")]
struct Cli {
    /// Path to the vendor sensor_cfg.ini (normally /mnt/data/sensor_cfg.ini)
    #[arg(long, default_value = "/mnt/data/sensor_cfg.ini")]
    sensor_config: PathBuf,

    /// Crop the fixed cube ROI and resize it to 320x320 through VPSS
    #[arg(long)]
    vpss: bool,

    /// VPSS frames to discard while sensor streaming and AE/AWB stabilize
    #[arg(long, default_value_t = 10)]
    warmup_frames: u32,

    /// Save the VPSS RGB-planar output as a binary PPM image for inspection
    #[arg(long)]
    dump_ppm: Option<PathBuf>,
}

#[cfg(feature = "cvi-camera")]
fn write_ppm(path: &PathBuf, planar_rgb: &[u8]) -> Result<()> {
    let pixels = (camera::MODEL_WIDTH * camera::MODEL_HEIGHT) as usize;
    if planar_rgb.len() != pixels * 3 {
        bail!("unexpected VPSS RGB layout while writing PPM");
    }
    let mut output = File::create(path)?;
    write!(
        output,
        "P6\n{} {}\n255\n",
        camera::MODEL_WIDTH,
        camera::MODEL_HEIGHT
    )?;
    for pixel in 0..pixels {
        output.write_all(&[
            planar_rgb[pixel],
            planar_rgb[pixels + pixel],
            planar_rgb[2 * pixels + pixel],
        ])?;
    }
    Ok(())
}

#[cfg(feature = "cvi-camera")]
fn main() -> Result<()> {
    let cli = Cli::parse();
    if cli.dump_ppm.is_some() && !cli.vpss {
        bail!("--dump-ppm requires --vpss");
    }
    let sensor_config = CString::new(cli.sensor_config.to_string_lossy().as_bytes())?;
    let camera = Camera::open(&sensor_config)?;
    if cli.vpss {
        camera.warmup_vpss(cli.warmup_frames)?;
    }
    let frame = if let Some(path) = &cli.dump_ppm {
        let (frame, rgb) = camera.capture_vpss_rgb()?;
        write_ppm(path, &rgb)?;
        eprintln!("wrote VPSS ROI to {}", path.display());
        frame
    } else {
        camera.probe(cli.vpss)?
    };

    println!(
        "{} frame: {}x{}, pixel_format={}, stride={:?}, length={:?}",
        if cli.vpss { "VPSS" } else { "VI" },
        frame.width,
        frame.height,
        frame.pixel_format,
        frame.stride,
        frame.length
    );
    Ok(())
}

#[cfg(not(feature = "cvi-camera"))]
fn main() {
    eprintln!("rubik-camera-probe requires --features cvi-camera in a Duo cross-build");
    std::process::exit(2);
}
