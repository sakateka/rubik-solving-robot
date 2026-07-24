//! Minimal verification of the Milk-V Duo camera path: GC2083 -> VI -> ISP.
//!
//! This binary intentionally stops before VPSS and TPU. A successful probe
//! proves that the vendor sensor configuration and media-stack lifecycle work.

#[cfg(feature = "cvi-camera")]
use anyhow::{bail, Result};
#[cfg(feature = "cvi-camera")]
use clap::Parser;
#[cfg(feature = "cvi-camera")]
use std::{
    ffi::{c_char, c_int, CStr, CString},
    path::PathBuf,
};

#[cfg(feature = "cvi-camera")]
#[repr(C)]
struct RubikCameraFrameInfo {
    width: u32,
    height: u32,
    pixel_format: u32,
    stride: [u32; 3],
    length: [u32; 3],
}

#[cfg(feature = "cvi-camera")]
enum RubikCamera {}

#[cfg(feature = "cvi-camera")]
unsafe extern "C" {
    fn rubik_camera_open(
        sensor_config: *const c_char,
        error: *mut c_char,
        error_len: u32,
    ) -> *mut RubikCamera;
    fn rubik_camera_probe_frame(
        camera: *mut RubikCamera,
        info: *mut RubikCameraFrameInfo,
        error: *mut c_char,
        error_len: u32,
    ) -> c_int;
    fn rubik_camera_close(camera: *mut RubikCamera);
}

#[cfg(feature = "cvi-camera")]
#[derive(Parser)]
#[command(about = "Probe GC2083 through the CVI VI/ISP media stack")]
struct Cli {
    /// Path to the vendor sensor_cfg.ini (normally /mnt/data/sensor_cfg.ini)
    #[arg(long, default_value = "/mnt/data/sensor_cfg.ini")]
    sensor_config: PathBuf,
}

#[cfg(feature = "cvi-camera")]
fn error_text(buffer: &[c_char]) -> String {
    // The C adapter always writes a NUL-terminated diagnostic when it fails.
    unsafe { CStr::from_ptr(buffer.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}

#[cfg(feature = "cvi-camera")]
fn main() -> Result<()> {
    let cli = Cli::parse();
    let sensor_config = CString::new(cli.sensor_config.to_string_lossy().as_bytes())?;
    let mut error = [0 as c_char; 256];
    let camera = unsafe {
        rubik_camera_open(
            sensor_config.as_ptr(),
            error.as_mut_ptr(),
            error.len() as u32,
        )
    };
    if camera.is_null() {
        bail!("camera initialization failed: {}", error_text(&error));
    }

    let mut frame = RubikCameraFrameInfo {
        width: 0,
        height: 0,
        pixel_format: 0,
        stride: [0; 3],
        length: [0; 3],
    };
    let result = unsafe {
        rubik_camera_probe_frame(camera, &mut frame, error.as_mut_ptr(), error.len() as u32)
    };
    unsafe { rubik_camera_close(camera) };
    if result != 0 {
        bail!("camera frame capture failed: {}", error_text(&error));
    }

    println!(
        "VI frame: {}x{}, pixel_format={}, stride={:?}, length={:?}",
        frame.width, frame.height, frame.pixel_format, frame.stride, frame.length
    );
    Ok(())
}

#[cfg(not(feature = "cvi-camera"))]
fn main() {
    eprintln!("rubik-camera-probe requires --features cvi-camera in a Duo cross-build");
    std::process::exit(2);
}
