//! Safe camera facade for the Milk-V Duo VI/ISP/VPSS pipeline.
//!
//! The C adapter owns the vendor media-stack lifecycle.  This module turns
//! its opaque handle and RGB-planar u8 output into ordinary Rust values.

use anyhow::{anyhow, bail, Result};
use std::{
    ffi::{c_char, c_int, CString},
    ptr::NonNull,
};

pub const MODEL_WIDTH: u32 = 320;
pub const MODEL_HEIGHT: u32 = 320;
const RGB_BYTES: usize = MODEL_WIDTH as usize * MODEL_HEIGHT as usize * 3;

#[repr(C)]
pub struct FrameInfo {
    pub width: u32,
    pub height: u32,
    pub pixel_format: u32,
    pub stride: [u32; 3],
    pub length: [u32; 3],
}

impl FrameInfo {
    fn empty() -> Self {
        Self {
            width: 0,
            height: 0,
            pixel_format: 0,
            stride: [0; 3],
            length: [0; 3],
        }
    }
}

enum RawCamera {}

/// Owns one initialized GC2083 → VI → ISP → VPSS pipeline.
pub struct Camera(NonNull<RawCamera>);

#[link(name = "rubik_cvi_camera", kind = "static")]
unsafe extern "C" {
    fn rubik_camera_open(
        sensor_config: *const c_char,
        error: *mut c_char,
        error_len: u32,
    ) -> *mut RawCamera;
    #[allow(dead_code)] // used by the standalone camera-probe binary
    fn rubik_camera_probe_frame(
        camera: *mut RawCamera,
        info: *mut FrameInfo,
        error: *mut c_char,
        error_len: u32,
    ) -> c_int;
    #[allow(dead_code)] // used by the standalone camera-probe binary
    fn rubik_camera_probe_vpss_frame(
        camera: *mut RawCamera,
        info: *mut FrameInfo,
        error: *mut c_char,
        error_len: u32,
    ) -> c_int;
    fn rubik_camera_warmup_vpss(
        camera: *mut RawCamera,
        frame_count: u32,
        error: *mut c_char,
        error_len: u32,
    ) -> c_int;
    fn rubik_camera_copy_vpss_rgb(
        camera: *mut RawCamera,
        output: *mut u8,
        output_len: u32,
        info: *mut FrameInfo,
        error: *mut c_char,
        error_len: u32,
    ) -> c_int;
    fn rubik_camera_close(camera: *mut RawCamera);
}

impl Camera {
    pub fn open(sensor_config: &CString) -> Result<Self> {
        let mut error = [0 as c_char; 256];
        // SAFETY: `sensor_config` and `error` remain valid during this call;
        // on success C returns one owned handle accepted by its close function.
        let raw = unsafe {
            rubik_camera_open(
                sensor_config.as_ptr(),
                error.as_mut_ptr(),
                error.len() as u32,
            )
        };
        NonNull::new(raw)
            .map(Self)
            .ok_or_else(|| anyhow!("camera initialization failed: {}", error_text(&error)))
    }

    #[allow(dead_code)] // the probe binary reuses this module by source path
    pub fn probe(&self, use_vpss: bool) -> Result<FrameInfo> {
        let mut frame = FrameInfo::empty();
        let mut error = [0 as c_char; 256];
        // SAFETY: this Camera owns a live handle; `frame` and `error` are
        // writable buffers valid for the duration of the call.
        let result = unsafe {
            if use_vpss {
                rubik_camera_probe_vpss_frame(
                    self.0.as_ptr(),
                    &mut frame,
                    error.as_mut_ptr(),
                    error.len() as u32,
                )
            } else {
                rubik_camera_probe_frame(
                    self.0.as_ptr(),
                    &mut frame,
                    error.as_mut_ptr(),
                    error.len() as u32,
                )
            }
        };
        if result != 0 {
            bail!("camera frame capture failed: {}", error_text(&error));
        }
        Ok(frame)
    }

    pub fn warmup_vpss(&self, frame_count: u32) -> Result<()> {
        if frame_count == 0 {
            return Ok(());
        }
        let mut error = [0 as c_char; 256];
        // SAFETY: this Camera owns a live handle and `error` is writable for
        // the duration of the call.
        let result = unsafe {
            rubik_camera_warmup_vpss(
                self.0.as_ptr(),
                frame_count,
                error.as_mut_ptr(),
                error.len() as u32,
            )
        };
        if result != 0 {
            bail!("VPSS warm-up failed: {}", error_text(&error));
        }
        Ok(())
    }

    /// Captures one 320×320 RGB-planar frame as tightly packed CHW u8.
    pub fn capture_vpss_rgb(&self) -> Result<(FrameInfo, Vec<u8>)> {
        let mut frame = FrameInfo::empty();
        let mut rgb = vec![0_u8; RGB_BYTES];
        let mut error = [0 as c_char; 256];
        // SAFETY: this Camera owns a live handle; the supplied output buffers
        // are valid and their exact length is passed to the C adapter.
        let result = unsafe {
            rubik_camera_copy_vpss_rgb(
                self.0.as_ptr(),
                rgb.as_mut_ptr(),
                rgb.len() as u32,
                &mut frame,
                error.as_mut_ptr(),
                error.len() as u32,
            )
        };
        if result != 0 {
            bail!("VPSS RGB capture failed: {}", error_text(&error));
        }
        if frame.width != MODEL_WIDTH || frame.height != MODEL_HEIGHT {
            bail!(
                "unexpected VPSS dimensions: {}x{} (expected {}x{})",
                frame.width,
                frame.height,
                MODEL_WIDTH,
                MODEL_HEIGHT
            );
        }
        Ok((frame, rgb))
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        // SAFETY: this wrapper owns the handle exactly once and Drop runs
        // after all safe methods have stopped using it.
        unsafe { rubik_camera_close(self.0.as_ptr()) };
    }
}

fn error_text(buffer: &[c_char]) -> String {
    let bytes: Vec<u8> = buffer
        .iter()
        .map(|&value| value as u8)
        .take_while(|&value| value != 0)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}
