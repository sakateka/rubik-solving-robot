//! Safe-ish Rust facade for the intentionally tiny C CVI Runtime adapter.
//!
//! The vendor headers stay in `native/`; no generated bindgen layer leaks
//! CVI-specific structs into the rest of the Rust application.

use crate::{model::Detector, postprocess::Detection, preprocess::Letterbox};
use anyhow::{bail, Context, Result};
use std::{
    ffi::{c_char, c_int, CString},
    path::Path,
    ptr::NonNull,
};

const INPUT_FLOATS: usize = 3 * 320 * 320;
const OUTPUT_FLOATS: usize = 10 * 2100;
const ANCHORS: usize = 2100;
const CLASSES: usize = 6;

#[repr(C)]
struct RawTpu {
    _private: [u8; 0],
}

#[link(name = "rubik_cvi_tpu", kind = "static")]
unsafe extern "C" {
    fn rubik_cvi_tpu_open(
        model_path: *const c_char,
        out: *mut *mut RawTpu,
        error: *mut c_char,
        error_size: usize,
    ) -> c_int;
    fn rubik_cvi_tpu_close(tpu: *mut RawTpu);
    fn rubik_cvi_tpu_input_len(tpu: *const RawTpu) -> usize;
    fn rubik_cvi_tpu_output_len(tpu: *const RawTpu) -> usize;
    fn rubik_cvi_tpu_forward(
        tpu: *mut RawTpu,
        input: *const f32,
        input_len: usize,
        output: *mut f32,
        output_len: usize,
        error: *mut c_char,
        error_size: usize,
    ) -> c_int;
}

pub struct CviTpuDetector {
    raw: NonNull<RawTpu>,
}

impl CviTpuDetector {
    pub fn load(model_path: &Path) -> Result<Self> {
        let model_path = CString::new(model_path.as_os_str().as_encoded_bytes())
            .context("cvimodel path contains a NUL byte")?;
        let mut raw = std::ptr::null_mut();
        let mut error = [0 as c_char; 256];
        // SAFETY: pointers remain valid during the call; C creates one owned handle.
        let rc = unsafe {
            rubik_cvi_tpu_open(
                model_path.as_ptr(),
                &mut raw,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        if rc != 0 {
            bail!("failed to open CVI model: {}", error_text(&error));
        }
        let raw = NonNull::new(raw).context("CVI Runtime returned a null model handle")?;
        // SAFETY: `raw` came from the successful open call above.
        let (input_len, output_len) = unsafe {
            (
                rubik_cvi_tpu_input_len(raw.as_ptr()),
                rubik_cvi_tpu_output_len(raw.as_ptr()),
            )
        };
        if input_len != INPUT_FLOATS || output_len != OUTPUT_FLOATS {
            // SAFETY: ownership has not been transferred and this is the matching destructor.
            unsafe { rubik_cvi_tpu_close(raw.as_ptr()) };
            bail!(
                "unexpected cvimodel tensors: input {input_len} (expected {INPUT_FLOATS}), \
                 output {output_len} (expected {OUTPUT_FLOATS})"
            );
        }
        Ok(Self { raw })
    }

    fn forward(&self, input: &[f32]) -> Result<Vec<f32>> {
        if input.len() != INPUT_FLOATS {
            bail!(
                "TPU input has {} floats, expected {INPUT_FLOATS}",
                input.len()
            );
        }
        let mut output = vec![0.0; OUTPUT_FLOATS];
        let mut error = [0 as c_char; 256];
        // SAFETY: the adapter neither retains the input/output pointers nor outlives `self`.
        let rc = unsafe {
            rubik_cvi_tpu_forward(
                self.raw.as_ptr(),
                input.as_ptr(),
                input.len(),
                output.as_mut_ptr(),
                output.len(),
                error.as_mut_ptr(),
                error.len(),
            )
        };
        if rc != 0 {
            bail!("CVI TPU inference failed: {}", error_text(&error));
        }
        Ok(output)
    }
}

impl Detector for CviTpuDetector {
    fn detect(&self, input: &Letterbox) -> Result<Vec<Detection>> {
        let output = self.forward(&input.data)?;
        let mut detections = Vec::with_capacity(ANCHORS);
        // Runtime uses [1, 10, 2100, 1], contiguous in channel-major order.
        for anchor in 0..ANCHORS {
            let (class_id, confidence) = (0..CLASSES)
                .map(|class_id| (class_id, output[(4 + class_id) * ANCHORS + anchor]))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .expect("six cube classes are non-empty");
            detections.push(Detection {
                x: output[anchor],
                y: output[ANCHORS + anchor],
                w: output[2 * ANCHORS + anchor],
                h: output[3 * ANCHORS + anchor],
                class_id,
                confidence,
            });
        }
        Ok(detections)
    }
}

impl Drop for CviTpuDetector {
    fn drop(&mut self) {
        // SAFETY: `raw` is created only by open and is dropped once here.
        unsafe { rubik_cvi_tpu_close(self.raw.as_ptr()) };
    }
}

fn error_text(error: &[c_char]) -> String {
    let bytes: Vec<u8> = error
        .iter()
        .map(|&c| c as u8)
        .take_while(|&c| c != 0)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}
