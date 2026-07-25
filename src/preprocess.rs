//! Frame preprocessing for the YOLO input.
//!
//! Letterbox: downscale the frame keeping the aspect ratio and pad it to a
//! INPUT_SIZE x INPUT_SIZE square. Distorting the aspect ratio is not an
//! option — the boxes would shift. Pixels are normalized to 0..1 and laid
//! out as CHW = Channels, Height, Width (the whole R plane, then the whole
//! G plane, then B) — as opposed to the HWC layout a decoded image has
//! (interleaved R,G,B per pixel). CHW is what the model expects.

use image::{imageops, imageops::FilterType, Rgb, RgbImage};

/// Letterbox padding color. 114 is the YOLO convention (neutral gray).
const PAD_COLOR: Rgb<u8> = Rgb([114, 114, 114]);

/// Letterbox result: the model input tensor.
pub struct Letterbox {
    /// CHW fp32, normalized to 0..1, size 3*size*size.
    /// Consumed by the real detector (the stub does not need it).
    #[allow(dead_code)]
    pub data: Vec<f32>,
    /// Side of the square model input (320)
    pub size: usize,
}

/// Letterboxes frame `img` to a `size` x `size` square.
pub fn letterbox(img: &RgbImage, size: usize) -> Letterbox {
    let (w, h) = (img.width() as f32, img.height() as f32);
    let scale = (size as f32 / w).min(size as f32 / h);

    let new_w = ((w * scale).round() as u32).max(1);
    let new_h = ((h * scale).round() as u32).max(1);
    let resized = imageops::resize(img, new_w, new_h, FilterType::Triangle);

    let pad_x = (size as u32 - new_w) / 2;
    let pad_y = (size as u32 - new_h) / 2;

    let mut canvas = RgbImage::from_pixel(size as u32, size as u32, PAD_COLOR);
    imageops::overlay(&mut canvas, &resized, pad_x as i64, pad_y as i64);

    // HWC u8 -> CHW f32, normalized to 0..1
    let plane = size * size;
    let mut data = vec![0.0f32; 3 * plane];
    for (i, px) in canvas.pixels().enumerate() {
        data[i] = px[0] as f32 / 255.0; // R
        data[plane + i] = px[1] as f32 / 255.0; // G
        data[2 * plane + i] = px[2] as f32 / 255.0; // B
    }

    Letterbox { data, size }
}

/// The crop used for both training and device inference. It intentionally has
/// almost no horizontal background margin: an orange part of the rig used to
/// produce false positives when the whole 1920x1080 frame was shown.
#[cfg_attr(not(feature = "cvi-runtime"), allow(dead_code))]
pub const CUBE_ROI: (u32, u32, u32, u32) = (464, 32, 1296, 864);

/// Crop the fixed square cube ROI and resize it directly to the model input.
/// This is deliberately *not* letterbox: training saw a square 832x832 crop
/// scaled to 320x320 without padding.
#[cfg_attr(not(feature = "cvi-runtime"), allow(dead_code))]
pub fn cube_roi_resize(img: &RgbImage, size: usize) -> anyhow::Result<Letterbox> {
    let (left, top, right, bottom) = CUBE_ROI;
    if img.width() < right || img.height() < bottom {
        anyhow::bail!(
            "frame {}x{} is too small for cube ROI x={}..{}, y={}..{}",
            img.width(),
            img.height(),
            left,
            right,
            top,
            bottom
        );
    }
    let roi = imageops::crop_imm(img, left, top, right - left, bottom - top).to_image();
    let resized = imageops::resize(&roi, size as u32, size as u32, FilterType::Triangle);
    let plane = size * size;
    let mut data = vec![0.0f32; 3 * plane];
    for (i, px) in resized.pixels().enumerate() {
        data[i] = px[0] as f32 / 255.0;
        data[plane + i] = px[1] as f32 / 255.0;
        data[2 * plane + i] = px[2] as f32 / 255.0;
    }
    Ok(Letterbox { data, size })
}

/// Converts the VPSS output directly to the TPU input tensor.
///
/// VPSS has already applied the exact training crop `(464, 32)..(1296, 864)`
/// and resized it to 320×320 RGB planar (CHW).  The only remaining operation
/// is u8 → fp32 normalization.  Keeping this separate from `cube_roi_resize`
/// makes it explicit that camera inference does not resize a frame twice.
#[cfg_attr(not(feature = "cvi-camera"), allow(dead_code))]
pub fn cube_roi_vpss_rgb(planar_rgb: &[u8]) -> anyhow::Result<Letterbox> {
    let size = model_size();
    let plane = size * size;
    if planar_rgb.len() != 3 * plane {
        anyhow::bail!(
            "VPSS RGB buffer has {} bytes, expected {}",
            planar_rgb.len(),
            3 * plane
        );
    }
    let data = planar_rgb
        .iter()
        .map(|&pixel| pixel as f32 / 255.0)
        .collect();
    Ok(Letterbox { data, size })
}

fn model_size() -> usize {
    crate::model::INPUT_SIZE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letterbox_scales_and_pads() {
        // 640x480 -> 320x240 + 40px padding top/bottom
        let img = RgbImage::new(640, 480);
        let lb = letterbox(&img, 320);
        assert_eq!(lb.data.len(), 3 * 320 * 320);
    }

    #[test]
    fn vpss_tensor_is_chw_and_uses_cube_roi_mapping() {
        let mut rgb = vec![0; 3 * 320 * 320];
        rgb[0] = 255;
        rgb[320 * 320] = 128;
        let input = cube_roi_vpss_rgb(&rgb).unwrap();
        assert_eq!(input.data[0], 1.0);
        assert_eq!(input.data[320 * 320], 128.0 / 255.0);
    }
}
