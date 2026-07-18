//! Frame preprocessing for the YOLO input.
//!
//! Letterbox: downscale the frame keeping the aspect ratio and pad it to a
//! INPUT_SIZE x INPUT_SIZE square. Distorting the aspect ratio is not an
//! option — the boxes would shift. Pixels are normalized to 0..1 and laid
//! out as CHW = Channels, Height, Width (the whole R plane, then the whole
//! G plane, then B) — as opposed to the HWC layout a decoded image has
//! (interleaved R,G,B per pixel). CHW is what the model expects.

use crate::postprocess::Detection;
use image::{imageops, imageops::FilterType, Rgb, RgbImage};

/// Letterbox padding color. 114 is the YOLO convention (neutral gray).
const PAD_COLOR: Rgb<u8> = Rgb([114, 114, 114]);

/// Letterbox result: the model input tensor plus the transform parameters
/// needed to map boxes back to original frame coordinates.
pub struct Letterbox {
    /// CHW fp32, normalized to 0..1, size 3*size*size.
    /// Consumed by the real detector (the stub does not need it).
    #[allow(dead_code)]
    pub data: Vec<f32>,
    /// Side of the square model input (320)
    pub size: usize,
    /// Scale factor applied to the original frame
    pub scale: f32,
    /// Horizontal padding (left), in model-input pixels
    pub pad_x: f32,
    /// Vertical padding (top), in model-input pixels
    pub pad_y: f32,
}

impl Letterbox {
    /// Maps a box from model-input coordinates back to original frame
    /// coordinates.
    pub fn to_original(&self, det: &Detection) -> Detection {
        Detection {
            x: (det.x - self.pad_x) / self.scale,
            y: (det.y - self.pad_y) / self.scale,
            w: det.w / self.scale,
            h: det.h / self.scale,
            ..*det
        }
    }
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

    Letterbox {
        data,
        size,
        scale,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letterbox_scales_and_pads() {
        // 640x480 -> 320x240 + 40px padding top/bottom
        let img = RgbImage::new(640, 480);
        let lb = letterbox(&img, 320);
        assert_eq!(lb.scale, 0.5);
        assert_eq!(lb.pad_x, 0.0);
        assert_eq!(lb.pad_y, 40.0);
        assert_eq!(lb.data.len(), 3 * 320 * 320);
    }

    #[test]
    fn to_original_roundtrip() {
        let img = RgbImage::new(640, 480);
        let lb = letterbox(&img, 320);
        // The center of the model input must map to the center of the
        // original frame
        let det = Detection {
            x: 160.0,
            y: 160.0,
            w: 50.0,
            h: 50.0,
            class_id: 0,
            confidence: 1.0,
        };
        let orig = lb.to_original(&det);
        assert_eq!(orig.x, 320.0);
        assert_eq!(orig.y, 240.0);
        assert_eq!(orig.w, 100.0);
    }
}
