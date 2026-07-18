//! Frame capture.
//!
//! Stage 1 (PC prototype): read a photo from disk.
//! Stage 3 (device): capture from GC2083 on Milk-V Duo — vendor VI/VPSS
//! or v4l2, depending on what the firmware exposes (see PLAN.md, section 9).

use anyhow::{Context, Result};
use image::RgbImage;
use std::path::Path;

/// Loads an image from disk and converts it to RGB8.
pub fn load_from_file(path: &Path) -> Result<RgbImage> {
    let img =
        image::open(path).with_context(|| format!("failed to open image: {}", path.display()))?;
    Ok(img.to_rgb8())
}
