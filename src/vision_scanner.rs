//! Production GC2083 + CV181X TPU backend for the stateful robot daemon.

use crate::{
    camera::Camera,
    model::{Detector, CLASS_COLORS},
    postprocess::{self, Detection},
    preprocess,
    robot_service::FaceScanner,
    tpu::CviTpuDetector,
};
use anyhow::{bail, Context, Result};
use rubik_link_protocol as link;
use std::{
    ffi::CString,
    fmt::Write as _,
    path::{Path, PathBuf},
    process::Command,
};

pub struct VisionScanner {
    camera: Camera,
    detector: CviTpuDetector,
    confidence: f32,
    iou: f32,
    record_dir: PathBuf,
    active_record: Option<PathBuf>,
}

impl VisionScanner {
    pub fn open(
        sensor_config: &Path,
        warmup_frames: u32,
        cvimodel: &Path,
        confidence: f32,
        iou: f32,
        record_dir: PathBuf,
    ) -> Result<Self> {
        let sensor_config = CString::new(sensor_config.as_os_str().as_encoded_bytes())
            .context("sensor config path contains a NUL byte")?;
        let camera = Camera::open(&sensor_config)?;
        camera.warmup_vpss(warmup_frames)?;
        let detector = CviTpuDetector::load(cvimodel)?;
        Ok(Self {
            camera,
            detector,
            confidence,
            iou,
            record_dir,
            active_record: None,
        })
    }

    fn save_artifacts(
        &self,
        face: link::CubeFace,
        rgb: &[u8],
        detections: &[Detection],
        recognized: link::RecognizedFace,
    ) -> Result<()> {
        let Some(directory) = &self.active_record else {
            return Ok(());
        };
        const PIXELS: usize = 320 * 320;
        if rgb.len() != PIXELS * 3 {
            bail!("unexpected RGB frame length {}", rgb.len());
        }
        let mut interleaved = Vec::with_capacity(rgb.len());
        for pixel in 0..PIXELS {
            interleaved.extend_from_slice(&[
                rgb[pixel],
                rgb[PIXELS + pixel],
                rgb[2 * PIXELS + pixel],
            ]);
        }
        let name = face_name(face);
        image::RgbImage::from_raw(320, 320, interleaved)
            .context("could not construct scan image")?
            .save(directory.join(format!("{name}.png")))?;

        let mut labels = String::new();
        for detection in detections {
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
        std::fs::write(directory.join(format!("{name}.yolo.txt")), labels)?;
        let symbols = recognized
            .colors
            .into_iter()
            .map(color_symbol)
            .collect::<String>();
        std::fs::write(
            directory.join(format!("{name}.face.txt")),
            format!("{symbols}\n"),
        )?;
        Ok(())
    }
}

impl FaceScanner for VisionScanner {
    fn begin_scan(&mut self, revision: u32) -> Result<()> {
        std::fs::create_dir_all(&self.record_dir)?;
        let directory = self
            .record_dir
            .join(format!("scan-{}-r{revision}", utc_timestamp()?));
        std::fs::create_dir(&directory)?;
        self.active_record = Some(directory);
        Ok(())
    }

    fn capture(&mut self, face: link::CubeFace) -> Result<link::RecognizedFace> {
        let (_, rgb) = self.camera.capture_vpss_rgb()?;
        let input = preprocess::cube_roi_vpss_rgb(&rgb)?;
        let detections = self.detector.detect(&input)?;
        let detections = postprocess::filter_confidence(detections, self.confidence);
        let detections =
            postprocess::filter_center_window(detections, 320.0, 0.05, 0.90, 0.05, 0.95);
        let detections = postprocess::nms(detections, self.iou);
        let recognized = recognized_face(&detections)?;
        self.save_artifacts(face, &rgb, &detections, recognized)?;
        sync_filesystems()?;
        Ok(recognized)
    }

    fn finish_scan(&mut self, status: &link::ScanStatus) -> Result<()> {
        let Some(directory) = self.active_record.take() else {
            return Ok(());
        };
        let mut report = format!(
            "state={:?}\nrevision={:?}\nscanned_faces=0b{:06b}\ncolor_counts={:?}\nvalidation_error={:?}\n",
            status.state,
            status.revision,
            status.scanned_faces,
            status.color_counts,
            status.validation_error,
        );
        for face in [
            link::CubeFace::Up,
            link::CubeFace::Right,
            link::CubeFace::Front,
            link::CubeFace::Down,
            link::CubeFace::Left,
            link::CubeFace::Back,
        ] {
            if let Some(recognized) = status.faces[face as usize] {
                let symbols = recognized
                    .colors
                    .into_iter()
                    .map(color_symbol)
                    .collect::<String>();
                writeln!(report, "{}={symbols}", face_name(face))?;
            }
        }
        std::fs::write(directory.join("result.txt"), report)?;
        Ok(())
    }

    fn abort(&mut self) {
        self.active_record = None;
    }
}

fn sync_filesystems() -> Result<()> {
    let status = Command::new("sync")
        .status()
        .context("failed to run sync after saving scan face")?;
    if !status.success() {
        bail!("sync failed after saving scan face with status {status}");
    }
    Ok(())
}

fn recognized_face(detections: &[Detection]) -> Result<link::RecognizedFace> {
    if detections.len() != link::STICKERS_PER_FACE {
        bail!("expected 9 detections, got {}", detections.len());
    }
    let mut rows = detections.to_vec();
    rows.sort_by(|a, b| a.y.total_cmp(&b.y));
    let mut colors = [link::StickerColor::Unknown; link::STICKERS_PER_FACE];
    let mut confidence = [0; link::STICKERS_PER_FACE];
    for (row_index, row) in rows.chunks_mut(3).enumerate() {
        row.sort_by(|a, b| a.x.total_cmp(&b.x));
        for (column, detection) in row.iter().enumerate() {
            let index = row_index * 3 + column;
            colors[index] = class_color(detection.class_id)?;
            confidence[index] = (detection.confidence.clamp(0.0, 1.0) * 255.0).round() as u8;
        }
    }
    Ok(link::RecognizedFace { colors, confidence })
}

fn class_color(class_id: usize) -> Result<link::StickerColor> {
    let symbol = *CLASS_COLORS
        .get(class_id)
        .with_context(|| format!("unknown class id {class_id}"))?;
    match symbol {
        'W' => Ok(link::StickerColor::White),
        'Y' => Ok(link::StickerColor::Yellow),
        'R' => Ok(link::StickerColor::Red),
        'O' => Ok(link::StickerColor::Orange),
        'G' => Ok(link::StickerColor::Green),
        'B' => Ok(link::StickerColor::Blue),
        _ => bail!("unsupported class color {symbol}"),
    }
}

fn color_symbol(color: link::StickerColor) -> char {
    match color {
        link::StickerColor::White => 'W',
        link::StickerColor::Yellow => 'Y',
        link::StickerColor::Red => 'R',
        link::StickerColor::Orange => 'O',
        link::StickerColor::Green => 'G',
        link::StickerColor::Blue => 'B',
        link::StickerColor::Unknown => '?',
    }
}

fn face_name(face: link::CubeFace) -> char {
    match face {
        link::CubeFace::Up => 'U',
        link::CubeFace::Right => 'R',
        link::CubeFace::Front => 'F',
        link::CubeFace::Down => 'D',
        link::CubeFace::Left => 'L',
        link::CubeFace::Back => 'B',
    }
}

fn utc_timestamp() -> Result<String> {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)?
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
