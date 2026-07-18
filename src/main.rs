//! Rubik's cube face scanner.
//!
//! Pipeline (see PLAN.md):
//!   frame -> letterbox preprocessing -> detector -> NMS -> 3x3 grid -> text.
//!
//! Stage 1: PC prototype, reads a photo from disk.

mod capture;
mod grid;
mod model;
mod postprocess;
mod preprocess;
mod yolo_v8;

use anyhow::{bail, Result};
use clap::Parser;
use model::Detector;
use std::path::PathBuf;

#[derive(Parser)]
#[command(about = "Rubik's cube face scanner: photo -> 3x3 colors as text")]
struct Cli {
    /// Path to the face photo
    #[arg(long)]
    image: PathBuf,

    /// Use the stub instead of a real model (no weights needed)
    #[arg(long)]
    stub: bool,

    /// Path to YOLOv8n weights (.safetensors)
    #[arg(long)]
    model: Option<PathBuf>,

    /// Number of model classes: 80 = pretrained COCO, 6 = cube colors
    #[arg(long, default_value_t = 80)]
    classes: usize,

    /// Confidence threshold: detections below it are discarded
    #[arg(long, default_value_t = 0.5)]
    conf: f32,

    /// IoU (intersection over union) threshold for NMS (non-maximum
    /// suppression): boxes overlapping more than this are treated as
    /// duplicates, and only the most confident one survives
    #[arg(long, default_value_t = 0.45)]
    iou: f32,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    if cli.stub == cli.model.is_some() {
        bail!("specify exactly one of --stub or --model <path>");
    }
    // The stub plays the role of the future 6-class cube-color model.
    let num_classes = if cli.stub { 6 } else { cli.classes };

    // 1. Frame
    let img = capture::load_from_file(&cli.image)?;

    // 2. Preprocessing: letterbox to model input size + normalization
    let input = preprocess::letterbox(&img, model::INPUT_SIZE);

    // 3. Detector (boxes come in model-input coordinates)
    let detections = if cli.stub {
        model::StubDetector::default().detect(&input)?
    } else {
        let detector = model::YoloV8Detector::load(cli.model.as_deref().unwrap(), num_classes)?;
        detector.detect(&input)?
    };

    // Convert boxes back to original frame coordinates
    let detections: Vec<_> = detections.iter().map(|d| input.to_original(d)).collect();

    // 4. Postprocessing
    let detections = postprocess::filter_confidence(detections, cli.conf);
    let detections = postprocess::nms(detections, cli.iou);
    println!("detections after NMS: {}", detections.len());
    for d in &detections {
        println!(
            "  {:<14} conf={:.2} box=({:.0},{:.0} {:.0}x{:.0})",
            model::class_name(d.class_id, num_classes),
            d.confidence,
            d.x,
            d.y,
            d.w,
            d.h
        );
    }

    // 5. 3x3 grid -> text (only makes sense for the 6-class cube model)
    if num_classes == 6 {
        let face = grid::build_grid(&detections)?;
        println!("{face}");
        println!("compact: {}", face.to_compact_string());
    }

    Ok(())
}
