//! Rubik's cube face scanner.
//!
//! Pipeline (see PROJECT_NOTES.md):
//!   frame -> letterbox preprocessing -> detector -> NMS -> 3x3 grid -> text.
//!
//! Stage 1: PC prototype, reads a photo from disk.

mod capture;
mod grid;
mod model;
mod postprocess;
mod preprocess;
#[cfg(feature = "cvi-runtime")]
mod tpu;
mod yolo_v8;

use anyhow::{bail, Result};
use clap::Parser;
use model::Detector;
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

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

    /// Path to the CV181X .cvimodel. Requires a Milk-V cross-build with
    /// `--features cvi-runtime`; this is the production detector backend.
    #[arg(long)]
    cvimodel: Option<PathBuf>,

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

    /// Print decode, preprocessing, model-load, inference and postprocess timings to stderr
    #[arg(long)]
    timings: bool,
}

fn main() -> Result<()> {
    let total_started = Instant::now();
    let cli = Cli::parse();

    let backend_count = usize::from(cli.stub)
        + usize::from(cli.model.is_some())
        + usize::from(cli.cvimodel.is_some());
    if backend_count != 1 {
        bail!("specify exactly one of --stub, --model <path>, or --cvimodel <path>");
    }

    // 1. Frame
    let phase_started = Instant::now();
    let img = capture::load_from_file(&cli.image)?;
    let decode_time = phase_started.elapsed();

    // 2–3. The old PC prototype uses full-frame letterbox. The production TPU
    // model uses the exact crop→resize preprocessing from its training set.
    let (
        input,
        detections,
        num_classes,
        use_center_window,
        preprocess_time,
        model_load_time,
        inference_time,
    ) = if cli.stub {
        let phase_started = Instant::now();
        let input = preprocess::letterbox(&img, model::INPUT_SIZE);
        let preprocess_time = phase_started.elapsed();
        let phase_started = Instant::now();
        let detections = model::StubDetector::default().detect(&input)?;
        let inference_time = phase_started.elapsed();
        (
            input,
            detections,
            6,
            false,
            preprocess_time,
            Duration::ZERO,
            inference_time,
        )
    } else if let Some(cvimodel) = cli.cvimodel.as_deref() {
        #[cfg(feature = "cvi-runtime")]
        {
            let phase_started = Instant::now();
            let input = preprocess::cube_roi_resize(&img, model::INPUT_SIZE)?;
            let preprocess_time = phase_started.elapsed();
            let phase_started = Instant::now();
            let detector = tpu::CviTpuDetector::load(cvimodel)?;
            let model_load_time = phase_started.elapsed();
            let phase_started = Instant::now();
            let detections = detector.detect(&input)?;
            let inference_time = phase_started.elapsed();
            (
                input,
                detections,
                6,
                true,
                preprocess_time,
                model_load_time,
                inference_time,
            )
        }
        #[cfg(not(feature = "cvi-runtime"))]
        {
            let _ = cvimodel;
            bail!("--cvimodel requires a cross-build with --features cvi-runtime")
        }
    } else {
        let phase_started = Instant::now();
        let input = preprocess::letterbox(&img, model::INPUT_SIZE);
        let preprocess_time = phase_started.elapsed();
        let phase_started = Instant::now();
        let detector = model::YoloV8Detector::load(cli.model.as_deref().unwrap(), cli.classes)?;
        let model_load_time = phase_started.elapsed();
        let phase_started = Instant::now();
        let detections = detector.detect(&input)?;
        let inference_time = phase_started.elapsed();
        (
            input,
            detections,
            cli.classes,
            false,
            preprocess_time,
            model_load_time,
            inference_time,
        )
    };

    // 4. Postprocessing
    let phase_started = Instant::now();
    let detections = postprocess::filter_confidence(detections, cli.conf);
    let detections = if use_center_window {
        postprocess::filter_center_window(
            detections,
            model::INPUT_SIZE as f32,
            0.05,
            0.90,
            0.05,
            0.95,
        )
    } else {
        detections
    };
    let detections = postprocess::nms(detections, cli.iou);
    // Only map coordinates after model-space filters have done their job.
    let detections: Vec<_> = detections.iter().map(|d| input.to_original(d)).collect();
    let postprocess_time = phase_started.elapsed();
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

    if cli.timings {
        eprintln!(
            "timings_ms decode={:.2} preprocess={:.2} model_load={:.2} inference={:.2} postprocess={:.2} total={:.2}",
            decode_time.as_secs_f64() * 1e3,
            preprocess_time.as_secs_f64() * 1e3,
            model_load_time.as_secs_f64() * 1e3,
            inference_time.as_secs_f64() * 1e3,
            postprocess_time.as_secs_f64() * 1e3,
            total_started.elapsed().as_secs_f64() * 1e3,
        );
    }

    Ok(())
}
