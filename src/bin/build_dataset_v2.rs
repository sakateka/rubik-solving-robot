//! Build the fixed-ROI YOLO dataset without Python/Pillow recompression.
//!
//! Labels must first be extracted from a Label Studio YOLO export; `unzip` only
//! extracts tiny text files, while this program does all image work in Rust.

use anyhow::{anyhow, bail, Context, Result};
use clap::Parser;
use image::{codecs::jpeg::JpegEncoder, ColorType, GenericImageView, ImageEncoder};
use std::{collections::BTreeMap, fs, fs::File, io::BufWriter, path::PathBuf};

const ROI: (u32, u32, u32, u32) = (464, 32, 1296, 864);
const SOURCE_SIZE: (u32, u32) = (1920, 1080);

#[derive(Parser, Debug)]
#[command(about = "Crop the static cube ROI and convert YOLO labels for dataset v2")]
struct Args {
    /// Directory containing source shot_*.yuv.png images.
    #[arg(long, default_value = "images")]
    images: PathBuf,
    /// Directory `labels/` extracted from the Label Studio YOLO ZIP.
    #[arg(long)]
    labels: PathBuf,
    /// New dataset directory. It must not already exist.
    #[arg(long, default_value = "dataset-v2")]
    output: PathBuf,
}

fn shot_number(name: &str) -> Result<u32> {
    let start = name.find("shot_").context("missing shot_ in filename")? + 5;
    let end = name[start..]
        .find('.')
        .context("missing extension after shot number")?
        + start;
    name[start..end].parse().context("invalid shot number")
}

fn split_for(shot: u32) -> &'static str {
    // Whole 10-shot groups stay together: similar consecutive shots cannot leak.
    match ((shot - 1) / 10) % 7 {
        1 => "val",
        5 => "test",
        _ => "train",
    }
}

fn convert_labels(text: &str) -> Result<String> {
    let (left, top, right, bottom) = ROI;
    let (source_w, source_h) = SOURCE_SIZE;
    let roi_w = (right - left) as f32;
    let roi_h = (bottom - top) as f32;
    let mut converted = String::new();

    for line in text.lines() {
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 5 {
            bail!("expected 5 YOLO fields: {line}");
        }
        let class = fields[0].parse::<u8>().context("invalid class")?;
        if class > 5 {
            bail!("class outside 0..5: {class}");
        }
        let values: Vec<f32> = fields[1..]
            .iter()
            .map(|v| v.parse())
            .collect::<std::result::Result<_, _>>()
            .context("invalid YOLO coordinate")?;
        let (cx, cy, width, height) = (values[0], values[1], values[2], values[3]);
        let x1 = (cx - width / 2.0) * source_w as f32;
        let y1 = (cy - height / 2.0) * source_h as f32;
        let x2 = (cx + width / 2.0) * source_w as f32;
        let y2 = (cy + height / 2.0) * source_h as f32;
        if x1 < left as f32 || y1 < top as f32 || x2 > right as f32 || y2 > bottom as f32 {
            bail!("label lies outside fixed ROI {ROI:?}: {line}");
        }
        let out_cx = ((x1 + x2) / 2.0 - left as f32) / roi_w;
        let out_cy = ((y1 + y2) / 2.0 - top as f32) / roi_h;
        let out_w = (x2 - x1) / roi_w;
        let out_h = (y2 - y1) / roi_h;
        converted.push_str(&format!(
            "{class} {out_cx:.8} {out_cy:.8} {out_w:.8} {out_h:.8}\n"
        ));
    }
    Ok(converted)
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.output.exists() {
        bail!("refusing to overwrite {}", args.output.display());
    }
    let mut images = BTreeMap::new();
    for entry in fs::read_dir(&args.images).context("read source images directory")? {
        let path = entry?.path();
        if path.extension().and_then(|e| e.to_str()) == Some("png") {
            images.insert(
                shot_number(&path.file_name().unwrap().to_string_lossy())?,
                path,
            );
        }
    }
    let mut labels = Vec::new();
    for entry in fs::read_dir(&args.labels).context("read labels directory")? {
        let path = entry?.path();
        if path.extension().and_then(|e| e.to_str()) == Some("txt") {
            labels.push(path);
        }
    }
    if images.len() != 305 || labels.len() != 305 {
        bail!(
            "expected 305 images and labels; got {} and {}",
            images.len(),
            labels.len()
        );
    }
    for split in ["train", "val", "test"] {
        fs::create_dir_all(args.output.join("images").join(split))?;
        fs::create_dir_all(args.output.join("labels").join(split))?;
    }
    let mut split_counts = BTreeMap::new();
    let mut jobs = Vec::with_capacity(labels.len());
    for label_path in labels {
        let shot = shot_number(&label_path.file_name().unwrap().to_string_lossy())?;
        let split = split_for(shot);
        let source = images
            .get(&shot)
            .context("label without matching source image")?
            .clone();
        jobs.push((label_path, shot, split, source));
        *split_counts.entry(split).or_insert(0usize) += 1;
    }
    let workers = std::thread::available_parallelism()
        .map_or(1, usize::from)
        .min(8);
    let chunk_size = jobs.len().div_ceil(workers);
    std::thread::scope(|scope| -> Result<()> {
        let handles: Vec<_> = jobs
            .chunks(chunk_size)
            .map(|chunk| {
                let output = &args.output;
                scope.spawn(move || -> Result<()> {
                    for (label_path, shot, split, source) in chunk {
                        let image = image::open(source)
                            .with_context(|| format!("open {}", source.display()))?;
                        if image.dimensions() != SOURCE_SIZE {
                            bail!(
                                "unexpected source size {:?}: {}",
                                image.dimensions(),
                                source.display()
                            );
                        }
                        let cropped = image
                            .crop_imm(ROI.0, ROI.1, ROI.2 - ROI.0, ROI.3 - ROI.1)
                            .to_rgb8();
                        let destination = output
                            .join("images")
                            .join(split)
                            .join(format!("shot_{shot:03}.jpg"));
                        let writer = BufWriter::new(File::create(destination)?);
                        JpegEncoder::new_with_quality(writer, 95).write_image(
                            cropped.as_raw(),
                            cropped.width(),
                            cropped.height(),
                            ColorType::Rgb8.into(),
                        )?;
                        let converted = convert_labels(&fs::read_to_string(label_path)?)?;
                        fs::write(
                            output
                                .join("labels")
                                .join(split)
                                .join(format!("shot_{shot:03}.txt")),
                            converted,
                        )?;
                    }
                    Ok(())
                })
            })
            .collect();
        for handle in handles {
            handle
                .join()
                .map_err(|_| anyhow!("crop worker panicked"))??;
        }
        Ok(())
    })?;
    let dataset_path = args.output.canonicalize()?;
    fs::write(
        args.output.join("data.yaml"),
        format!(
            "path: {}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: white\n  1: yellow\n  2: red\n  3: orange\n  4: green\n  5: blue\n",
            dataset_path.display()
        ),
    )?;
    println!(
        "ROI {ROI:?}, cropped size {}x{}",
        ROI.2 - ROI.0,
        ROI.3 - ROI.1
    );
    println!("split: {split_counts:?}");
    Ok(())
}
