//! Sticker detector (YOLO).
//!
//! Two implementations of the Detector trait:
//!  - StubDetector: 9 deterministic boxes, to exercise the pipeline without
//!    trained weights.
//!  - YoloV8Detector: real YOLOv8n inference on candle. Currently validated
//!    with COCO-pretrained weights (80 classes); once the cube dataset is
//!    trained, the same code loads the 6-class color model (the architecture
//!    and weight layout are identical, only num_classes differs).

use crate::postprocess::Detection;
use crate::preprocess::Letterbox;
use crate::yolo_v8::{Multiples, YoloV8};
use anyhow::{Context, Result};
use candle_core::{DType, Device, Tensor};
use candle_nn::{Module, VarBuilder};
use std::path::Path;

/// Side of the square model input.
pub const INPUT_SIZE: usize = 320;

/// Color symbols by class_id. The order MUST match the data.yaml the model
/// was trained with.
pub const CLASS_COLORS: [char; 6] = ['W', 'Y', 'R', 'O', 'G', 'B'];

/// Common detector interface: any implementation (stub, candle-YOLO, TPU
/// variant) returns boxes in model-input coordinates.
pub trait Detector {
    fn detect(&self, input: &Letterbox) -> Result<Vec<Detection>>;
}

/// Display name of a class id: cube colors for the 6-class model, COCO
/// names for the 80-class pretrained model (dev smoke-test stage).
pub fn class_name(class_id: usize, num_classes: usize) -> String {
    let name = match num_classes {
        6 => CLASS_COLORS
            .get(class_id)
            .map(|c| c.to_string())
            .unwrap_or_default(),
        80 => COCO_NAMES
            .get(class_id)
            .map(|s| s.to_string())
            .unwrap_or_default(),
        _ => String::new(),
    };
    if name.is_empty() {
        format!("class#{class_id}")
    } else {
        name
    }
}

/// Real YOLOv8n detector on candle (pure Rust, cross-compiles to riscv64).
pub struct YoloV8Detector {
    model: YoloV8,
    device: Device,
}

impl YoloV8Detector {
    /// Loads weights from a .safetensors file. `num_classes` must match the
    /// checkpoint (80 for COCO-pretrained, 6 for the cube-color model).
    pub fn load(weights: &Path, num_classes: usize) -> Result<Self> {
        let device = Device::Cpu;
        // SAFETY: the weights file is opened read-only and never modified
        // while mapped.
        let vb = unsafe {
            VarBuilder::from_mmaped_safetensors(&[weights], DType::F32, &device)
                .with_context(|| format!("failed to load weights: {}", weights.display()))?
        };
        let model = YoloV8::load(vb, Multiples::n(), num_classes)
            .context("failed to build YOLOv8n from the given weights")?;
        Ok(Self { model, device })
    }
}

impl Detector for YoloV8Detector {
    fn detect(&self, input: &Letterbox) -> Result<Vec<Detection>> {
        let s = input.size;
        let xs = Tensor::from_vec(input.data.clone(), (1, 3, s, s), &self.device)?;

        // [1, 4 + num_classes, anchors]. "Anchors" are the candidate grid
        // cells the model predicts for: 40x40 + 20x20 + 10x10 = 2100 of
        // them at a 320 input. Boxes are already decoded into input-image
        // pixels, class scores are sigmoided probabilities.
        let pred = self.model.forward(&xs)?;
        let rows = pred.get(0)?.t()?.to_vec2::<f32>()?;

        let mut out = Vec::new();
        for row in &rows {
            let (bbox, scores) = row.split_at(4);
            let (class_id, &conf) = scores
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .expect("model must have at least one class");
            out.push(Detection {
                x: bbox[0],
                y: bbox[1],
                w: bbox[2],
                h: bbox[3],
                class_id,
                confidence: conf,
            });
        }
        Ok(out)
    }
}

/// Detector stub: deterministically generates 9 boxes in a 3x3 lattice with
/// a small "jitter" so grid sorting is exercised honestly.
pub struct StubDetector {
    pub conf: f32,
}

impl Default for StubDetector {
    fn default() -> Self {
        Self { conf: 0.9 }
    }
}

impl Detector for StubDetector {
    fn detect(&self, input: &Letterbox) -> Result<Vec<Detection>> {
        let s = input.size as f32;
        let step = s / 4.0; // lattice centers: 80, 160, 240 for a 320 input
        let box_side = step * 0.7;

        // Demo layout: W W Y / O R R / G B O
        let pattern = ['W', 'W', 'Y', 'O', 'R', 'R', 'G', 'B', 'O'];

        let mut out = Vec::with_capacity(9);
        for (i, &color) in pattern.iter().enumerate() {
            let (row, col) = (i / 3, i % 3);
            let class_id = CLASS_COLORS
                .iter()
                .position(|&c| c == color)
                .expect("pattern color must exist in CLASS_COLORS");

            // Jitter is +/-3 px — far below the lattice step (80 px), so it
            // cannot break row/column order but still exercises the sorting.
            let jx = ((i * 7) % 5) as f32 - 2.0;
            let jy = ((i * 5) % 7) as f32 - 3.0;

            out.push(Detection {
                x: (col as f32 + 1.0) * step + jx,
                y: (row as f32 + 1.0) * step + jy,
                w: box_side,
                h: box_side,
                class_id,
                confidence: self.conf,
            });
        }
        Ok(out)
    }
}

/// COCO class names (80), in the index order of the pretrained checkpoint.
/// COCO = Common Objects in Context — the standard 80-class object
/// detection dataset the public weights are trained on. Only needed while
/// we validate the pipeline; the cube model will use CLASS_COLORS instead.
pub const COCO_NAMES: [&str; 80] = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Loads the real COCO weights and runs one forward pass on a blank
    /// input. Validates weight loading and output decoding end to end.
    /// Ignored by default: requires models/yolov8n.safetensors and takes a
    /// few seconds in debug builds. Run with: cargo test -- --ignored
    #[test]
    #[ignore = "requires models/yolov8n.safetensors"]
    fn yolov8n_loads_and_runs_forward() {
        let img = image::RgbImage::new(640, 480);
        let input = Letterbox {
            data: vec![0.0; 3 * INPUT_SIZE * INPUT_SIZE],
            size: INPUT_SIZE,
            scale: 1.0,
            pad_x: 0.0,
            pad_y: 0.0,
        };
        let _ = img; // the blank tensor above stands in for a real frame here
        let detector =
            YoloV8Detector::load(Path::new("models/yolov8n.safetensors"), COCO_NAMES.len())
                .expect("weights must load");
        let dets = detector.detect(&input).expect("forward pass must succeed");
        // 320/8=40, 320/16=20, 320/16=10 -> 40*40 + 20*20 + 10*10 = 2100 anchors
        assert_eq!(dets.len(), 2100);
        assert!(dets.iter().all(|d| (0.0..=1.0).contains(&d.confidence)));
    }
}
