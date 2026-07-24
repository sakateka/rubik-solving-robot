//! Detection postprocessing: confidence filtering and NMS.
//!
//! YOLO produces thousands of candidates with duplicate boxes around the
//! same object (several neighboring anchors "fire" at once).
//! NMS = Non-Maximum Suppression: out of each group of overlapping boxes
//! it keeps the MAXIMUM-confidence one and SUPPRESSES (deletes) all the
//! NON-MAXIMUM ones — the duplicates.

/// A single detection: sticker box (center + size) and class/color.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    /// Box center, x
    pub x: f32,
    /// Box center, y
    pub y: f32,
    /// Box width
    pub w: f32,
    /// Box height
    pub h: f32,
    /// Class index (0..6, see model::CLASS_COLORS)
    pub class_id: usize,
    /// Model confidence, 0..1
    pub confidence: f32,
}

impl Detection {
    fn left(&self) -> f32 {
        self.x - self.w / 2.0
    }
    fn right(&self) -> f32 {
        self.x + self.w / 2.0
    }
    fn top(&self) -> f32 {
        self.y - self.h / 2.0
    }
    fn bottom(&self) -> f32 {
        self.y + self.h / 2.0
    }
    fn area(&self) -> f32 {
        self.w * self.h
    }
}

/// IoU = Intersection over Union ("area of overlap / area of union")
/// of two boxes: 0 — disjoint, 1 — identical. This is THE measure of
/// "are these two boxes about the same object".
pub fn iou(a: &Detection, b: &Detection) -> f32 {
    let inter_w = (a.right().min(b.right()) - a.left().max(b.left())).max(0.0);
    let inter_h = (a.bottom().min(b.bottom()) - a.top().max(b.top())).max(0.0);
    let inter = inter_w * inter_h;
    let union = a.area() + b.area() - inter;
    if union <= 0.0 {
        0.0
    } else {
        inter / union
    }
}

/// Drops detections below the confidence threshold.
pub fn filter_confidence(dets: Vec<Detection>, threshold: f32) -> Vec<Detection> {
    dets.into_iter()
        .filter(|d| d.confidence >= threshold)
        .collect()
}

/// Keep only boxes whose centers lie in the expected central part of the
/// model image. The physical stand is nearly static; this deliberately rejects
/// distant rig/background objects before the "exactly 9" safety check.
pub fn filter_center_window(
    dets: Vec<Detection>,
    input_size: f32,
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
) -> Vec<Detection> {
    dets.into_iter()
        .filter(|d| {
            let x = d.x / input_size;
            let y = d.y / input_size;
            (min_x..=max_x).contains(&x) && (min_y..=max_y).contains(&y)
        })
        .collect()
}

/// NMS, class-agnostic variant: duplicates are suppressed regardless of
/// class.
///
/// For our task this beats the classic per-class NMS: the model can catch
/// the same sticker twice with different colors, and such a duplicate must
/// be suppressed too, otherwise the grid gets 10 detections.
pub fn nms(mut dets: Vec<Detection>, iou_threshold: f32) -> Vec<Detection> {
    // Greedy: take the most confident box, suppress everything overlapping
    // it heavily, repeat for the rest.
    dets.sort_by(|a, b| b.confidence.total_cmp(&a.confidence));

    let mut keep: Vec<Detection> = Vec::with_capacity(dets.len());
    'candidates: for det in dets {
        for kept in &keep {
            if iou(&det, kept) > iou_threshold {
                continue 'candidates;
            }
        }
        keep.push(det);
    }
    keep
}

#[cfg(test)]
mod tests {
    use super::*;

    fn det(x: f32, y: f32, class_id: usize, confidence: f32) -> Detection {
        Detection {
            x,
            y,
            w: 50.0,
            h: 50.0,
            class_id,
            confidence,
        }
    }

    #[test]
    fn iou_identical_boxes_is_one() {
        let a = det(100.0, 100.0, 0, 0.9);
        assert_eq!(iou(&a, &a), 1.0);
    }

    #[test]
    fn iou_disjoint_boxes_is_zero() {
        let a = det(0.0, 0.0, 0, 0.9);
        let b = det(500.0, 500.0, 0, 0.9);
        assert_eq!(iou(&a, &b), 0.0);
    }

    #[test]
    fn nms_suppresses_duplicate_and_keeps_most_confident() {
        // A duplicate of one sticker: nearly the same box, lower confidence.
        let best = det(100.0, 100.0, 2, 0.95);
        let dup = det(102.0, 101.0, 2, 0.80);
        let kept = nms(vec![dup, best], 0.45);
        assert_eq!(kept, vec![best]);
    }

    #[test]
    fn nms_is_class_agnostic() {
        // A duplicate with a DIFFERENT class is suppressed too
        // (deliberate choice, see the doc comment above).
        let best = det(100.0, 100.0, 2, 0.95);
        let dup_other_class = det(101.0, 100.0, 4, 0.70);
        let kept = nms(vec![dup_other_class, best], 0.45);
        assert_eq!(kept, vec![best]);
    }

    #[test]
    fn nms_keeps_distant_boxes() {
        let a = det(100.0, 100.0, 0, 0.9);
        let b = det(300.0, 100.0, 1, 0.8);
        assert_eq!(nms(vec![a, b], 0.45).len(), 2);
    }

    #[test]
    fn filter_confidence_works() {
        let dets = vec![det(0.0, 0.0, 0, 0.4), det(0.0, 0.0, 0, 0.6)];
        let kept = filter_confidence(dets, 0.5);
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].confidence, 0.6);
    }

    #[test]
    fn center_window_removes_distant_background_box() {
        let center = det(160.0, 160.0, 0, 0.9);
        let background = det(310.0, 160.0, 3, 0.9);
        let kept = filter_center_window(vec![center, background], 320.0, 0.05, 0.90, 0.05, 0.95);
        assert_eq!(kept, vec![center]);
    }
}
