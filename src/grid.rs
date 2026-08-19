//! Building the 3x3 grid from 9 detections.
//!
//! Algorithm: sort boxes by center Y, split into 3 triples (rows), sort
//! each triple by X. Works for near-frontal shots — our case with the
//! stand. With a heavily tilted camera, row clustering would be needed
//! (see docs/PROJECT_NOTES.md, "Pitfalls").

use crate::cube::Face;
use crate::{model::CLASS_COLORS, postprocess::Detection};
use anyhow::{bail, Context, Result};
use std::fmt;

/// Recognized face: 3x3 color symbols ('W', 'Y', 'R', 'O', 'G', 'B').
pub struct FaceGrid {
    pub cells: [[char; 3]; 3],
}

impl FaceGrid {
    /// Flat 9-char string, row by row: "WWYORRGBO".
    /// Will be useful for assembling the full cube state (URFDLB notation).
    pub fn to_compact_string(&self) -> String {
        self.cells.iter().flatten().collect()
    }

    /// Converts vision output into the domain type used by scan/solve code.
    pub fn to_face(&self) -> Result<Face> {
        Face::from_symbols(&self.to_compact_string())
    }
}

impl fmt::Display for FaceGrid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for row in &self.cells {
            writeln!(f, "{} {} {}", row[0], row[1], row[2])?;
        }
        Ok(())
    }
}

/// Builds the grid from exactly 9 detections. Any other count is an error:
/// silently producing garbage is not allowed (docs/PROJECT_NOTES.md, "Pitfalls").
pub fn build_grid(detections: &[Detection]) -> Result<FaceGrid> {
    if detections.len() != 9 {
        bail!(
            "expected exactly 9 detections, got {} — retake the shot \
             or tune --conf",
            detections.len()
        );
    }

    let mut by_y = detections.to_vec();
    by_y.sort_by(|a, b| a.y.total_cmp(&b.y));

    let mut cells = [['?'; 3]; 3];
    for (row_idx, row) in by_y.chunks(3).enumerate() {
        let mut row = row.to_vec();
        row.sort_by(|a, b| a.x.total_cmp(&b.x));
        for (col_idx, det) in row.iter().enumerate() {
            cells[row_idx][col_idx] = *CLASS_COLORS
                .get(det.class_id)
                .with_context(|| format!("unknown class_id {}", det.class_id))?;
        }
    }

    Ok(FaceGrid { cells })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn det(x: f32, y: f32, class_id: usize) -> Detection {
        Detection {
            x,
            y,
            w: 50.0,
            h: 50.0,
            class_id,
            confidence: 0.9,
        }
    }

    /// Nine detections in a 3x3 lattice but in a shuffled order.
    /// Layout: W W Y / O R R / G B O (classes: W=0, Y=1, R=2, O=3, G=4, B=5).
    fn nine_shuffled() -> Vec<Detection> {
        let class_at = |row: usize, col: usize| [0, 0, 1, 3, 2, 2, 4, 5, 3][row * 3 + col];
        let mut dets = Vec::new();
        // Iterate out of order with coordinate jitter — the grid must
        // restore the correct order.
        for &i in &[5usize, 0, 8, 2, 4, 6, 1, 7, 3] {
            let (row, col) = (i / 3, i % 3);
            let jx = ((i * 7) % 5) as f32 - 2.0;
            let jy = ((i * 5) % 7) as f32 - 3.0;
            dets.push(det(
                (col + 1) as f32 * 80.0 + jx,
                (row + 1) as f32 * 80.0 + jy,
                class_at(row, col),
            ));
        }
        dets
    }

    #[test]
    fn builds_correct_grid_from_shuffled_detections() {
        let face = build_grid(&nine_shuffled()).unwrap();
        assert_eq!(face.cells[0], ['W', 'W', 'Y']);
        assert_eq!(face.cells[1], ['O', 'R', 'R']);
        assert_eq!(face.cells[2], ['G', 'B', 'O']);
        assert_eq!(face.to_compact_string(), "WWYORRGBO");
        assert_eq!(face.to_face().unwrap().compact(), "WWYORRGBO");
    }

    #[test]
    fn rejects_wrong_detection_count() {
        let dets = nine_shuffled()[..8].to_vec();
        assert!(build_grid(&dets).is_err());
    }
}
