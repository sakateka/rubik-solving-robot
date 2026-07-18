//! Generates a test "face photo": 9 colored squares in a 3x3 arrangement
//! on a dark background. Handy for end-to-end pipeline checks without a
//! real camera.
//!
//!   cargo run --example make_test_image

use image::{Rgb, RgbImage};

fn main() {
    // Layout matches the detector stub: W W Y / O R R / G B O
    let colors: [[u8; 3]; 9] = [
        [255, 255, 255], // W
        [255, 255, 255], // W
        [255, 255, 0],   // Y
        [255, 140, 0],   // O
        [220, 20, 20],   // R
        [220, 20, 20],   // R
        [0, 160, 0],     // G
        [30, 30, 220],   // B
        [255, 140, 0],   // O
    ];

    let mut img = RgbImage::from_pixel(640, 480, Rgb([40, 40, 40]));

    for (i, color) in colors.iter().enumerate() {
        let (row, col) = (i / 3, i % 3);
        let x0 = 120 + col * 140;
        let y0 = 40 + row * 140;
        for y in y0..y0 + 100 {
            for x in x0..x0 + 100 {
                img.put_pixel(x as u32, y as u32, Rgb(*color));
            }
        }
    }

    img.save("test_face.png").expect("failed to save PNG");
    println!("saved: test_face.png");
}
