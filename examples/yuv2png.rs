//! Converts a raw YUV420 semi-planar frame (NV12/NV21) dumped on the
//! Milk-V Duo by `sample_sensor_test` (mode 2) into a PNG.
//!
//! The dump layout (see cvi_mpi/sample/sensor_test/sample_sensor_test.c):
//!   - plane 0: Y,  width*height bytes
//!   - plane 1: chroma interleaved, width*height/2 bytes
//!     (NV21: V,U,V,U,... — NV12: U,V,U,V,...; one chroma pair per 2x2
//!     luma block)
//!
//! For 1920x1080 the file must be exactly 1920*1080*1.5 = 3_110_400 bytes
//! (stride equals width at this resolution).
//!
//!   cargo run --release --example yuv2png -- \
//!       --input sample_0.yuv --output frame.png
//!   # if colors look swapped (red <-> blue), retry with --format nv12

use image::{ImageBuffer, Rgb};
use std::{env, fs, process::exit};

struct Args {
    input: String,
    output: String,
    format: String,
    width: usize,
    height: usize,
}

fn parse_args() -> Args {
    let mut a = Args {
        input: String::new(),
        output: String::from("frame.png"),
        format: String::from("nv21"),
        width: 1920,
        height: 1080,
    };
    let mut it = env::args().skip(1);
    while let Some(k) = it.next() {
        match k.as_str() {
            "--input" => a.input = it.next().unwrap_or_default(),
            "--output" => a.output = it.next().unwrap_or(a.output),
            "--format" => a.format = it.next().unwrap_or(a.format),
            "--size" => {
                let s = it.next().unwrap_or_default();
                let (w, h) = s.split_once('x').expect("--size must be WxH");
                a.width = w.parse().expect("bad width");
                a.height = h.parse().expect("bad height");
            }
            other => {
                eprintln!("unknown arg: {other}");
                exit(2);
            }
        }
    }
    if a.input.is_empty() {
        eprintln!("usage: yuv2png --input <file.yuv> [--output frame.png] [--format nv21|nv12] [--size 1920x1080]");
        exit(2);
    }
    a
}

/// BT.601 limited-range YUV -> RGB, the standard conversion for camera YUV.
fn yuv_to_rgb(y: f32, u: f32, v: f32) -> [u8; 3] {
    let clamp = |x: f32| x.clamp(0.0, 255.0) as u8;
    [
        clamp(y + 1.402 * (v - 128.0)),
        clamp(y - 0.344136 * (u - 128.0) - 0.714136 * (v - 128.0)),
        clamp(y + 1.772 * (u - 128.0)),
    ]
}

fn main() {
    let a = parse_args();
    let (w, h) = (a.width, a.height);
    let nv21 = match a.format.as_str() {
        "nv21" => true,
        "nv12" => false,
        _ => {
            eprintln!("--format must be nv21 or nv12");
            exit(2);
        }
    };

    let raw = fs::read(&a.input).expect("cannot read input file");
    let expected = w * h * 3 / 2;
    if raw.len() != expected {
        eprintln!(
            "bad file size: got {}, expected {} ({}x{} YUV420). \
             If the size differs, the VI stride is not equal to the width — \
             tell me the actual size and I will add stride handling.",
            raw.len(),
            expected,
            w,
            h
        );
        exit(1);
    }

    let (y_plane, uv_plane) = raw.split_at(w * h);
    let mut img = ImageBuffer::<Rgb<u8>, Vec<u8>>::new(w as u32, h as u32);

    for py in 0..h {
        for px in 0..w {
            let y = y_plane[py * w + px] as f32;
            // One interleaved chroma pair per 2x2 luma block.
            let c0 = uv_plane[(py / 2) * w + (px / 2) * 2] as f32;
            let c1 = uv_plane[(py / 2) * w + (px / 2) * 2 + 1] as f32;
            let (u, v) = if nv21 { (c1, c0) } else { (c0, c1) };
            img.put_pixel(px as u32, py as u32, Rgb(yuv_to_rgb(y, u, v)));
        }
    }

    img.save(&a.output).expect("cannot save PNG");
    println!("saved: {} ({}x{}, {})", a.output, w, h, a.format);
}
