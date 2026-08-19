//! Platform-independent Rubik's cube domain logic.
//!
//! Binaries for camera scanning, servo control, and diagnostics depend on this
//! small crate; it deliberately has no CVI, Bevy, or hardware dependencies.

#[cfg(feature = "cvi-camera")]
pub mod camera;
pub mod cube;
pub mod grid;
pub mod model;
#[cfg(feature = "pca9685")]
pub mod pca9685;
pub mod postprocess;
pub mod preprocess;
pub mod robot_client;
pub mod robot_link;
#[cfg(feature = "pca9685")]
pub mod robot_service;
pub mod stand;
#[cfg(feature = "pca9685")]
pub mod stand_runtime;
#[cfg(feature = "cvi-runtime")]
pub mod tpu;
mod yolo_v8;
