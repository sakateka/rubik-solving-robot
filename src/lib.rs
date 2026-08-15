//! Platform-independent Rubik's cube domain logic.
//!
//! Binaries for camera scanning, servo control, and diagnostics depend on this
//! small crate; it deliberately has no CVI, Bevy, or hardware dependencies.

#[cfg(feature = "cvi-camera")]
pub mod camera;
pub mod cube;
#[cfg(feature = "pca9685")]
pub mod pca9685;
pub mod stand;
