//! # Poor Man's Camera RS
//!
//! Pure Rust 3D third-person camera rig with sphere-cast collision avoidance
//! and 2nd-order spring damping, based on Evan Todd's "The Poor Man's 3D Camera".
//!
//! ## Core Architecture
//! - **Sphere-Cast Occlusion Solver (`occlusion`)**: Sweeps a bounding sphere from target to ideal
//!   camera position against collision geometry (AABB, spheres, planes, triangles) to prevent near-clip plane clipping,
//!   with hysteresis to eliminate pop/jitter.
//! - **Mechanical Spring Damping (`spring`)**: Analytical 2nd-order damped harmonic oscillators
//!   supporting critical damping ($\zeta = 1$), underdamping ($\zeta < 1$), and overdamping ($\zeta > 1$),
//!   along with shortest-arc quaternion rotational smoothing.
//! - **Camera Rig & Framing (`rig`)**: Third-person orbit, shoulder offsets, pitch/yaw clamps,
//!   dynamic speed-based FOV dilation, vehicle chase alignment, and dual-target combat framing.

pub mod occlusion;
pub mod rig;
pub mod spring;

pub use occlusion::{
    Collider, CollisionMesh, HysteresisFilter, OcclusionHit, OcclusionSolver, Ray, SphereCastResult,
};
pub use rig::{CameraRig, CameraRigConfig, FramingTarget, ViewProjection};
pub use spring::{AngularSpring, QuaternionSpring, SpringConfig, SpringDampingType, Vec3Spring};

use thiserror::Error;

/// Error types for camera operations.
#[derive(Debug, Error, PartialEq)]
pub enum CameraError {
    #[error("Invalid camera parameter: {0}")]
    InvalidParameter(String),

    #[error("Zero direction vector cannot be normalized for camera orientation")]
    ZeroDirection,
}
