# poormans-camera-rs

[![GitHub Pages](https://img.shields.io/badge/Demo-Live%203D%20Visualizer-38bdf8?style=flat-square&logo=github)](https://code.brandonhubbard.com/poormans-camera-rs/)
[![Rust](https://img.shields.io/badge/Rust-Edition%202024-orange?style=flat-square&logo=rust)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/License-MIT%20%2F%20Apache--2.0-blue?style=flat-square)](LICENSE)

Pure Rust 3D third-person camera rig featuring **Sphere-Cast Collision Avoidance** and **2nd-Order Spring Damping**, based on Evan Todd's iconic *The Poor Man's 3D Camera*.

👉 **[Launch Interactive 3D Camera Visualizer](https://code.brandonhubbard.com/poormans-camera-rs/)**

---

## ⚡ Mathematical Foundations

### 1. Mechanical 2nd-Order Spring Damping (`spring`)

Cameras using simple linear interpolation (`lerp`) feel floaty and unresponsive. `poormans-camera-rs` implements closed-form analytical solutions to the 2nd-order damped harmonic oscillator differential equation:

$$\ddot{\mathbf{x}} + 2\zeta\omega\dot{\mathbf{x}} + \omega^2(\mathbf{x} - \mathbf{x}_{\text{target}}) = 0$$

- **Natural Angular Frequency $\omega$ (rad/s)**: Controls responsiveness and tightness of tracking.
- **Damping Ratio $\zeta$**:
  - $\zeta = 1.0$ (**Critically Damped**): Fastest asymptotic convergence to target without any overshoot.
  - $\zeta < 1.0$ (**Underdamped**): Springy oscillation with game feel bounce.
  - $\zeta > 1.0$ (**Overdamped**): Sluggish exponential decay without oscillation.
- **Unconditionally Stable**: Computed via exact closed-form analytical formulas—impervious to variable framerates ($\Delta t$) or integration instability.
- **Angular Smoothing**: Shortest-arc quaternion interpolation (`QuaternionSpring`) and Euler angle spring (`AngularSpring`) with continuous unwrapping across the $[-\pi, +\pi]$ boundary.

### 2. Sphere-Cast Occlusion Solver (`occlusion`)

Raycasts alone fail because the camera near-clip plane clips through walls when approaching corners. `poormans-camera-rs` sweeps a bounding sphere with radius $R_{\text{near}} \ge \text{near\_clip} \cdot \sqrt{2}$:

- **Minkowski Swept Primitives**: Supports `Sphere`, `AABB`, `Plane`, and `Triangle` collision geometry.
- **Predictive Pull-In**: When an obstacle intersects the swept sphere ray between focal point and ideal camera, the camera is pulled forward along the ray to the nearest surface contact point.
- **Asymmetric Hysteresis Filter**:
  - **Instant / High-Speed Pull-In**: Prevents camera clipping through geometry even during sudden fast maneuvers.
  - **Smooth Push-Out Recovery**: Slowly returns to the ideal distance at a controlled rate (`recovery_speed`), eliminating camera pop and jitter when skimming past rough walls or pillars.

### 3. Camera Rig & Framing (`rig`)

- Third-person orbit with pitch/yaw clamping (preventing gimbal lock).
- Configurable shoulder offsets (over-the-shoulder shooter perspective).
- **Speed-Based Dynamic FOV Dilation**: Widens vertical field-of-view smoothly as the character accelerates.
- **Vehicle Chase Auto-Alignment**: Automatically rotates camera yaw behind velocity direction.
- **Dual-Target Combat Framing**: Smoothly frames between player character and locked-on enemy targets.

---

## 📦 Installation

Add `poormans-camera-rs` to your `Cargo.toml`:

```toml
[dependencies]
poormans-camera-rs = "0.1.0"
glam = "0.29"
```

---

## 🚀 Quick Start

### Basic Third-Person Orbit Camera

```rust
use glam::Vec3;
use poormans_camera_rs::{CameraRig, CameraRigConfig, FramingTarget, SpringConfig};

// Initialize rig focused at origin
let mut config = CameraRigConfig::default();
config.ideal_distance = 6.0;
config.position_spring = SpringConfig::cinematic(); // Natural smooth follow

let mut rig = CameraRig::new(Vec3::new(0.0, 1.0, 0.0), config);

// Add collision wall box
rig.occlusion.environment.add_box(
    Vec3::new(-2.0, 0.0, 2.0),
    Vec3::new(2.0, 4.0, 2.5),
);

// In your game loop (e.g. 60 FPS):
let target = FramingTarget::single(
    Vec3::new(0.0, 1.5, 0.0), // Player character position
    Vec3::new(0.0, 0.0, 5.0), // Character moving forward at 5 m/s
);

let dt = 1.0 / 60.0;
let (view_proj, occlusion_result) = rig.update(&target, dt);

println!("Camera Eye: {:?}", view_proj.eye_position);
println!("Target Focal Point: {:?}", view_proj.target_position);
println!("Is Occluded: {}", occlusion_result.is_occluded);
```

### Dual-Target Combat Lock-On

```rust
use glam::Vec3;
use poormans_camera_rs::{CameraRig, CameraRigConfig, FramingTarget};

let mut rig = CameraRig::new(Vec3::ZERO, CameraRigConfig::default());

// Target frames both player and combat target at a weighted midpoint
let target = FramingTarget::combat_lock_on(
    Vec3::new(-2.0, 1.0, 0.0), // Player
    Vec3::ZERO,
    Vec3::new(4.0, 1.5, 8.0),  // Locked-on enemy
);

let (view_proj, _) = rig.update(&target, 0.016);
```

---

## 🧪 Running Tests

```bash
cargo test
```

All 11 unit & integration tests verify:
- Exact critically damped monotonic convergence with zero overshoot
- Underdamped sinusoidal oscillation and overshoot
- Asymmetric hysteresis filter recovery without jitter
- 3D swept sphere collision against spheres, boxes, and planes
- Angular spring seam unwrapping across $[-\pi, +\pi]$
- View and projection matrix generation
- Speed-based FOV dilation and dual-target framing

---

## 📄 License

Licensed under either of [Apache License, Version 2.0](LICENSE) or [MIT License](LICENSE) at your option.
