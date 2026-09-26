//! Benchmark comparing `poormans-camera-rs` (Rust) vs Unity C# / Cinemachine Poor Man's Camera implementation.

use glam::{Quat, Vec3};
use poormans_camera_rs::{
    CameraRig, CameraRigConfig, Collider, FramingTarget,
    OcclusionSolver, QuaternionSpring, SpringConfig, Vec3Spring,
};
use std::time::Instant;

fn main() {
    println!("============================================================");
    println!("  poormans-camera-rs (Rust) vs Unity C# Poorman's Camera   ");
    println!("============================================================");

    // 1. Full Camera Rig Update (Springs + Dilation + View/Projection)
    println!("\n--- 1. Full Third-Person Camera Rig Frame Update ---");
    {
        let mut rig = CameraRig::new(Vec3::new(0.0, 1.0, 0.0), CameraRigConfig::default());
        let iterations = 1_000_000;
        let dt = 1.0 / 60.0;
        let start = Instant::now();
        let mut dummy_det = 0.0f32;

        for i in 0..iterations {
            let t = (i as f32) * 0.01;
            let target_pos = Vec3::new(t.sin() * 10.0, 1.0, t.cos() * 10.0);
            let target_vel = Vec3::new(t.cos() * 10.0, 0.0, -t.sin() * 10.0);
            let target = FramingTarget::single(target_pos, target_vel);

            let (view_proj, _) = rig.update(&target, dt);
            dummy_det += view_proj.view.x_axis.x + view_proj.projection.x_axis.x;
        }

        std::hint::black_box(dummy_det);
        let elapsed = start.elapsed();
        let ns_per_step = elapsed.as_nanos() as f64 / iterations as f64;
        let steps_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Rig Updates: {} | Total: {:.2?} | Latency: {:.2} ns/step | {:>10.0} steps/s",
            iterations, elapsed, ns_per_step, steps_per_sec
        );
    }

    // 2. Analytical 2nd-Order Spring Dynamics (Vec3 + Quat)
    println!("\n--- 2. Analytical 2nd-Order Spring Dynamics (Harmonic Oscillator) ---");
    {
        let config = SpringConfig::new(15.0, 1.0);
        let mut v_spring = Vec3Spring::new(Vec3::ZERO, config);
        let mut q_spring = QuaternionSpring::new(Quat::IDENTITY, config);

        let iterations = 5_000_000;
        let dt = 0.016;
        let start = Instant::now();
        let mut dummy = 0.0f32;

        for i in 0..iterations {
            let target_p = Vec3::new((i % 100) as f32 * 0.1, 2.0, (i % 50) as f32 * 0.2);
            let target_q = Quat::from_rotation_y((i % 360) as f32 * 0.01745);

            v_spring.update(target_p, dt);
            q_spring.update(target_q, dt);

            dummy += v_spring.position.x + q_spring.current.w;
        }

        std::hint::black_box(dummy);
        let elapsed = start.elapsed();
        let ns_per_eval = elapsed.as_nanos() as f64 / iterations as f64;
        let evals_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Spring Steps: {} | Total: {:.2?} | Latency: {:.2} ns/step | {:>10.0} steps/s",
            iterations, elapsed, ns_per_eval, evals_per_sec
        );
    }

    // 3. Occlusion Sphere-Cast Solver with Hysteresis
    println!("\n--- 3. Continuous Sphere-Cast Occlusion Solver & Hysteresis ---");
    {
        let mut solver = OcclusionSolver::new(0.3, 6.0, 0.5, 4.0);
        // Populate scene with obstacles
        for i in 0..10 {
            let z = 1.0 + (i as f32) * 1.5;
            solver.environment.add_box(
                Vec3::new(-2.0, -1.0, z),
                Vec3::new(2.0, 3.0, z + 0.5),
            );
            solver.environment.add_sphere(Vec3::new(3.0, 1.0, z), 0.8);
        }

        let iterations = 500_000;
        let dt = 0.016;
        let start = Instant::now();
        let mut occluded_count = 0;

        for i in 0..iterations {
            let angle = (i % 360) as f32 * 0.01745;
            let target = Vec3::new(angle.cos() * 2.0, 1.0, angle.sin() * 2.0);
            let ideal_cam = target + Vec3::new(0.0, 1.5, 5.0);

            let res = solver.solve(target, ideal_cam, dt);
            if res.is_occluded {
                occluded_count += 1;
            }
        }

        std::hint::black_box(occluded_count);
        let elapsed = start.elapsed();
        let ns_per_cast = elapsed.as_nanos() as f64 / iterations as f64;
        let casts_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Occlusion Casts: {} (20 colliders) | Total: {:.2?} | Latency: {:.2} ns/cast | {:>10.0} casts/s | Hits: {}",
            iterations, elapsed, ns_per_cast, casts_per_sec, occluded_count
        );
    }

    // 4. Primitive Sphere-AABB Continuous Sweep
    println!("\n--- 4. Continuous Sphere-AABB Geometry Sweep ---");
    {
        let box_col = Collider::Aabb {
            min: Vec3::new(-2.0, -2.0, 4.0),
            max: Vec3::new(2.0, 2.0, 6.0),
        };

        let iterations = 10_000_000;
        let start = Instant::now();
        let mut hits = 0;

        for i in 0..iterations {
            let offset_x = (i % 100) as f32 * 0.05 - 2.5;
            let origin = Vec3::new(offset_x, 0.0, 0.0);
            let dir = Vec3::Z;
            if let Some(hit) = box_col.sweep_sphere(origin, dir, 10.0, 0.25) {
                if hit.fraction < 1.0 {
                    hits += 1;
                }
            }
        }

        std::hint::black_box(hits);
        let elapsed = start.elapsed();
        let ns_per_sweep = elapsed.as_nanos() as f64 / iterations as f64;
        let sweeps_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Sphere-AABB Sweeps: {} | Total: {:.2?} | Latency: {:.2} ns/sweep | {:>10.0} sweeps/s | Hits: {}",
            iterations, elapsed, ns_per_sweep, sweeps_per_sec, hits
        );
    }

    println!("\n============================================================");
    println!("                      Benchmark Complete                    ");
    println!("============================================================");
}
