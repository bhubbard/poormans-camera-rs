use glam::{Mat4, Quat, Vec3};
use poormans_camera_rs::{
    AngularSpring, CameraRig, CameraRigConfig, Collider, FramingTarget,
    OcclusionSolver, QuaternionSpring, SpringConfig, Vec3Spring,
};

#[test]
fn test_scalar_spring_critical_damping() {
    let config = SpringConfig::new(10.0, 1.0); // Critically damped
    let mut pos = 0.0f32;
    let mut vel = 0.0f32;
    let target = 10.0f32;
    let dt = 0.016f32;

    for _ in 0..100 {
        let (n_pos, n_vel) = poormans_camera_rs::spring::update_scalar_spring(pos, vel, target, &config, dt);
        pos = n_pos;
        vel = n_vel;
    }

    // After 1.6s of omega=10, should be practically at 10.0 without exceeding 10.0
    assert!((pos - target).abs() < 0.01);
    assert!(pos <= target + 1e-4, "Critically damped spring must not overshoot");
}

#[test]
fn test_scalar_spring_underdamping() {
    let config = SpringConfig::new(10.0, 0.4); // Underdamped -> should overshoot
    let mut pos = 0.0f32;
    let mut vel = 0.0f32;
    let target = 10.0f32;
    let dt = 0.016f32;
    let mut overshot = false;

    for _ in 0..150 {
        let (n_pos, n_vel) = poormans_camera_rs::spring::update_scalar_spring(pos, vel, target, &config, dt);
        pos = n_pos;
        vel = n_vel;
        if pos > target {
            overshot = true;
            break;
        }
    }

    assert!(overshot, "Underdamped spring must oscillate and overshoot target");
}

#[test]
fn test_vec3_spring_convergence() {
    let mut spring = Vec3Spring::new(Vec3::ZERO, SpringConfig::stiff());
    let target = Vec3::new(5.0, 2.0, -10.0);

    for _ in 0..60 {
        spring.update(target, 1.0 / 60.0);
    }

    assert!(spring.position.distance(target) < 0.05);
}

#[test]
fn test_angular_spring_wraparound() {
    let config = SpringConfig::new(10.0, 1.0);
    // Start at near +PI (3.10 rad) and target near -PI (-3.10 rad)
    // Shortest arc is only 0.08 rad across the seam!
    let mut spring = AngularSpring::new(3.10, 0.0, config);

    let (next_yaw, _) = spring.update(-3.10, 0.0, 0.016);
    // Next yaw should be moving towards/past PI wrap, not winding backwards through 0!
    assert!(next_yaw.abs() > 2.5, "Angular spring must take shortest geodesic arc across +/- PI");
}

#[test]
fn test_quaternion_spring_tracking() {
    let config = SpringConfig::new(15.0, 1.0);
    let mut q_spring = QuaternionSpring::new(Quat::IDENTITY, config);
    let target_rot = Quat::from_rotation_y(90.0f32.to_radians());

    for _ in 0..60 {
        q_spring.update(target_rot, 1.0 / 60.0);
    }

    let dot = q_spring.current.dot(target_rot).abs();
    assert!((dot - 1.0).abs() < 0.02, "Quaternion spring should converge to target rotation");
}

#[test]
fn test_sphere_sweep_against_sphere() {
    let obstacle = Collider::Sphere {
        center: Vec3::new(0.0, 0.0, 5.0),
        radius: 1.0,
    };

    // Sweep sphere of radius 0.5 from (0,0,0) along Z+ towards obstacle
    let hit = obstacle.sweep_sphere(Vec3::ZERO, Vec3::Z, 10.0, 0.5);
    assert!(hit.is_some());
    let hit = hit.unwrap();
    // Total radius = 1.0 + 0.5 = 1.5. Center is at 5.0, so hit occurs at distance = 5.0 - 1.5 = 3.5
    assert!((hit.distance - 3.5).abs() < 1e-3);
    assert!((hit.fraction - 0.35).abs() < 1e-3);
}

#[test]
fn test_sphere_sweep_against_aabb() {
    let obstacle = Collider::Aabb {
        min: Vec3::new(-2.0, -2.0, 4.0),
        max: Vec3::new(2.0, 2.0, 6.0),
    };

    // Sweep sphere radius 0.2 towards front face at Z = 4.0
    let hit = obstacle.sweep_sphere(Vec3::ZERO, Vec3::Z, 10.0, 0.2);
    assert!(hit.is_some());
    let hit = hit.unwrap();
    // Hit distance should be at 4.0 - 0.2 = 3.8
    assert!((hit.distance - 3.8).abs() < 0.05);
}

#[test]
fn test_occlusion_solver_pull_in_and_hysteresis() {
    let mut solver = OcclusionSolver::new(0.2, 5.0, 0.5, 3.0);
    // Add a wall box between focal point (0,0,0) and ideal camera (0,0,5)
    solver.environment.add_box(
        Vec3::new(-5.0, -5.0, 2.0),
        Vec3::new(5.0, 5.0, 2.5),
    );

    let target = Vec3::ZERO;
    let ideal_camera = Vec3::new(0.0, 0.0, 5.0);

    // Initial tick with obstruction
    let res = solver.solve(target, ideal_camera, 0.016);
    assert!(res.is_occluded);
    // Wall front face is at Z = 2.0. With near radius 0.2, camera is pulled to ~1.8
    assert!(res.current_distance < 2.0);
    assert!(res.resolved_position.z < 2.0);

    // Clear obstacles and verify recovery with hysteresis
    solver.environment.colliders.clear();
    let res_clear_1 = solver.solve(target, ideal_camera, 0.1);
    assert!(!res_clear_1.is_occluded);
    // Distance should increase smoothly, not pop instantly to 5.0
    assert!(res_clear_1.current_distance > 1.8);
    assert!(res_clear_1.current_distance < 5.0, "Recovery should use smooth hysteresis");
}

#[test]
fn test_camera_rig_view_projection() {
    let target = FramingTarget::single(Vec3::new(0.0, 1.0, 0.0), Vec3::ZERO);
    let mut rig = CameraRig::new(target.primary_position, CameraRigConfig::default());

    let (view_proj, _) = rig.update(&target, 0.016);

    // View matrix should transform target into camera space with negative Z
    let target_in_view = view_proj.view.transform_point3(target.focal_point());
    assert!(target_in_view.z < 0.0, "Target should be in front of camera (Z < 0)");

    // Projection matrix should have valid aspect ratio and FOV
    assert!(view_proj.projection != Mat4::IDENTITY);
}

#[test]
fn test_speed_based_fov_dilation() {
    let _resting_target = FramingTarget::single(Vec3::ZERO, Vec3::ZERO);
    let fast_target = FramingTarget::single(Vec3::ZERO, Vec3::new(0.0, 0.0, 25.0));

    let mut rig = CameraRig::new(Vec3::ZERO, CameraRigConfig::default());
    let base_fov = rig.current_fov;

    // Fast movement over several ticks to let FOV dilate
    for _ in 0..30 {
        rig.update(&fast_target, 0.016);
    }
    assert!(rig.current_fov > base_fov, "FOV must dilate at high target velocity");
}

#[test]
fn test_dual_target_framing() {
    let player = Vec3::new(-2.0, 0.0, 0.0);
    let enemy = Vec3::new(2.0, 0.0, 0.0);
    let target = FramingTarget::combat_lock_on(player, Vec3::ZERO, enemy);

    let focal = target.focal_point();
    // Weight 0.45 should place focal point near x = -0.2 (between player and enemy)
    assert!(focal.x > -2.0 && focal.x < 2.0);
}
