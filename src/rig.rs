use crate::occlusion::{CollisionMesh, OcclusionSolver, SphereCastResult};
use crate::spring::{AngularSpring, SpringConfig, Vec3Spring};
use glam::{Mat4, Vec3};
use serde::{Deserialize, Serialize};

/// Target entities framed by the camera rig.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FramingTarget {
    /// World position of primary subject (e.g. player character).
    pub primary_position: Vec3,
    /// Velocity of primary subject for lead anticipation and speed-based FOV.
    pub primary_velocity: Vec3,
    /// Optional secondary target for dual combat lock-on framing.
    pub secondary_position: Option<Vec3>,
    /// Interpolation weight towards secondary target (0.0 = only player, 0.5 = midpoint).
    pub dual_framing_weight: f32,
}

impl FramingTarget {
    pub fn single(position: Vec3, velocity: Vec3) -> Self {
        Self {
            primary_position: position,
            primary_velocity: velocity,
            secondary_position: None,
            dual_framing_weight: 0.0,
        }
    }

    pub fn combat_lock_on(player_pos: Vec3, player_vel: Vec3, enemy_pos: Vec3) -> Self {
        Self {
            primary_position: player_pos,
            primary_velocity: player_vel,
            secondary_position: Some(enemy_pos),
            dual_framing_weight: 0.45,
        }
    }

    /// Computes effective focal point framed between targets.
    pub fn focal_point(&self) -> Vec3 {
        if let Some(sec) = self.secondary_position {
            let weight = self.dual_framing_weight.clamp(0.0, 1.0);
            self.primary_position.lerp(sec, weight)
        } else {
            self.primary_position
        }
    }
}

/// Configuration parameters for the camera rig.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CameraRigConfig {
    /// Desired camera distance in unobstructed open space.
    pub ideal_distance: f32,
    /// Minimum allowed camera distance when pulled in by obstacles.
    pub min_distance: f32,
    /// Local shoulder offset (X = right, Y = up, Z = forward).
    pub shoulder_offset: Vec3,
    /// Minimum camera pitch clamp in radians (prevents ground gimbal lock).
    pub min_pitch: f32,
    /// Maximum camera pitch clamp in radians (prevents zenith gimbal lock).
    pub max_pitch: f32,
    /// Base vertical field of view in radians.
    pub base_fov_y: f32,
    /// Maximum additional FOV dilation at top speed in radians.
    pub max_fov_dilation: f32,
    /// Speed at which maximum FOV dilation is reached.
    pub speed_for_max_fov: f32,
    /// Near clip plane distance.
    pub near_clip: f32,
    /// Far clip plane distance.
    pub far_clip: f32,
    /// Viewport aspect ratio (width / height).
    pub aspect_ratio: f32,
    /// Position spring configuration.
    pub position_spring: SpringConfig,
    /// Rotation spring configuration.
    pub rotation_spring: SpringConfig,
    /// Auto-align camera yaw behind velocity vector (vehicle chase mode).
    pub auto_align_chase: bool,
    /// Rate of chase yaw auto-alignment (rad/sec).
    pub chase_align_rate: f32,
}

impl Default for CameraRigConfig {
    fn default() -> Self {
        Self {
            ideal_distance: 5.0,
            min_distance: 0.8,
            shoulder_offset: Vec3::new(0.6, 0.4, 0.0), // Classic third-person over-the-shoulder
            min_pitch: -75.0f32.to_radians(),
            max_pitch: 80.0f32.to_radians(),
            base_fov_y: 60.0f32.to_radians(),
            max_fov_dilation: 15.0f32.to_radians(),
            speed_for_max_fov: 18.0,
            near_clip: 0.1,
            far_clip: 1000.0,
            aspect_ratio: 16.0 / 9.0,
            position_spring: SpringConfig::cinematic(),
            rotation_spring: SpringConfig::new(12.0, 1.0),
            auto_align_chase: false,
            chase_align_rate: 2.0,
        }
    }
}

/// Output view and projection matrices computed by the camera rig.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ViewProjection {
    pub view: Mat4,
    pub projection: Mat4,
    pub view_projection: Mat4,
    pub eye_position: Vec3,
    pub target_position: Vec3,
    pub fov_y: f32,
}

/// Evan Todd's 3D third-person camera rig.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraRig {
    pub config: CameraRigConfig,
    pub yaw: f32,
    pub pitch: f32,
    pub current_fov: f32,
    pub occlusion: OcclusionSolver,
    position_spring: Vec3Spring,
    angular_spring: AngularSpring,
    last_eye_position: Vec3,
}

impl CameraRig {
    pub fn new(initial_target: Vec3, config: CameraRigConfig) -> Self {
        let occlusion = OcclusionSolver::new(
            config.near_clip * 1.5,
            config.ideal_distance,
            config.min_distance,
            5.0,
        );

        let initial_eye = initial_target - Vec3::Z * config.ideal_distance;
        let position_spring = Vec3Spring::new(initial_eye, config.position_spring);
        let angular_spring = AngularSpring::new(0.0, 0.0, config.rotation_spring);

        Self {
            config,
            yaw: 0.0,
            pitch: 0.0,
            current_fov: config.base_fov_y,
            occlusion,
            position_spring,
            angular_spring,
            last_eye_position: initial_eye,
        }
    }

    /// Modifies orbit yaw and pitch with user mouse/stick input.
    pub fn add_orbit_input(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(self.config.min_pitch, self.config.max_pitch);
    }

    /// Sets collision environment mesh for sphere-cast occlusion.
    pub fn set_collision_mesh(&mut self, mesh: CollisionMesh) {
        self.occlusion.environment = mesh;
    }

    /// Advances camera simulation tick.
    pub fn update(&mut self, target: &FramingTarget, dt: f32) -> (ViewProjection, SphereCastResult) {
        let focal_point = target.focal_point();

        // 1. Optional Vehicle Chase auto-alignment
        if self.config.auto_align_chase {
            let speed_sq = target.primary_velocity.length_squared();
            if speed_sq > 1.0 {
                let move_dir = target.primary_velocity.normalize();
                // Target yaw facing behind movement
                let desired_yaw = move_dir.x.atan2(move_dir.z);
                let yaw_diff = desired_yaw - self.yaw;
                self.yaw += yaw_diff * (self.config.chase_align_rate * dt).min(1.0);
            }
        }

        // 2. Angular spring smoothing
        let (smoothed_yaw, smoothed_pitch) = self.angular_spring.update(self.yaw, self.pitch, dt);

        // 3. Compute unoccluded ideal eye position relative to target
        let rot_y = Mat4::from_rotation_y(smoothed_yaw);
        let rot_x = Mat4::from_rotation_x(smoothed_pitch);
        let orientation = rot_y * rot_x;

        let forward = (orientation.transform_vector3(Vec3::NEG_Z)).normalize();
        let right = (orientation.transform_vector3(Vec3::X)).normalize();
        let up = (orientation.transform_vector3(Vec3::Y)).normalize();

        // Apply shoulder offset in camera local orientation space
        let shoulder_world = right * self.config.shoulder_offset.x
            + up * self.config.shoulder_offset.y
            + forward * self.config.shoulder_offset.z;

        let origin = focal_point + shoulder_world;
        let ideal_eye = origin - forward * self.config.ideal_distance;

        // 4. Sphere-Cast Occlusion Solver
        let occlusion_res = self.occlusion.solve(origin, ideal_eye, dt);

        // 5. Positional Spring Damping on the resolved position
        let smoothed_eye = self.position_spring.update(occlusion_res.resolved_position, dt);
        self.last_eye_position = smoothed_eye;

        // 6. Dynamic Speed-Based FOV Dilation
        let speed = target.primary_velocity.length();
        let speed_factor = (speed / self.config.speed_for_max_fov).clamp(0.0, 1.0);
        let target_fov = self.config.base_fov_y + speed_factor * self.config.max_fov_dilation;
        // Smooth FOV transition
        self.current_fov += (target_fov - self.current_fov) * (8.0 * dt).min(1.0);

        // 7. Matrix generation
        let view = Mat4::look_at_rh(smoothed_eye, focal_point, Vec3::Y);
        let projection = Mat4::perspective_rh(
            self.current_fov,
            self.config.aspect_ratio,
            self.config.near_clip,
            self.config.far_clip,
        );

        let view_proj = ViewProjection {
            view,
            projection,
            view_projection: projection * view,
            eye_position: smoothed_eye,
            target_position: focal_point,
            fov_y: self.current_fov,
        };

        (view_proj, occlusion_res)
    }

    /// Teleports camera instantly to target, clearing spring lag and hysteresis.
    pub fn snap_to(&mut self, target: &FramingTarget) {
        let focal = target.focal_point();
        let eye = focal - Vec3::Z * self.config.ideal_distance;
        self.position_spring.reset_to(eye);
        self.angular_spring.reset_to(self.yaw, self.pitch);
        self.occlusion.hysteresis.reset_to(self.config.ideal_distance);
        self.last_eye_position = eye;
    }
}
