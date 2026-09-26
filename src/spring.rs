use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Type of spring oscillation damping.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SpringDampingType {
    /// Critically damped (zeta = 1.0): Fastest convergence with zero overshoot.
    CriticallyDamped,
    /// Underdamped (zeta < 1.0): Springy oscillation with overshoot.
    Underdamped(f32),
    /// Overdamped (zeta > 1.0): Sluggish exponential decay without oscillation.
    Overdamped(f32),
}

/// Configuration parameters for a 2nd-order mechanical spring.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpringConfig {
    /// Natural angular frequency omega (rad/s). Higher values respond faster.
    pub angular_frequency: f32,
    /// Damping ratio zeta. 1.0 = critically damped, < 1.0 = springy, > 1.0 = sluggish.
    pub damping_ratio: f32,
}

impl Default for SpringConfig {
    fn default() -> Self {
        Self {
            angular_frequency: 10.0,
            damping_ratio: 1.0, // Critically damped by default
        }
    }
}

impl SpringConfig {
    pub fn new(angular_frequency: f32, damping_ratio: f32) -> Self {
        Self {
            angular_frequency: angular_frequency.max(0.001),
            damping_ratio: damping_ratio.max(0.0),
        }
    }

    /// Snappy camera response with minimal lag.
    pub fn stiff() -> Self {
        Self {
            angular_frequency: 20.0,
            damping_ratio: 1.0,
        }
    }

    /// Cinematic loose trailing feel with slight drift.
    pub fn cinematic() -> Self {
        Self {
            angular_frequency: 6.0,
            damping_ratio: 1.0,
        }
    }

    /// Bouncy spring response for game feel impacts.
    pub fn bouncy() -> Self {
        Self {
            angular_frequency: 12.0,
            damping_ratio: 0.6,
        }
    }
}

/// Analytical 2nd-order critically damped positional spring for scalar floats.
#[inline]
pub fn update_scalar_spring(
    current: f32,
    velocity: f32,
    target: f32,
    config: &SpringConfig,
    dt: f32,
) -> (f32, f32) {
    if dt <= 0.0 {
        return (current, velocity);
    }

    let omega = config.angular_frequency;
    let zeta = config.damping_ratio;
    let x0 = current - target;
    let v0 = velocity;

    if (zeta - 1.0).abs() < 1e-4 {
        // Case 1: Critically Damped (zeta = 1.0)
        let decay = (-omega * dt).exp();
        let c1 = x0;
        let c2 = v0 + omega * x0;
        let new_x = target + (c1 + c2 * dt) * decay;
        let new_v = (c2 - omega * (c1 + c2 * dt)) * decay;
        (new_x, new_v)
    } else if zeta < 1.0 {
        // Case 2: Underdamped (zeta < 1.0)
        let omega_d = omega * (1.0 - zeta * zeta).sqrt();
        let decay = (-zeta * omega * dt).exp();
        let c1 = x0;
        let c2 = (v0 + zeta * omega * x0) / omega_d;
        let sin_term = (omega_d * dt).sin();
        let cos_term = (omega_d * dt).cos();

        let new_x = target + decay * (c1 * cos_term + c2 * sin_term);
        let new_v = decay
            * ((c2 * omega_d - c1 * zeta * omega) * cos_term
                - (c1 * omega_d + c2 * zeta * omega) * sin_term);
        (new_x, new_v)
    } else {
        // Case 3: Overdamped (zeta > 1.0)
        let alpha = omega * (zeta * zeta - 1.0).sqrt();
        let r1 = -zeta * omega + alpha;
        let r2 = -zeta * omega - alpha;

        let c2 = (v0 - r1 * x0) / (r2 - r1);
        let c1 = x0 - c2;

        let exp1 = (r1 * dt).exp();
        let exp2 = (r2 * dt).exp();

        let new_x = target + c1 * exp1 + c2 * exp2;
        let new_v = c1 * r1 * exp1 + c2 * r2 * exp2;
        (new_x, new_v)
    }
}

/// 3D positional spring implementing exact closed-form 2nd-order harmonic oscillation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3Spring {
    pub position: Vec3,
    pub velocity: Vec3,
    pub config: SpringConfig,
}

impl Vec3Spring {
    pub fn new(initial_position: Vec3, config: SpringConfig) -> Self {
        Self {
            position: initial_position,
            velocity: Vec3::ZERO,
            config,
        }
    }

    /// Advances the spring towards `target` over timestep `dt` with unconditional stability.
    pub fn update(&mut self, target: Vec3, dt: f32) -> Vec3 {
        let (nx, vx) = update_scalar_spring(self.position.x, self.velocity.x, target.x, &self.config, dt);
        let (ny, vy) = update_scalar_spring(self.position.y, self.velocity.y, target.y, &self.config, dt);
        let (nz, vz) = update_scalar_spring(self.position.z, self.velocity.z, target.z, &self.config, dt);

        self.position = Vec3::new(nx, ny, nz);
        self.velocity = Vec3::new(vx, vy, vz);
        self.position
    }

    /// Instantly teleports the spring to target, clearing accumulated velocity.
    pub fn reset_to(&mut self, position: Vec3) {
        self.position = position;
        self.velocity = Vec3::ZERO;
    }
}

/// Euler angle spring with shortest-path wraparound handling (-PI to +PI).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AngularSpring {
    pub yaw: f32,
    pub pitch: f32,
    pub yaw_velocity: f32,
    pub pitch_velocity: f32,
    pub config: SpringConfig,
}

impl AngularSpring {
    pub fn new(yaw: f32, pitch: f32, config: SpringConfig) -> Self {
        Self {
            yaw,
            pitch,
            yaw_velocity: 0.0,
            pitch_velocity: 0.0,
            config,
        }
    }

    /// Wraps angle difference into [-PI, PI].
    #[inline]
    fn normalize_angle_diff(diff: f32) -> f32 {
        let two_pi = std::f32::consts::TAU;
        let mut d = diff % two_pi;
        if d > std::f32::consts::PI {
            d -= two_pi;
        } else if d < -std::f32::consts::PI {
            d += two_pi;
        }
        d
    }

    pub fn update(&mut self, target_yaw: f32, target_pitch: f32, dt: f32) -> (f32, f32) {
        // Shortest-path unwrapping for yaw
        let yaw_delta = Self::normalize_angle_diff(target_yaw - self.yaw);
        let effective_target_yaw = self.yaw + yaw_delta;

        let (ny, vy) = update_scalar_spring(
            self.yaw,
            self.yaw_velocity,
            effective_target_yaw,
            &self.config,
            dt,
        );
        let (np, vp) = update_scalar_spring(
            self.pitch,
            self.pitch_velocity,
            target_pitch,
            &self.config,
            dt,
        );

        // Normalize state to standard range
        self.yaw = Self::normalize_angle_diff(ny);
        self.pitch = np;
        self.yaw_velocity = vy;
        self.pitch_velocity = vp;

        (self.yaw, self.pitch)
    }

    pub fn reset_to(&mut self, yaw: f32, pitch: f32) {
        self.yaw = Self::normalize_angle_diff(yaw);
        self.pitch = pitch;
        self.yaw_velocity = 0.0;
        self.pitch_velocity = 0.0;
    }
}

/// Rotational spring tracking quaternions along the shortest geodesic arc.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct QuaternionSpring {
    pub current: Quat,
    pub angular_velocity: Vec3,
    pub config: SpringConfig,
}

impl QuaternionSpring {
    pub fn new(initial: Quat, config: SpringConfig) -> Self {
        Self {
            current: initial.normalize(),
            angular_velocity: Vec3::ZERO,
            config,
        }
    }

    pub fn update(&mut self, target: Quat, dt: f32) -> Quat {
        if dt <= 0.0 {
            return self.current;
        }

        let mut target_norm = target.normalize();
        // Shortest arc check
        if self.current.dot(target_norm) < 0.0 {
            target_norm = -target_norm;
        }

        // Error quaternion: q_diff = target * current^-1
        let q_diff = target_norm * self.current.inverse();
        let (axis, angle) = q_diff.to_axis_angle();
        let angle_clamped = Self::normalize_angle(angle);
        let rotation_error = axis * angle_clamped;

        // Apply 3D vector spring to rotational error
        let (rx, vx) = update_scalar_spring(0.0, self.angular_velocity.x, rotation_error.x, &self.config, dt);
        let (ry, vy) = update_scalar_spring(0.0, self.angular_velocity.y, rotation_error.y, &self.config, dt);
        let (rz, vz) = update_scalar_spring(0.0, self.angular_velocity.z, rotation_error.z, &self.config, dt);

        let delta_rot = Vec3::new(rx, ry, rz);
        self.angular_velocity = Vec3::new(vx, vy, vz);

        let delta_angle = delta_rot.length();
        if delta_angle > 1e-6 {
            let delta_q = Quat::from_axis_angle(delta_rot / delta_angle, delta_angle);
            self.current = (delta_q * self.current).normalize();
        }

        self.current
    }

    #[inline]
    fn normalize_angle(angle: f32) -> f32 {
        let two_pi = std::f32::consts::TAU;
        let mut a = angle % two_pi;
        if a > std::f32::consts::PI {
            a -= two_pi;
        } else if a < -std::f32::consts::PI {
            a += two_pi;
        }
        a
    }

    pub fn reset_to(&mut self, rotation: Quat) {
        self.current = rotation.normalize();
        self.angular_velocity = Vec3::ZERO;
    }
}
