use glam::Vec3;
use serde::{Deserialize, Serialize};

/// Geometric ray in 3D space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }

    #[inline]
    pub fn point_at(&self, t: f32) -> Vec3 {
        self.origin + self.direction * t
    }
}

/// Hit record from a sphere-cast query.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OcclusionHit {
    /// Distance traveled along the sweep ray before impact.
    pub distance: f32,
    /// Point of contact on the obstacle surface.
    pub point: Vec3,
    /// Surface normal vector at contact point.
    pub normal: Vec3,
    /// Fraction along ray length [0.0, 1.0].
    pub fraction: f32,
}

/// 3D collision primitives supported for camera occlusion checks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Collider {
    Sphere { center: Vec3, radius: f32 },
    Aabb { min: Vec3, max: Vec3 },
    Plane { point: Vec3, normal: Vec3 },
    Triangle { v0: Vec3, v1: Vec3, v2: Vec3 },
}

impl Collider {
    /// Sweeps a sphere with radius `sphere_radius` along the ray from `origin` to `target_pos`.
    /// Returns the hit info if an intersection occurs before `max_dist`.
    pub fn sweep_sphere(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        sphere_radius: f32,
    ) -> Option<OcclusionHit> {
        match self {
            Collider::Sphere { center, radius } => {
                // Swept sphere vs sphere is equivalent to ray vs sphere with radius = r1 + r2
                let total_radius = radius + sphere_radius;
                let oc = origin - *center;
                let b = oc.dot(dir);
                let c = oc.dot(oc) - total_radius * total_radius;
                let discriminant = b * b - c;

                if discriminant < 0.0 {
                    return None;
                }

                let sqrt_d = discriminant.sqrt();
                let mut t = -b - sqrt_d;
                if t < 0.0 {
                    t = -b + sqrt_d;
                }

                if t >= 0.0 && t <= max_dist {
                    let sphere_center_at_hit = origin + dir * t;
                    let normal = (sphere_center_at_hit - *center).normalize_or_zero();
                    let hit_point = *center + normal * *radius;
                    Some(OcclusionHit {
                        distance: t,
                        point: hit_point,
                        normal,
                        fraction: (t / max_dist).clamp(0.0, 1.0),
                    })
                } else {
                    None
                }
            }

            Collider::Aabb { min, max } => {
                // Conservative swept sphere vs AABB: Raycast against expanded AABB by sphere_radius
                let exp_min = *min - Vec3::splat(sphere_radius);
                let exp_max = *max + Vec3::splat(sphere_radius);

                let inv_d = Vec3::new(
                    if dir.x.abs() > 1e-6 { 1.0 / dir.x } else { 1e10 },
                    if dir.y.abs() > 1e-6 { 1.0 / dir.y } else { 1e10 },
                    if dir.z.abs() > 1e-6 { 1.0 / dir.z } else { 1e10 },
                );

                let t0 = (exp_min - origin) * inv_d;
                let t1 = (exp_max - origin) * inv_d;

                let tmin_v = t0.min(t1);
                let tmax_v = t0.max(t1);

                let t_enter = tmin_v.max_element();
                let t_exit = tmax_v.min_element();

                if t_enter <= t_exit && t_exit >= 0.0 && t_enter <= max_dist {
                    let hit_t = t_enter.max(0.0);
                    let hit_center = origin + dir * hit_t;

                    // Approximate normal based on which face was struck
                    let mut normal = Vec3::ZERO;
                    if (hit_center.x - exp_min.x).abs() < 1e-2 { normal.x = -1.0; }
                    else if (hit_center.x - exp_max.x).abs() < 1e-2 { normal.x = 1.0; }
                    else if (hit_center.y - exp_min.y).abs() < 1e-2 { normal.y = -1.0; }
                    else if (hit_center.y - exp_max.y).abs() < 1e-2 { normal.y = 1.0; }
                    else if (hit_center.z - exp_min.z).abs() < 1e-2 { normal.z = -1.0; }
                    else if (hit_center.z - exp_max.z).abs() < 1e-2 { normal.z = 1.0; }
                    else { normal = -dir; }

                    let hit_point = hit_center - normal * sphere_radius;

                    Some(OcclusionHit {
                        distance: hit_t,
                        point: hit_point,
                        normal,
                        fraction: (hit_t / max_dist).clamp(0.0, 1.0),
                    })
                } else {
                    None
                }
            }

            Collider::Plane { point, normal } => {
                let n = normal.normalize();
                let denom = n.dot(dir);
                if denom.abs() < 1e-6 {
                    return None;
                }

                // Plane pushed towards ray origin by sphere radius along normal
                let effective_point = *point + n * sphere_radius;
                let t = n.dot(effective_point - origin) / denom;

                if t >= 0.0 && t <= max_dist {
                    let hit_point = origin + dir * t - n * sphere_radius;
                    Some(OcclusionHit {
                        distance: t,
                        point: hit_point,
                        normal: n,
                        fraction: (t / max_dist).clamp(0.0, 1.0),
                    })
                } else {
                    None
                }
            }

            Collider::Triangle { v0, v1, v2 } => {
                // Möller–Trumbore ray-triangle intersection on triangle offset by sphere radius
                let edge1 = *v1 - *v0;
                let edge2 = *v2 - *v0;
                let normal = edge1.cross(edge2).normalize_or_zero();

                let effective_v0 = *v0 + normal * sphere_radius;
                let h = dir.cross(edge2);
                let a = edge1.dot(h);

                if a.abs() < 1e-6 {
                    return None;
                }

                let f = 1.0 / a;
                let s = origin - effective_v0;
                let u = f * s.dot(h);
                if !(0.0..=1.0).contains(&u) {
                    return None;
                }

                let q = s.cross(edge1);
                let v = f * dir.dot(q);
                if v < 0.0 || u + v > 1.0 {
                    return None;
                }

                let t = f * edge2.dot(q);
                if t >= 0.0 && t <= max_dist {
                    let hit_point = origin + dir * t - normal * sphere_radius;
                    Some(OcclusionHit {
                        distance: t,
                        point: hit_point,
                        normal,
                        fraction: (t / max_dist).clamp(0.0, 1.0),
                    })
                } else {
                    None
                }
            }
        }
    }
}

/// Collection of triangular faces representing an environment collision mesh.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CollisionMesh {
    pub colliders: Vec<Collider>,
}

impl CollisionMesh {
    pub fn new() -> Self {
        Self {
            colliders: Vec::new(),
        }
    }

    pub fn add(&mut self, collider: Collider) {
        self.colliders.push(collider);
    }

    pub fn add_box(&mut self, min: Vec3, max: Vec3) {
        self.colliders.push(Collider::Aabb { min, max });
    }

    pub fn add_sphere(&mut self, center: Vec3, radius: f32) {
        self.colliders.push(Collider::Sphere { center, radius });
    }

    pub fn add_ground_plane(&mut self, y: f32) {
        self.colliders.push(Collider::Plane {
            point: Vec3::new(0.0, y, 0.0),
            normal: Vec3::Y,
        });
    }

    pub fn sweep_sphere(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        radius: f32,
    ) -> Option<OcclusionHit> {
        let mut nearest: Option<OcclusionHit> = None;

        for collider in &self.colliders {
            if let Some(hit) = collider.sweep_sphere(origin, dir, max_dist, radius) {
                if let Some(ref best) = nearest {
                    if hit.distance < best.distance {
                        nearest = Some(hit);
                    }
                } else {
                    nearest = Some(hit);
                }
            }
        }

        nearest
    }
}

/// Full result of a sphere-cast camera pull-in query.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SphereCastResult {
    /// Ideal unobstructed camera world position.
    pub ideal_position: Vec3,
    /// Occlusion-resolved camera world position.
    pub resolved_position: Vec3,
    /// Effective camera distance from focal target.
    pub current_distance: f32,
    /// Ideal camera distance from focal target.
    pub ideal_distance: f32,
    /// Whether geometry currently occludes the ideal camera position.
    pub is_occluded: bool,
    /// Optional hit details on the colliding geometry.
    pub hit: Option<OcclusionHit>,
}

/// Hysteresis filter to eliminate camera pop and jitter when clipping against geometry.
///
/// Implements asymmetric tracking:
/// - Pull-in (obstacle detection): Fast instant or high-speed pull-in to avoid near-plane clipping.
/// - Push-out (recovery): Smooth delayed recovery rate back to ideal distance.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HysteresisFilter {
    pub current_distance: f32,
    /// Speed of zooming back out when clear of obstacles (units/sec).
    pub recovery_speed: f32,
    /// Minimum camera distance allowed from target focal point.
    pub min_distance: f32,
}

impl Default for HysteresisFilter {
    fn default() -> Self {
        Self {
            current_distance: 5.0,
            recovery_speed: 4.0,
            min_distance: 0.5,
        }
    }
}

impl HysteresisFilter {
    pub fn new(initial_distance: f32, min_distance: f32, recovery_speed: f32) -> Self {
        Self {
            current_distance: initial_distance.max(min_distance),
            recovery_speed: recovery_speed.max(0.1),
            min_distance: min_distance.max(0.01),
        }
    }

    /// Updates smoothed camera distance based on detected hit distance and delta time.
    pub fn update(&mut self, target_distance: f32, dt: f32) -> f32 {
        let clamped_target = target_distance.max(self.min_distance);

        if clamped_target < self.current_distance {
            // Obstructed: Pull camera in rapidly to prevent clipping through walls
            self.current_distance = clamped_target;
        } else {
            // Clear: Push camera back out smoothly with hysteresis
            let step = self.recovery_speed * dt;
            self.current_distance = (self.current_distance + step).min(clamped_target);
        }

        self.current_distance
    }

    pub fn reset_to(&mut self, distance: f32) {
        self.current_distance = distance.max(self.min_distance);
    }
}

/// Evan Todd's Sphere-Cast Camera Occlusion Solver.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OcclusionSolver {
    /// Radius of the bounding sphere swept along camera line-of-sight.
    pub near_radius: f32,
    /// Asymmetric hysteresis filter smoothing camera distance.
    pub hysteresis: HysteresisFilter,
    /// Collision geometry environment mesh.
    pub environment: CollisionMesh,
}

impl OcclusionSolver {
    pub fn new(near_radius: f32, initial_dist: f32, min_dist: f32, recovery_speed: f32) -> Self {
        Self {
            near_radius: near_radius.max(0.01),
            hysteresis: HysteresisFilter::new(initial_dist, min_dist, recovery_speed),
            environment: CollisionMesh::new(),
        }
    }

    /// Resolves occlusion between target focal point and ideal camera position.
    pub fn solve(&mut self, target: Vec3, ideal_pos: Vec3, dt: f32) -> SphereCastResult {
        let to_camera = ideal_pos - target;
        let ideal_dist = to_camera.length();

        if ideal_dist < 1e-4 {
            return SphereCastResult {
                ideal_position: ideal_pos,
                resolved_position: ideal_pos,
                current_distance: 0.0,
                ideal_distance: 0.0,
                is_occluded: false,
                hit: None,
            };
        }

        let dir = to_camera / ideal_dist;
        let hit = self.environment.sweep_sphere(target, dir, ideal_dist, self.near_radius);

        let target_dist = if let Some(ref h) = hit {
            h.distance.max(self.hysteresis.min_distance)
        } else {
            ideal_dist
        };

        let smoothed_dist = self.hysteresis.update(target_dist, dt);
        let resolved_position = target + dir * smoothed_dist;

        SphereCastResult {
            ideal_position: ideal_pos,
            resolved_position,
            current_distance: smoothed_dist,
            ideal_distance: ideal_dist,
            is_occluded: hit.is_some(),
            hit,
        }
    }
}
