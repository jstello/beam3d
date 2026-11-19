use eframe::egui;
use nalgebra::{Matrix4, Point3, Vector3, Vector4};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkplaneKind {
    /// Horizontal floor plane (`Z = offset`).
    XY,
    /// Vertical front elevation plane (`Y = offset`).
    XZ,
    /// Vertical side elevation plane (`X = offset`).
    YZ,
}

impl WorkplaneKind {
    pub const ALL: [WorkplaneKind; 3] = [WorkplaneKind::XY, WorkplaneKind::XZ, WorkplaneKind::YZ];

    pub fn short_label(self) -> &'static str {
        match self {
            WorkplaneKind::XY => "XY",
            WorkplaneKind::XZ => "XZ",
            WorkplaneKind::YZ => "YZ",
        }
    }

    pub fn full_label(self) -> &'static str {
        match self {
            WorkplaneKind::XY => "XY (Floor)",
            WorkplaneKind::XZ => "XZ (Vertical Front)",
            WorkplaneKind::YZ => "YZ (Vertical Side)",
        }
    }

    pub fn normal_axis_name(self) -> &'static str {
        match self {
            WorkplaneKind::XY => "Z",
            WorkplaneKind::XZ => "Y",
            WorkplaneKind::YZ => "X",
        }
    }

    pub fn normal_index(self) -> usize {
        match self {
            WorkplaneKind::XY => 2,
            WorkplaneKind::XZ => 1,
            WorkplaneKind::YZ => 0,
        }
    }

    pub fn in_plane_indices(self) -> (usize, usize) {
        match self {
            WorkplaneKind::XY => (0, 1),
            WorkplaneKind::XZ => (0, 2),
            WorkplaneKind::YZ => (1, 2),
        }
    }

    pub fn next(self) -> Self {
        match self {
            WorkplaneKind::XY => WorkplaneKind::XZ,
            WorkplaneKind::XZ => WorkplaneKind::YZ,
            WorkplaneKind::YZ => WorkplaneKind::XY,
        }
    }
}

#[derive(Debug, Clone)]
pub struct OrbitCamera {
    pub target: Vector3<f32>,
    /// Azimuth angle around +Z axis (radians).
    pub yaw: f32,
    /// Elevation angle above X-Y plane (radians).
    pub pitch: f32,
    pub distance: f32,
    pub fov_y_rad: f32,
    pub z_near: f32,
    pub z_far: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vector3::new(6.0, 5.5, 4.5),
            yaw: -0.95,
            pitch: 0.48,
            distance: 26.0,
            fov_y_rad: 42.0_f32.to_radians(),
            z_near: 0.05,
            z_far: 2000.0,
        }
    }
}

impl OrbitCamera {
    pub fn eye_position(&self) -> Vector3<f32> {
        let cp = self.pitch.cos();
        let sp = self.pitch.sin();
        let cy = self.yaw.cos();
        let sy = self.yaw.sin();
        self.target + Vector3::new(self.distance * cp * cy, self.distance * cp * sy, self.distance * sp)
    }

    pub fn basis_vectors(&self) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
        let eye = self.eye_position();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = if forward.dot(&world_up).abs() < 0.999 {
            forward.cross(&world_up).normalize()
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        };
        let up = right.cross(&forward).normalize();
        (right, up, forward)
    }

    pub fn view_matrix(&self) -> Matrix4<f32> {
        let eye = Point3::from(self.eye_position());
        let target = Point3::from(self.target);
        let (right, up, _forward) = self.basis_vectors();
        let _ = right;
        Matrix4::look_at_rh(&eye, &target, &up)
    }

    /// Right-handed perspective matrix mapping view-space depth into WebGPU `[0, 1]` NDC z-range.
    pub fn proj_matrix(&self, aspect: f32) -> Matrix4<f32> {
        let aspect = aspect.max(0.05);
        let f = 1.0 / (0.5 * self.fov_y_rad).tan();
        let n = self.z_near;
        let far = self.z_far;
        let mut p = Matrix4::zeros();
        p[(0, 0)] = f / aspect;
        p[(1, 1)] = f;
        p[(2, 2)] = far / (n - far);
        p[(2, 3)] = (n * far) / (n - far);
        p[(3, 2)] = -1.0;
        p
    }

    pub fn view_proj_matrix(&self, aspect: f32) -> Matrix4<f32> {
        self.proj_matrix(aspect) * self.view_matrix()
    }

    pub fn orbit(&mut self, delta_px: egui::Vec2) {
        let sensitivity = 0.0065;
        self.yaw -= delta_px.x * sensitivity;
        self.pitch = (self.pitch + delta_px.y * sensitivity).clamp(-1.52, 1.52);
    }

    pub fn pan(&mut self, delta_px: egui::Vec2, viewport_height: f32) {
        let (right, up, _) = self.basis_vectors();
        let scale = 2.0 * self.distance * (0.5 * self.fov_y_rad).tan() / viewport_height.max(50.0);
        self.target -= right * (delta_px.x * scale) - up * (delta_px.y * scale);
    }

    pub fn zoom(&mut self, scroll_delta: f32) {
        let factor = (-scroll_delta * 0.0022).exp();
        self.distance = (self.distance * factor).clamp(0.4, 800.0);
        self.z_near = (self.distance * 0.005).clamp(0.01, 1.0);
        self.z_far = (self.distance * 40.0).max(500.0);
    }

    pub fn fit_to_bounds(&mut self, min_p: [f64; 3], max_p: [f64; 3]) {
        let center = Vector3::new(
            0.5 * (min_p[0] + max_p[0]) as f32,
            0.5 * (min_p[1] + max_p[1]) as f32,
            0.5 * (min_p[2] + max_p[2]) as f32,
        );
        let dx = (max_p[0] - min_p[0]) as f32;
        let dy = (max_p[1] - min_p[1]) as f32;
        let dz = (max_p[2] - min_p[2]) as f32;
        let diag = (dx * dx + dy * dy + dz * dz).sqrt().max(2.0);
        self.target = center;
        self.distance = (diag * 1.45).clamp(2.5, 500.0);
        self.z_near = (self.distance * 0.005).clamp(0.01, 1.0);
        self.z_far = (self.distance * 40.0).max(500.0);
    }

    pub fn set_preset_iso(&mut self) {
        self.yaw = -0.95;
        self.pitch = 0.48;
    }

    pub fn set_preset_front_xz(&mut self) {
        self.yaw = -std::f32::consts::FRAC_PI_2;
        self.pitch = 0.0;
    }

    pub fn set_preset_side_yz(&mut self) {
        self.yaw = 0.0;
        self.pitch = 0.0;
    }

    pub fn set_preset_top_xy(&mut self) {
        self.yaw = -std::f32::consts::FRAC_PI_2;
        self.pitch = 1.50;
    }

    /// Projects a 3D world coordinate `[X, Y, Z]` onto 2D `egui` screen coordinates inside `rect`.
    pub fn project_to_screen(&self, pos: [f64; 3], rect: egui::Rect) -> Option<(egui::Pos2, f32)> {
        let aspect = rect.width() / rect.height().max(1.0);
        let vp = self.view_proj_matrix(aspect);
        let clip = vp * Vector4::new(pos[0] as f32, pos[1] as f32, pos[2] as f32, 1.0);
        if clip.w <= 1e-4 {
            return None;
        }
        let ndc_x = clip.x / clip.w;
        let ndc_y = clip.y / clip.w;
        let ndc_z = clip.z / clip.w;
        if !(0.0..=1.0).contains(&ndc_z) {
            return None;
        }
        let sx = rect.left() + (ndc_x * 0.5 + 0.5) * rect.width();
        let sy = rect.top() + (0.5 - ndc_y * 0.5) * rect.height();
        Some((egui::pos2(sx, sy), ndc_z))
    }

    /// Computes a world-space ray `(origin, unit_dir)` from a 2D cursor position inside `rect`.
    pub fn screen_ray(&self, cursor: egui::Pos2, rect: egui::Rect) -> (Vector3<f64>, Vector3<f64>) {
        let nx = ((cursor.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
        let ny = 1.0 - ((cursor.y - rect.top()) / rect.height().max(1.0)) * 2.0;
        let aspect = rect.width() / rect.height().max(1.0);
        let tan_half = (0.5 * self.fov_y_rad).tan();

        let (right, up, forward) = self.basis_vectors();
        let dir_f32 = (forward + right * (nx * aspect * tan_half) + up * (ny * tan_half)).normalize();
        let eye = self.eye_position();
        (
            Vector3::new(eye.x as f64, eye.y as f64, eye.z as f64),
            Vector3::new(dir_f32.x as f64, dir_f32.y as f64, dir_f32.z as f64),
        )
    }

    /// Intersects a cursor ray with an `XY`, `XZ`, or `YZ` construction plane at `coord[normal] = offset`.
    pub fn intersect_workplane(
        &self,
        cursor: egui::Pos2,
        rect: egui::Rect,
        kind: WorkplaneKind,
        offset: f64,
    ) -> Option<[f64; 3]> {
        let (orig, dir) = self.screen_ray(cursor, rect);
        let n_idx = kind.normal_index();
        let denom = dir[n_idx];
        if denom.abs() < 1e-4 {
            return None;
        }
        let t = (offset - orig[n_idx]) / denom;
        if t <= 0.0 || t > 2000.0 {
            return None;
        }
        let hit = orig + dir * t;
        let mut out = [hit.x, hit.y, hit.z];
        out[n_idx] = offset;
        Some(out)
    }
}

/// Shortest screen-space distance from `cursor` to a 2D line segment `[a, b]`.
pub fn point_to_segment_dist_2d(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len2 = ab.length_sq();
    if len2 < 1e-6 {
        return ap.length();
    }
    let t = (ap.dot(ab) / len2).clamp(0.0, 1.0);
    let proj = a + ab * t;
    (p - proj).length()
}
