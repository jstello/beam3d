use crate::model::{GravitySettings, LoadCoordSystem};
use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, Vector3};
use serde::{Deserialize, Serialize};

/// Orthonormal local coordinate frame for a flat 3D shell element (Tri3 or Quad4).
#[derive(Debug, Clone)]
pub struct ShellFrame {
    /// Element centroid in global coordinates.
    pub centroid: [f64; 3],
    /// Total surface area in m².
    pub area: f64,
    /// 3x3 rotation matrix mapping global -> local (`v_local = r_mat * v_global`).
    /// Rows are `e1` (in-plane x), `e2` (in-plane y), `e3` (outward normal z).
    pub r_mat: Matrix3<f64>,
    /// Local 2D node coordinates `(x, y)` relative to the element centroid.
    pub local_xy: Vec<[f64; 2]>,
}

impl ShellFrame {
    pub fn compute(positions: &[[f64; 3]]) -> Result<Self, String> {
        let n = positions.len();
        if n != 3 && n != 4 {
            return Err(format!("Unsupported shell node count: {n}"));
        }

        let mut centroid_v = Vector3::zeros();
        for p in positions {
            centroid_v += Vector3::from(*p);
        }
        centroid_v /= n as f64;

        let p0 = Vector3::from(positions[0]);
        let p1 = Vector3::from(positions[1]);
        let p2 = Vector3::from(positions[2]);

        let e1_raw = p1 - p0;
        let e1_len = e1_raw.norm();
        if e1_len < 1e-9 {
            return Err("Degenerate shell element (zero-length edge).".to_string());
        }
        let e1 = e1_raw / e1_len;

        // Compute outward normal using cross product (for Quad4, use both diagonals for best flat-plane fit)
        let normal_raw = if n == 4 {
            let p3 = Vector3::from(positions[3]);
            (p2 - p0).cross(&(p3 - p1))
        } else {
            (p1 - p0).cross(&(p2 - p0))
        };
        let n_len = normal_raw.norm();
        if n_len < 1e-9 {
            return Err("Degenerate shell element (zero area / collinear nodes).".to_string());
        }
        let e3 = normal_raw / n_len;
        let e2 = e3.cross(&e1).normalize();
        // Re-orthogonalize e1 in case Quad4 is slightly warped
        let e1 = e2.cross(&e3).normalize();

        let r_mat = Matrix3::from_rows(&[e1.transpose(), e2.transpose(), e3.transpose()]);

        let mut local_xy = Vec::with_capacity(n);
        for p in positions {
            let rel = Vector3::from(*p) - centroid_v;
            let loc = r_mat * rel;
            local_xy.push([loc.x, loc.y]);
        }

        // Exact 2D polygon area via shoelace formula
        let mut area2 = 0.0;
        for i in 0..n {
            let [x0, y0] = local_xy[i];
            let [x1, y1] = local_xy[(i + 1) % n];
            area2 += x0 * y1 - x1 * y0;
        }
        let area = 0.5 * area2.abs();
        if area < 1e-9 {
            return Err("Degenerate shell element (zero projected area).".to_string());
        }

        Ok(Self {
            centroid: [centroid_v.x, centroid_v.y, centroid_v.z],
            area,
            r_mat,
            local_xy,
        })
    }

    pub fn normal(&self) -> [f64; 3] {
        [self.r_mat[(2, 0)], self.r_mat[(2, 1)], self.r_mat[(2, 2)]]
    }

    pub fn transformation_matrix(&self) -> DMatrix<f64> {
        let n = self.local_xy.len();
        let ndof = n * 6;
        let mut t = DMatrix::<f64>::zeros(ndof, ndof);
        for block in 0..(2 * n) {
            let idx = block * 3;
            for r in 0..3 {
                for c in 0..3 {
                    t[(idx + r, idx + c)] = self.r_mat[(r, c)];
                }
            }
        }
        t
    }
}

/// Computes the local `6n × 6n` Mindlin–Reissner + Plane-Stress shell stiffness matrix.
/// - `Quad4`: 2×2 Gauss for membrane & bending + 1×1 Selective Reduced Integration for transverse shear (locking-free).
/// - `Tri3`: 1-point CST membrane + shear-stabilized Mindlin–Reissner bending/shear.
/// - Includes invariant drilling rotation (`rz`) stabilization so coplanar shells are never singular.
pub fn shell_local_stiffness(
    frame: &ShellFrame,
    e: f64,
    g: f64,
    nu: f64,
    thickness: f64,
) -> Result<DMatrix<f64>, String> {
    let n = frame.local_xy.len();
    let ndof = n * 6;
    let t = thickness.max(1e-5);
    let nu = nu.clamp(0.0, 0.49);

    let dm = membrane_constitutive(e, nu, t);
    let db = bending_constitutive(e, nu, t);
    let ds = shear_constitutive(g, t);

    let mut k = DMatrix::<f64>::zeros(ndof, ndof);

    if n == 3 {
        let (dn_dx, dn_dy, area) = tri3_shape_derivatives(&frame.local_xy)?;
        let n_vals = [1.0 / 3.0; 3];

        let mut bm = DMatrix::<f64>::zeros(3, ndof);
        let mut bb = DMatrix::<f64>::zeros(3, ndof);
        let mut bs = DMatrix::<f64>::zeros(2, ndof);
        fill_b_matrices(&n_vals, &dn_dx, &dn_dy, &mut bm, &mut bb, &mut bs);

        // Residual bending flexibility shear factor prevents thin-triangle shear locking
        let shear_alpha = 1.0 / (1.0 + (0.12 * e * t * t) / (g * area).max(1e-12));

        k += bm.transpose() * dm * &bm * area;
        k += bb.transpose() * db * &bb * area;
        k += bs.transpose() * ds * &bs * (area * shear_alpha);
    } else {
        // Quad4: 2x2 Gauss integration for membrane & bending
        let gp = 1.0 / 3.0_f64.sqrt();
        let gauss_2x2 = [(-gp, -gp), (gp, -gp), (gp, gp), (-gp, gp)];

        for &(xi, eta) in &gauss_2x2 {
            let (n_vals, dn_dx, dn_dy, det_j) = quad4_shape_at(&frame.local_xy, xi, eta)?;
            let mut bm = DMatrix::<f64>::zeros(3, ndof);
            let mut bb = DMatrix::<f64>::zeros(3, ndof);
            let mut bs_dummy = DMatrix::<f64>::zeros(2, ndof);
            fill_b_matrices(&n_vals, &dn_dx, &dn_dy, &mut bm, &mut bb, &mut bs_dummy);

            k += bm.transpose() * dm * &bm * det_j;
            k += bb.transpose() * db * &bb * det_j;
        }

        // Quad4: 1x1 Selective Reduced Integration at (0, 0) for transverse shear (weight = 4.0)
        let (n_vals, dn_dx, dn_dy, det_j) = quad4_shape_at(&frame.local_xy, 0.0, 0.0)?;
        let mut bm_dummy = DMatrix::<f64>::zeros(3, ndof);
        let mut bb_dummy = DMatrix::<f64>::zeros(3, ndof);
        let mut bs = DMatrix::<f64>::zeros(2, ndof);
        fill_b_matrices(&n_vals, &dn_dx, &dn_dy, &mut bm_dummy, &mut bb_dummy, &mut bs);
        k += bs.transpose() * ds * &bs * (4.0 * det_j);
    }

    // Invariant drilling rotation (local rz = DOF 5) stabilization
    let k_drill = 1.0e-4 * e * t * frame.area;
    let inv_n = 1.0 / n as f64;
    for a in 0..n {
        for b in 0..n {
            let val = if a == b {
                k_drill * (1.0 - inv_n + 1.0e-2)
            } else {
                -k_drill * inv_n
            };
            k[(6 * a + 5, 6 * b + 5)] += val;
        }
    }

    Ok(k)
}

/// Computes the total global surface traction vector `[px, py, pz]` (Pa = N/m²) on a shell element,
/// combining self-weight gravity (`rho * t * g`) and all attached `SurfaceLoad` entries.
pub fn total_global_surface_traction(
    frame: &ShellFrame,
    density: f64,
    thickness: f64,
    gravity: &GravitySettings,
    surface_loads: &[(LoadCoordSystem, [f64; 3])],
) -> [f64; 3] {
    let mut p_glob = Vector3::zeros();

    if gravity.enabled {
        p_glob += Vector3::from(gravity.acceleration) * (density * thickness);
    }

    let r_t = frame.r_mat.transpose();
    for &(cs, p) in surface_loads {
        let pv = Vector3::from(p);
        match cs {
            LoadCoordSystem::Global => p_glob += pv,
            LoadCoordSystem::Local => p_glob += r_t * pv,
        }
    }

    [p_glob.x, p_glob.y, p_glob.z]
}

/// Integrates a uniform global surface traction `[px, py, pz]` (N/m²) into consistent
/// global nodal force vectors (size `6 * n_nodes`).
pub fn consistent_global_surface_forces(
    frame: &ShellFrame,
    p_global: [f64; 3],
) -> Result<DVector<f64>, String> {
    let n = frame.local_xy.len();
    let mut f_glob = DVector::<f64>::zeros(n * 6);

    if p_global[0].abs() + p_global[1].abs() + p_global[2].abs() == 0.0 {
        return Ok(f_glob);
    }

    if n == 3 {
        let w = frame.area / 3.0;
        for a in 0..3 {
            f_glob[6 * a] += w * p_global[0];
            f_glob[6 * a + 1] += w * p_global[1];
            f_glob[6 * a + 2] += w * p_global[2];
        }
    } else {
        let gp = 1.0 / 3.0_f64.sqrt();
        let gauss_2x2 = [(-gp, -gp), (gp, -gp), (gp, gp), (-gp, gp)];
        for &(xi, eta) in &gauss_2x2 {
            let (n_vals, _, _, det_j) = quad4_shape_at(&frame.local_xy, xi, eta)?;
            for a in 0..4 {
                let w = n_vals[a] * det_j;
                f_glob[6 * a] += w * p_global[0];
                f_glob[6 * a + 1] += w * p_global[1];
                f_glob[6 * a + 2] += w * p_global[2];
            }
        }
    }

    Ok(f_glob)
}

/// Post-processed results at a 2D shell element's centroid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShellResult {
    pub shell_id: usize,
    pub area: f64,
    pub centroid: [f64; 3],
    pub normal: [f64; 3],
    /// In-plane membrane forces `[Nxx, Nyy, Nxy]` (N/m).
    pub membrane_forces: [f64; 3],
    /// Plate bending moments per unit width `[Mxx, Myy, Mxy]` (N·m/m).
    pub bending_moments: [f64; 3],
    /// Transverse shear forces per unit width `[Vxz, Vyz]` (N/m).
    pub transverse_shears: [f64; 2],
    /// Maximum extreme-fiber von Mises stress across top/bottom surfaces (Pa).
    pub max_von_mises_pa: f64,
    /// Maximum principal/normal surface stress magnitude (Pa).
    pub max_normal_stress_pa: f64,
    /// Yield utilization `max_von_mises_pa / fy`.
    pub utilization: f64,
    /// Centroid displacement magnitude `|u|` (m).
    pub centroid_disp_mag_m: f64,
}

/// Recovers centroid membrane forces, plate moments, transverse shears, and extreme-fiber stresses.
pub fn recover_shell_result(
    shell_id: usize,
    frame: &ShellFrame,
    e: f64,
    g: f64,
    nu: f64,
    fy: f64,
    thickness: f64,
    u_local: &DVector<f64>,
) -> Result<ShellResult, String> {
    let n = frame.local_xy.len();
    let ndof = n * 6;
    let t = thickness.max(1e-5);
    let nu = nu.clamp(0.0, 0.49);

    let dm = membrane_constitutive(e, nu, t);
    let db = bending_constitutive(e, nu, t);
    let ds = shear_constitutive(g, t);

    let (n_vals, dn_dx, dn_dy) = if n == 3 {
        let (dx, dy, _) = tri3_shape_derivatives(&frame.local_xy)?;
        (vec![1.0 / 3.0; 3], dx, dy)
    } else {
        let (nv, dx, dy, _) = quad4_shape_at(&frame.local_xy, 0.0, 0.0)?;
        (nv, dx, dy)
    };

    let mut bm = DMatrix::<f64>::zeros(3, ndof);
    let mut bb = DMatrix::<f64>::zeros(3, ndof);
    let mut bs = DMatrix::<f64>::zeros(2, ndof);
    fill_b_matrices(&n_vals, &dn_dx, &dn_dy, &mut bm, &mut bb, &mut bs);

    let eps = &bm * u_local;
    let kappa = &bb * u_local;
    let gamma = &bs * u_local;

    let n_mem = dm * eps;
    let m_bend = db * kappa;
    let v_shear = ds * gamma;

    // Extreme-fiber stresses on top (+t/2) and bottom (-t/2) surfaces
    let mut max_vm = 0.0_f64;
    let mut max_norm = 0.0_f64;
    for sign in [-1.0_f64, 1.0_f64] {
        let sxx = n_mem[0] / t + sign * (6.0 * m_bend[0]) / (t * t);
        let syy = n_mem[1] / t + sign * (6.0 * m_bend[1]) / (t * t);
        let txy = n_mem[2] / t + sign * (6.0 * m_bend[2]) / (t * t);
        let vm = (sxx * sxx - sxx * syy + syy * syy + 3.0 * txy * txy)
            .max(0.0)
            .sqrt();
        max_vm = max_vm.max(vm);
        max_norm = max_norm.max(sxx.abs()).max(syy.abs());
    }

    // Centroid displacement magnitude
    let mut u_c = Vector3::<f64>::zeros();
    for a in 0..n {
        u_c.x += n_vals[a] * u_local[6 * a];
        u_c.y += n_vals[a] * u_local[6 * a + 1];
        u_c.z += n_vals[a] * u_local[6 * a + 2];
    }

    Ok(ShellResult {
        shell_id,
        area: frame.area,
        centroid: frame.centroid,
        normal: frame.normal(),
        membrane_forces: [n_mem[0], n_mem[1], n_mem[2]],
        bending_moments: [m_bend[0], m_bend[1], m_bend[2]],
        transverse_shears: [v_shear[0], v_shear[1]],
        max_von_mises_pa: max_vm,
        max_normal_stress_pa: max_norm,
        utilization: max_vm / fy.max(1.0),
        centroid_disp_mag_m: u_c.norm(),
    })
}

fn membrane_constitutive(e: f64, nu: f64, t: f64) -> Matrix3<f64> {
    let c = (e * t) / (1.0 - nu * nu);
    Matrix3::new(
        c,
        c * nu,
        0.0,
        c * nu,
        c,
        0.0,
        0.0,
        0.0,
        c * 0.5 * (1.0 - nu),
    )
}

fn bending_constitutive(e: f64, nu: f64, t: f64) -> Matrix3<f64> {
    let d = (e * t * t * t) / (12.0 * (1.0 - nu * nu));
    Matrix3::new(
        d,
        d * nu,
        0.0,
        d * nu,
        d,
        0.0,
        0.0,
        0.0,
        d * 0.5 * (1.0 - nu),
    )
}

fn shear_constitutive(g: f64, t: f64) -> Matrix2<f64> {
    let ks = (5.0 / 6.0) * g * t;
    Matrix2::new(ks, 0.0, 0.0, ks)
}

fn tri3_shape_derivatives(xy: &[[f64; 2]]) -> Result<(Vec<f64>, Vec<f64>, f64), String> {
    let [x1, y1] = xy[0];
    let [x2, y2] = xy[1];
    let [x3, y3] = xy[2];
    let det2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
    if det2.abs() < 1e-12 {
        return Err("Degenerate Tri3 shell element (zero area).".to_string());
    }
    let inv = 1.0 / det2;
    let dn_dx = vec![(y2 - y3) * inv, (y3 - y1) * inv, (y1 - y2) * inv];
    let dn_dy = vec![(x3 - x2) * inv, (x1 - x3) * inv, (x2 - x1) * inv];
    Ok((dn_dx, dn_dy, 0.5 * det2.abs()))
}

fn quad4_shape_at(
    xy: &[[f64; 2]],
    xi: f64,
    eta: f64,
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>, f64), String> {
    let n_vals = vec![
        0.25 * (1.0 - xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 + eta),
        0.25 * (1.0 - xi) * (1.0 + eta),
    ];
    let dn_dxi = [
        -0.25 * (1.0 - eta),
        0.25 * (1.0 - eta),
        0.25 * (1.0 + eta),
        -0.25 * (1.0 + eta),
    ];
    let dn_deta = [
        -0.25 * (1.0 - xi),
        -0.25 * (1.0 + xi),
        0.25 * (1.0 + xi),
        0.25 * (1.0 - xi),
    ];

    let mut j = Matrix2::<f64>::zeros();
    for a in 0..4 {
        j[(0, 0)] += dn_dxi[a] * xy[a][0];
        j[(0, 1)] += dn_dxi[a] * xy[a][1];
        j[(1, 0)] += dn_deta[a] * xy[a][0];
        j[(1, 1)] += dn_deta[a] * xy[a][1];
    }
    let det_j = j.determinant();
    if det_j <= 1e-12 {
        return Err(
            "Degenerate or inverted Quad4 shell element (ensure nodes are ordered CCW/convex)."
                .to_string(),
        );
    }
    let j_inv = j
        .try_inverse()
        .ok_or_else(|| "Singular Jacobian in Quad4 shell element.".to_string())?;

    let mut dn_dx = Vec::with_capacity(4);
    let mut dn_dy = Vec::with_capacity(4);
    for a in 0..4 {
        dn_dx.push(j_inv[(0, 0)] * dn_dxi[a] + j_inv[(0, 1)] * dn_deta[a]);
        dn_dy.push(j_inv[(1, 0)] * dn_dxi[a] + j_inv[(1, 1)] * dn_deta[a]);
    }
    Ok((n_vals, dn_dx, dn_dy, det_j))
}

fn fill_b_matrices(
    n_vals: &[f64],
    dn_dx: &[f64],
    dn_dy: &[f64],
    bm: &mut DMatrix<f64>,
    bb: &mut DMatrix<f64>,
    bs: &mut DMatrix<f64>,
) {
    for a in 0..n_vals.len() {
        let base = a * 6;
        let na = n_vals[a];
        let nx = dn_dx[a];
        let ny = dn_dy[a];

        // Membrane strain [du/dx, dv/dy, du/dy + dv/dx]
        bm[(0, base)] = nx;
        bm[(1, base + 1)] = ny;
        bm[(2, base)] = ny;
        bm[(2, base + 1)] = nx;

        // Mindlin-Reissner bending curvature [d(ry)/dx, -d(rx)/dy, d(ry)/dy - d(rx)/dx]
        bb[(0, base + 4)] = nx;
        bb[(1, base + 3)] = -ny;
        bb[(2, base + 3)] = -nx;
        bb[(2, base + 4)] = ny;

        // Mindlin-Reissner transverse shear [dw/dx + ry, dw/dy - rx]
        bs[(0, base + 2)] = nx;
        bs[(0, base + 4)] = na;
        bs[(1, base + 2)] = ny;
        bs[(1, base + 3)] = -na;
    }
}
