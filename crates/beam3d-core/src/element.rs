use crate::model::{BeamFormulation, GravitySettings, LoadCoordSystem};
use crate::section::SectionProperties;
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};
use serde::{Deserialize, Serialize};

/// Orthonormal local coordinate frame for a 3D beam element.
#[derive(Debug, Clone, Copy)]
pub struct ElementFrame {
    pub length: f64,
    /// 3x3 rotation matrix mapping global -> local (`v_local = r_mat * v_global`).
    /// Rows are `ex`, `ey`, `ez` expressed in global coordinates.
    pub r_mat: Matrix3<f64>,
}

impl ElementFrame {
    pub fn compute(p_i: [f64; 3], p_j: [f64; 3], roll_angle_deg: f64) -> Result<Self, String> {
        let vi = Vector3::from(p_i);
        let vj = Vector3::from(p_j);
        let delta = vj - vi;
        let length = delta.norm();
        if length < 1e-9 {
            return Err("Zero-length beam element detected (nodes coincide).".to_string());
        }
        let ex = delta / length;

        // Reference orientation: global Z is vertical up.
        // For non-vertical members: ey0 = normalize(Z x ex), ez0 = ex x ey0 (ez0 points upward).
        // For vertical members (|ex . Z| > 0.9999): ey0 = normalize([-1, 0, 0] x ex) -> +Y when ex = +Z.
        let global_z = Vector3::new(0.0, 0.0, 1.0);
        let ey0 = if ex.dot(&global_z).abs() < 0.9999 {
            global_z.cross(&ex).normalize()
        } else {
            let neg_x = Vector3::new(-1.0, 0.0, 0.0);
            neg_x.cross(&ex).normalize()
        };
        let ez0 = ex.cross(&ey0).normalize();

        let gamma = roll_angle_deg.to_radians();
        let (cg, sg) = (gamma.cos(), gamma.sin());
        let ey = cg * ey0 + sg * ez0;
        let ez = -sg * ey0 + cg * ez0;

        let r_mat = Matrix3::from_rows(&[ex.transpose(), ey.transpose(), ez.transpose()]);
        Ok(Self { length, r_mat })
    }

    pub fn ex(&self) -> [f64; 3] {
        [self.r_mat[(0, 0)], self.r_mat[(0, 1)], self.r_mat[(0, 2)]]
    }

    pub fn ey(&self) -> [f64; 3] {
        [self.r_mat[(1, 0)], self.r_mat[(1, 1)], self.r_mat[(1, 2)]]
    }

    pub fn ez(&self) -> [f64; 3] {
        [self.r_mat[(2, 0)], self.r_mat[(2, 1)], self.r_mat[(2, 2)]]
    }

    pub fn transformation_12x12(&self) -> SMatrix<f64, 12, 12> {
        let mut t = SMatrix::<f64, 12, 12>::zeros();
        for block in 0..4 {
            let idx = block * 3;
            t.fixed_view_mut::<3, 3>(idx, idx).copy_from(&self.r_mat);
        }
        t
    }
}

/// Uncondensed 12x12 local stiffness matrix (supports both Timoshenko and Euler-Bernoulli).
/// Local DOF order: `[ux1, uy1, uz1, rx1, ry1, rz1, ux2, uy2, uz2, rx2, ry2, rz2]`.
pub fn uncondensed_local_stiffness(
    e: f64,
    g: f64,
    props: &SectionProperties,
    length: f64,
    formulation: BeamFormulation,
) -> SMatrix<f64, 12, 12> {
    let l = length;
    let l2 = l * l;
    let l3 = l2 * l;

    let (phi_y, phi_z) = match formulation {
        BeamFormulation::EulerBernoulli => (0.0, 0.0),
        BeamFormulation::Timoshenko => {
            let py = if g * props.asy > 1e-12 {
                (12.0 * e * props.iz) / (g * props.asy * l2)
            } else {
                0.0
            };
            let pz = if g * props.asz > 1e-12 {
                (12.0 * e * props.iy) / (g * props.asz * l2)
            } else {
                0.0
            };
            (py, pz)
        }
    };

    let mut k = SMatrix::<f64, 12, 12>::zeros();

    // 1. Axial stiffness (ux1=0, ux2=6)
    let ka = e * props.area / l;
    k[(0, 0)] = ka;
    k[(0, 6)] = -ka;
    k[(6, 0)] = -ka;
    k[(6, 6)] = ka;

    // 2. St. Venant torsional stiffness (rx1=3, rx2=9)
    let kt = g * props.j / l;
    k[(3, 3)] = kt;
    k[(3, 9)] = -kt;
    k[(9, 3)] = -kt;
    k[(9, 9)] = kt;

    // 3. Bending in local x-y plane (uy1=1, rz1=5, uy2=7, rz2=11), inertia Iz, shear area Asy
    let cy = e * props.iz / (l3 * (1.0 + phi_y));
    let k_yy = 12.0 * cy;
    let k_yr = 6.0 * l * cy;
    let k_rr1 = (4.0 + phi_y) * l2 * cy;
    let k_rr2 = (2.0 - phi_y) * l2 * cy;

    k[(1, 1)] = k_yy;
    k[(1, 5)] = k_yr;
    k[(1, 7)] = -k_yy;
    k[(1, 11)] = k_yr;

    k[(5, 1)] = k_yr;
    k[(5, 5)] = k_rr1;
    k[(5, 7)] = -k_yr;
    k[(5, 11)] = k_rr2;

    k[(7, 1)] = -k_yy;
    k[(7, 5)] = -k_yr;
    k[(7, 7)] = k_yy;
    k[(7, 11)] = -k_yr;

    k[(11, 1)] = k_yr;
    k[(11, 5)] = k_rr2;
    k[(11, 7)] = -k_yr;
    k[(11, 11)] = k_rr1;

    // 4. Bending in local x-z plane (uz1=2, ry1=4, uz2=8, ry2=10), inertia Iy, shear area Asz
    // Note right-hand rule: theta_y = -dw/dx, so displacement-rotation off-diagonal signs flip!
    let cz = e * props.iy / (l3 * (1.0 + phi_z));
    let k_zz = 12.0 * cz;
    let k_zr = 6.0 * l * cz;
    let k_ry1 = (4.0 + phi_z) * l2 * cz;
    let k_ry2 = (2.0 - phi_z) * l2 * cz;

    k[(2, 2)] = k_zz;
    k[(2, 4)] = -k_zr;
    k[(2, 8)] = -k_zz;
    k[(2, 10)] = -k_zr;

    k[(4, 2)] = -k_zr;
    k[(4, 4)] = k_ry1;
    k[(4, 8)] = k_zr;
    k[(4, 10)] = k_ry2;

    k[(8, 2)] = -k_zz;
    k[(8, 4)] = k_zr;
    k[(8, 8)] = k_zz;
    k[(8, 10)] = k_zr;

    k[(10, 2)] = -k_zr;
    k[(10, 4)] = k_ry2;
    k[(10, 8)] = k_zr;
    k[(10, 10)] = k_ry1;

    k
}

/// Consistent equivalent nodal load vector `f_eq_local` (12x1) for a linearly varying
/// local distributed load `q(xi) = (1 - xi) * q_start + xi * q_end` (in N/m).
/// Note: Fixed-end force vector `f_fef_local = -f_eq_local`.
pub fn consistent_equivalent_local_loads(
    q_local_start: [f64; 3],
    q_local_end: [f64; 3],
    e: f64,
    g: f64,
    props: &SectionProperties,
    length: f64,
    formulation: BeamFormulation,
) -> SVector<f64, 12> {
    let l = length;
    let l2 = l * l;

    let (phi_y, phi_z) = match formulation {
        BeamFormulation::EulerBernoulli => (0.0, 0.0),
        BeamFormulation::Timoshenko => {
            let py = if g * props.asy > 1e-12 {
                (12.0 * e * props.iz) / (g * props.asy * l2)
            } else {
                0.0
            };
            let pz = if g * props.asz > 1e-12 {
                (12.0 * e * props.iy) / (g * props.asz * l2)
            } else {
                0.0
            };
            (py, pz)
        }
    };

    // 3-point Gauss-Legendre quadrature on [0, 1] integrates polynomials up to degree 5 EXACTLY.
    // Our integrand N_i(xi) * q(xi) has degree 3 + 1 = 4 <= 5.
    let s = (0.6_f64).sqrt();
    let gp_xi = [0.5 * (1.0 - s), 0.5, 0.5 * (1.0 + s)];
    let gp_w = [5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0];

    let mut feq = SVector::<f64, 12>::zeros();

    for (&xi, &w) in gp_xi.iter().zip(gp_w.iter()) {
        let qx = (1.0 - xi) * q_local_start[0] + xi * q_local_end[0];
        let qy = (1.0 - xi) * q_local_start[1] + xi * q_local_end[1];
        let qz = (1.0 - xi) * q_local_start[2] + xi * q_local_end[2];
        let ds = w * l;

        // Linear axial shape functions
        feq[0] += (1.0 - xi) * qx * ds;
        feq[6] += xi * qx * ds;

        // Exact Timoshenko / Euler-Bernoulli shape functions in x-y plane
        let [nv1_y, nth1_y, nv2_y, nth2_y] = timoshenko_shape_fns(xi, l, phi_y);
        feq[1] += nv1_y * qy * ds;
        feq[5] += nth1_y * qy * ds;
        feq[7] += nv2_y * qy * ds;
        feq[11] += nth2_y * qy * ds;

        // Exact Timoshenko / Euler-Bernoulli shape functions in x-z plane (note theta_y = -dw/dx)
        let [nv1_z, nth1_z, nv2_z, nth2_z] = timoshenko_shape_fns(xi, l, phi_z);
        feq[2] += nv1_z * qz * ds;
        feq[4] += -nth1_z * qz * ds;
        feq[8] += nv2_z * qz * ds;
        feq[10] += -nth2_z * qz * ds;
    }

    feq
}

#[inline]
fn timoshenko_shape_fns(xi: f64, l: f64, phi: f64) -> [f64; 4] {
    let xi2 = xi * xi;
    let xi3 = xi2 * xi;
    let denom = 1.0 + phi;
    let nv1 = (1.0 - 3.0 * xi2 + 2.0 * xi3 + phi * (1.0 - xi)) / denom;
    let nth1 = l * (xi - 2.0 * xi2 + xi3 + 0.5 * phi * (xi - xi2)) / denom;
    let nv2 = (3.0 * xi2 - 2.0 * xi3 + phi * xi) / denom;
    let nth2 = l * (-xi2 + xi3 - 0.5 * phi * (xi - xi2)) / denom;
    [nv1, nth1, nv2, nth2]
}

/// Aggregates all distributed loads and self-weight on an element into `(q_local_start, q_local_end)`.
pub fn total_local_distributed_load(
    frame: &ElementFrame,
    density: f64,
    area: f64,
    gravity: &GravitySettings,
    dloads: &[(LoadCoordSystem, [f64; 3], [f64; 3])],
) -> ([f64; 3], [f64; 3]) {
    let mut q0 = Vector3::zeros();
    let mut q1 = Vector3::zeros();

    if gravity.enabled {
        let g_global = Vector3::from(gravity.acceleration) * (density * area);
        let g_local = frame.r_mat * g_global;
        q0 += g_local;
        q1 += g_local;
    }

    for &(cs, qs, qe) in dloads {
        let vs = Vector3::from(qs);
        let ve = Vector3::from(qe);
        match cs {
            LoadCoordSystem::Local => {
                q0 += vs;
                q1 += ve;
            }
            LoadCoordSystem::Global => {
                q0 += frame.r_mat * vs;
                q1 += frame.r_mat * ve;
            }
        }
    }

    ([q0[0], q0[1], q0[2]], [q1[0], q1[1], q1[2]])
}

/// Applies static condensation to release specified local member end DOFs (`releases_i` + `releases_j`).
/// Input: uncondensed `k_local` and `f_fef_local` (`f_int = k * u + f_fef`).
/// Output: condensed `(k_cond, f_fef_cond)`.
pub fn condense_element_releases(
    k_local: &SMatrix<f64, 12, 12>,
    f_fef_local: &SVector<f64, 12>,
    releases: &[bool; 12],
) -> (SMatrix<f64, 12, 12>, SVector<f64, 12>) {
    let condensed: Vec<usize> = (0..12).filter(|&i| releases[i]).collect();
    if condensed.is_empty() {
        return (*k_local, *f_fef_local);
    }
    let retained: Vec<usize> = (0..12).filter(|&i| !releases[i]).collect();
    if retained.is_empty() {
        return (SMatrix::zeros(), SVector::zeros());
    }

    let nr = retained.len();
    let nc = condensed.len();

    let mut k_rr = DMatrix::<f64>::zeros(nr, nr);
    let mut k_rc = DMatrix::<f64>::zeros(nr, nc);
    let mut k_cr = DMatrix::<f64>::zeros(nc, nr);
    let mut k_cc = DMatrix::<f64>::zeros(nc, nc);
    let mut f_r = DVector::<f64>::zeros(nr);
    let mut f_c = DVector::<f64>::zeros(nc);

    for (ir, &r) in retained.iter().enumerate() {
        f_r[ir] = f_fef_local[r];
        for (jr, &r2) in retained.iter().enumerate() {
            k_rr[(ir, jr)] = k_local[(r, r2)];
        }
        for (jc, &c) in condensed.iter().enumerate() {
            k_rc[(ir, jc)] = k_local[(r, c)];
        }
    }
    for (ic, &c) in condensed.iter().enumerate() {
        f_c[ic] = f_fef_local[c];
        for (jr, &r) in retained.iter().enumerate() {
            k_cr[(ic, jr)] = k_local[(c, r)];
        }
        for (jc, &c2) in condensed.iter().enumerate() {
            k_cc[(ic, jc)] = k_local[(c, c2)];
        }
    }

    let k_cc_lu = solve_or_regularize(&k_cc, &k_cr, &f_c);
    let (k_cc_inv_k_cr, k_cc_inv_f_c) = k_cc_lu;

    let k_tilde_rr = k_rr - &k_rc * k_cc_inv_k_cr;
    let f_tilde_r = f_r - &k_rc * k_cc_inv_f_c;

    let mut k_out = SMatrix::<f64, 12, 12>::zeros();
    let mut f_out = SVector::<f64, 12>::zeros();

    for (ir, &r) in retained.iter().enumerate() {
        f_out[r] = f_tilde_r[ir];
        for (jr, &r2) in retained.iter().enumerate() {
            k_out[(r, r2)] = k_tilde_rr[(ir, jr)];
        }
    }

    (k_out, f_out)
}

/// Recovers full uncondensed local displacements `u_local` (including hinged end rotations)
/// from the nodal displacements in local coordinates.
pub fn recover_uncondensed_local_displacements(
    u_nodal_local: &SVector<f64, 12>,
    k_local: &SMatrix<f64, 12, 12>,
    f_fef_local: &SVector<f64, 12>,
    releases: &[bool; 12],
) -> SVector<f64, 12> {
    let condensed: Vec<usize> = (0..12).filter(|&i| releases[i]).collect();
    if condensed.is_empty() {
        return *u_nodal_local;
    }
    let retained: Vec<usize> = (0..12).filter(|&i| !releases[i]).collect();
    let nr = retained.len();
    let nc = condensed.len();

    let mut k_cr = DMatrix::<f64>::zeros(nc, nr);
    let mut k_cc = DMatrix::<f64>::zeros(nc, nc);
    let mut u_r = DVector::<f64>::zeros(nr);
    let mut f_c = DVector::<f64>::zeros(nc);

    for (ir, &r) in retained.iter().enumerate() {
        u_r[ir] = u_nodal_local[r];
    }
    for (ic, &c) in condensed.iter().enumerate() {
        f_c[ic] = f_fef_local[c];
        for (jr, &r) in retained.iter().enumerate() {
            k_cr[(ic, jr)] = k_local[(c, r)];
        }
        for (jc, &c2) in condensed.iter().enumerate() {
            k_cc[(ic, jc)] = k_local[(c, c2)];
        }
    }

    let rhs = -(&k_cr * u_r + f_c);
    let u_c = if let Some(sol) = k_cc.clone().lu().solve(&rhs) {
        sol
    } else {
        let scale = k_cc
            .diagonal()
            .iter()
            .fold(0.0_f64, |acc, &v| acc.max(v.abs()))
            .max(1.0);
        let mut reg = k_cc;
        for i in 0..nc {
            reg[(i, i)] += 1e-12 * scale;
        }
        reg.lu().solve(&rhs).unwrap_or_else(|| DVector::zeros(nc))
    };

    let mut out = *u_nodal_local;
    for (ic, &c) in condensed.iter().enumerate() {
        out[c] = u_c[ic];
    }
    out
}

fn solve_or_regularize(
    k_cc: &DMatrix<f64>,
    k_cr: &DMatrix<f64>,
    f_c: &DVector<f64>,
) -> (DMatrix<f64>, DVector<f64>) {
    let lu = k_cc.clone().lu();
    if let (Some(m), Some(v)) = (lu.solve(k_cr), lu.solve(f_c)) {
        return (m, v);
    }
    let nc = k_cc.nrows();
    let scale = k_cc
        .diagonal()
        .iter()
        .fold(0.0_f64, |acc, &v| acc.max(v.abs()))
        .max(1.0);
    let mut reg = k_cc.clone();
    for i in 0..nc {
        reg[(i, i)] += 1e-12 * scale;
    }
    let reg_lu = reg.lu();
    let m = reg_lu
        .solve(k_cr)
        .unwrap_or_else(|| DMatrix::zeros(nc, k_cr.ncols()));
    let v = reg_lu.solve(f_c).unwrap_or_else(|| DVector::zeros(nc));
    (m, v)
}

/// Sampled post-processing values at a station `x = xi * L` along a beam element.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StationSample {
    /// Normalized coordinate `xi in [0, 1]`.
    pub xi: f64,
    /// Distance `x in [0, L]` (m).
    pub x: f64,
    /// Undeformed global position `[X, Y, Z]` (m).
    pub pos_undeformed: [f64; 3],
    /// Local displacement `[ux, uy, uz]` (m).
    pub disp_local: [f64; 3],
    /// Global displacement `[ux, uy, uz]` (m).
    pub disp_global: [f64; 3],
    /// Local section rotation `[rx, ry, rz]` (rad).
    pub rot_local: [f64; 3],
    /// Axial force `N` (N, tension > 0, compression < 0).
    pub axial_n: f64,
    /// Shear force `Vy` along local y (N).
    pub shear_vy: f64,
    /// Shear force `Vz` along local z (N).
    pub shear_vz: f64,
    /// St. Venant torque `T` about local x (N·m).
    pub torque_t: f64,
    /// Strong-axis bending moment `My` in local x-z plane (N·m, sagging > 0).
    pub moment_my: f64,
    /// Weak-axis bending moment `Mz` in local x-y plane (N·m).
    pub moment_mz: f64,
    /// Maximum extreme-fiber combined axial + biaxial bending normal stress `|N|/A + |My|*cz/Iy + |Mz|*cy/Iz` (Pa).
    pub max_normal_stress: f64,
    /// Approximate combined von Mises stress including average shear & torsion (Pa).
    pub von_mises_stress: f64,
}

/// Evaluates exact internal forces, stresses, and deflected shape at `n_stations` along the element.
#[allow(clippy::too_many_arguments)]
pub fn sample_element_stations(
    p_i: [f64; 3],
    frame: &ElementFrame,
    e: f64,
    g: f64,
    props: &SectionProperties,
    formulation: BeamFormulation,
    u_local: &SVector<f64, 12>,
    f_int_local: &SVector<f64, 12>,
    q_local_start: [f64; 3],
    q_local_end: [f64; 3],
    n_stations: usize,
) -> Vec<StationSample> {
    let n_pts = n_stations.max(2);
    let l = frame.length;
    let r_t = frame.r_mat.transpose();
    let pi = Vector3::from(p_i);
    let ex_global = Vector3::from(frame.ex());

    let fx1 = f_int_local[0];
    let fy1 = f_int_local[1];
    let fz1 = f_int_local[2];
    let mx1 = f_int_local[3];
    let my1 = f_int_local[4];
    let mz1 = f_int_local[5];

    let ux1 = u_local[0];
    let uy1 = u_local[1];
    let uz1 = u_local[2];
    let rx1 = u_local[3];
    let ry1 = u_local[4];
    let rz1 = u_local[5];

    let [qx0, qy0, qz0] = q_local_start;
    let [qx1, qy1, qz1] = q_local_end;
    let dqx = qx1 - qx0;
    let dqy = qy1 - qy0;
    let dqz = qz1 - qz0;

    let inv_ea = 1.0 / (e * props.area).max(1e-18);
    let inv_gj = 1.0 / (g * props.j).max(1e-18);
    let inv_eiz = 1.0 / (e * props.iz).max(1e-18);
    let inv_eiy = 1.0 / (e * props.iy).max(1e-18);

    let (inv_gasy, inv_gasz) = match formulation {
        BeamFormulation::EulerBernoulli => (0.0, 0.0),
        BeamFormulation::Timoshenko => (
            1.0 / (g * props.asy).max(1e-18),
            1.0 / (g * props.asz).max(1e-18),
        ),
    };

    let mut samples = Vec::with_capacity(n_pts);

    for k in 0..n_pts {
        let xi = k as f64 / (n_pts - 1) as f64;
        let x = xi * l;
        let x2 = x * x;
        let x3 = x2 * x;
        let x4 = x2 * x2;
        let x5 = x4 * x;

        // First integrals of q(s) from 0 to x
        let i1_x = qx0 * x + 0.5 * (dqx / l) * x2;
        let i1_y = qy0 * x + 0.5 * (dqy / l) * x2;
        let i1_z = qz0 * x + 0.5 * (dqz / l) * x2;

        // Second integrals int_0^x (x - s) q(s) ds
        let i2_y = 0.5 * qy0 * x2 + (dqy / (6.0 * l)) * x3;
        let i2_z = 0.5 * qz0 * x2 + (dqz / (6.0 * l)) * x3;

        // Exact internal section forces
        let axial_n = -fx1 - i1_x;
        let torque_t = -mx1;
        let shear_vy = fy1 + i1_y;
        let shear_vz = fz1 + i1_z;
        let moment_mz = -mz1 + fy1 * x + i2_y;
        let moment_my = my1 + fz1 * x + i2_z;

        // Exact axial displacement ux(x) = ux1 + int_0^x N(s)/(EA) ds
        let int_n = -fx1 * x - 0.5 * qx0 * x2 - (dqx / (6.0 * l)) * x3;
        let ux = ux1 + int_n * inv_ea;

        // Exact twist rotation rx(x) = rx1 + T/(GJ) * x
        let rx = rx1 + torque_t * x * inv_gj;

        // Exact rotation & deflection in x-y plane:
        // d(rz)/dx = Mz(x)/(E*Iz), d(uy)/dx = rz(x) - Vy(x)/(G*Asy)
        let int_mz = -mz1 * x + 0.5 * fy1 * x2 + (qy0 / 6.0) * x3 + (dqy / (24.0 * l)) * x4;
        let rz = rz1 + int_mz * inv_eiz;

        let int2_mz =
            -0.5 * mz1 * x2 + (fy1 / 6.0) * x3 + (qy0 / 24.0) * x4 + (dqy / (120.0 * l)) * x5;
        let int_vy = fy1 * x + 0.5 * qy0 * x2 + (dqy / (6.0 * l)) * x3;
        let uy = uy1 + rz1 * x + int2_mz * inv_eiz - int_vy * inv_gasy;

        // Exact rotation & deflection in x-z plane:
        // d(ry)/dx = -My(x)/(E*Iy), d(uz)/dx = -ry(x) - Vz(x)/(G*Asz)
        let int_my = my1 * x + 0.5 * fz1 * x2 + (qz0 / 6.0) * x3 + (dqz / (24.0 * l)) * x4;
        let ry = ry1 - int_my * inv_eiy;

        let int2_my =
            0.5 * my1 * x2 + (fz1 / 6.0) * x3 + (qz0 / 24.0) * x4 + (dqz / (120.0 * l)) * x5;
        let int_vz = fz1 * x + 0.5 * qz0 * x2 + (dqz / (6.0 * l)) * x3;
        let uz = uz1 - ry1 * x + int2_my * inv_eiy - int_vz * inv_gasz;

        let disp_local_vec = Vector3::new(ux, uy, uz);
        let disp_global_vec = r_t * disp_local_vec;
        let pos_u = pi + ex_global * x;

        // Stress recovery
        let sigma_axial = axial_n.abs() / props.area;
        let sigma_my = moment_my.abs() * props.half_depth_z / props.iy;
        let sigma_mz = moment_mz.abs() * props.half_width_y / props.iz;
        let max_normal_stress = sigma_axial + sigma_my + sigma_mz;

        let tau_vy = shear_vy.abs() / props.asy;
        let tau_vz = shear_vz.abs() / props.asz;
        let r_eff = props.half_depth_z.max(props.half_width_y);
        let tau_torsion = torque_t.abs() * r_eff / props.j;
        let tau_total = (tau_vy * tau_vy + tau_vz * tau_vz).sqrt() + tau_torsion;
        let von_mises_stress =
            (max_normal_stress * max_normal_stress + 3.0 * tau_total * tau_total).sqrt();

        samples.push(StationSample {
            xi,
            x,
            pos_undeformed: [pos_u[0], pos_u[1], pos_u[2]],
            disp_local: [ux, uy, uz],
            disp_global: [disp_global_vec[0], disp_global_vec[1], disp_global_vec[2]],
            rot_local: [rx, ry, rz],
            axial_n,
            shear_vy,
            shear_vz,
            torque_t,
            moment_my,
            moment_mz,
            max_normal_stress,
            von_mises_stress,
        });
    }

    samples
}
