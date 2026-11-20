use crate::element::beam_local_mass_matrix;
use crate::model::StructuralModel;
use crate::nonlinear::{
    PreparedModel, apply_increment, assemble_corotational_system, exp_so3, prepare_model,
    recover_corotational_solution, recover_linear_solution_from_u, solve_nonlinear_static_with_config,
    solve_sparse_system, NonlinearStaticConfig,
};
use crate::shell::shell_local_mass_matrix;
use crate::solver::{Solution, SolverError};
use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, Triplet};
use faer::{Mat, Side};
use nalgebra::{DMatrix, DVector, Matrix3, SymmetricEigen, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Instant;

/// Mass matrix discretization scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MassFormulation {
    Consistent,
    Lumped,
}

impl MassFormulation {
    pub fn label(self) -> &'static str {
        match self {
            MassFormulation::Consistent => "Consistent Mass",
            MassFormulation::Lumped => "Lumped (Diagonal) Mass",
        }
    }
}

/// Rayleigh proportional viscous damping `C = alpha * M + beta * K`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum RayleighDamping {
    None,
    /// Target modal damping ratio `zeta` (e.g. 0.02 = 2%) calibrated automatically from modes 1 & 2.
    ModalRatio { zeta: f64 },
    /// Direct Rayleigh coefficients `alpha` (1/s) and `beta` (s).
    Direct { alpha: f64, beta: f64 },
}

/// Configuration for natural frequency / modal analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalConfig {
    pub num_modes: usize,
    pub mass_formulation: MassFormulation,
    pub stations_per_element: usize,
}

impl Default for ModalConfig {
    fn default() -> Self {
        Self {
            num_modes: 6,
            mass_formulation: MassFormulation::Consistent,
            stations_per_element: 17,
        }
    }
}

/// Single vibration mode shape and modal participation metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModeShape {
    /// 1-based mode number.
    pub mode_index: usize,
    /// Natural frequency `f` in Hz.
    pub frequency_hz: f64,
    /// Circular frequency `omega = 2 * pi * f` in rad/s.
    pub omega_rad_s: f64,
    /// Natural period `T = 1 / f` in s.
    pub period_s: f64,
    /// Effective modal mass in global X, Y, Z directions (kg).
    pub effective_mass_kg: [f64; 3],
    /// Effective modal mass ratio `[mx/M, my/M, mz/M]` in `[0, 1]`.
    pub effective_mass_ratio: [f64; 3],
    /// Unit-normalized (`max |u_trans| = 1 m`) mode shape packaged as a full `Solution` for 3D rendering.
    pub solution: Solution,
}

/// Complete output of a modal / eigenvalue analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalSolution {
    pub modes: Vec<ModeShape>,
    pub total_mass_kg: f64,
    pub free_dofs: usize,
    pub solve_time_ms: f64,
}

/// Time-varying loading / ground motion excitation for transient dynamic analysis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TimeHistoryExcitation {
    /// Instantaneous Heaviside step application of all model loads (`f(t) = 1`).
    Step,
    /// Rectangular pulse of duration `duration_s`, then free vibration.
    Impulse { duration_s: f64 },
    /// Smooth half-sine pulse `sin(pi * t / duration_s)` for `t <= duration_s`, then free vibration.
    HalfSinePulse { duration_s: f64 },
    /// Continuous harmonic excitation `sin(2 * pi * frequency_hz * t)`.
    Harmonic { frequency_hz: f64 },
    /// Linear ramp from 0 to 1 over `rise_time_s`, then held constant at 1.
    Ramp { rise_time_s: f64 },
    /// Pluck-and-release: starts from static equilibrium deflection and vibrates freely with zero load.
    FreeVibration,
    /// Sinusoidal seismic base acceleration in global `direction` (amplitude in g's).
    BaseSine {
        direction: [f64; 3],
        amplitude_g: f64,
        frequency_hz: f64,
        duration_s: f64,
    },
}

impl TimeHistoryExcitation {
    pub fn label(self) -> &'static str {
        match self {
            TimeHistoryExcitation::Step => "Step Load (Heaviside)",
            TimeHistoryExcitation::Impulse { .. } => "Rectangular Impulse",
            TimeHistoryExcitation::HalfSinePulse { .. } => "Half-Sine Pulse",
            TimeHistoryExcitation::Harmonic { .. } => "Harmonic Sine Force",
            TimeHistoryExcitation::Ramp { .. } => "Linear Ramp Load",
            TimeHistoryExcitation::FreeVibration => "Free Vibration (Pluck & Release)",
            TimeHistoryExcitation::BaseSine { .. } => "Seismic Base Sine Wave",
        }
    }

    pub fn load_factor(&self, t: f64) -> f64 {
        match *self {
            TimeHistoryExcitation::Step => 1.0,
            TimeHistoryExcitation::Impulse { duration_s } => {
                if t <= duration_s.max(0.0) {
                    1.0
                } else {
                    0.0
                }
            }
            TimeHistoryExcitation::HalfSinePulse { duration_s } => {
                let td = duration_s.max(1e-9);
                if t >= 0.0 && t <= td {
                    (PI * t / td).sin()
                } else {
                    0.0
                }
            }
            TimeHistoryExcitation::Harmonic { frequency_hz } => (2.0 * PI * frequency_hz * t).sin(),
            TimeHistoryExcitation::Ramp { rise_time_s } => {
                let tr = rise_time_s.max(1e-9);
                (t / tr).clamp(0.0, 1.0)
            }
            TimeHistoryExcitation::FreeVibration => 0.0,
            TimeHistoryExcitation::BaseSine { .. } => 0.0,
        }
    }

    pub fn base_accel_global(&self, t: f64) -> [f64; 3] {
        match *self {
            TimeHistoryExcitation::BaseSine {
                direction,
                amplitude_g,
                frequency_hz,
                duration_s,
            } => {
                if t > duration_s.max(0.0) {
                    return [0.0; 3];
                }
                let norm = (direction[0].powi(2) + direction[1].powi(2) + direction[2].powi(2))
                    .sqrt()
                    .max(1e-12);
                let a = amplitude_g * 9.80665 * (2.0 * PI * frequency_hz * t).sin();
                [
                    a * direction[0] / norm,
                    a * direction[1] / norm,
                    a * direction[2] / norm,
                ]
            }
            _ => [0.0; 3],
        }
    }
}

/// Configuration for Newmark-beta transient dynamic time-history analysis (linear or nonlinear).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicConfig {
    /// Total simulation duration in seconds.
    pub duration_s: f64,
    /// Time step `dt` in seconds.
    pub dt_s: f64,
    /// Newmark `beta` parameter (0.25 = average acceleration, unconditionally stable).
    pub newmark_beta: f64,
    /// Newmark `gamma` parameter (0.5 = zero numerical damping).
    pub newmark_gamma: f64,
    pub mass_formulation: MassFormulation,
    pub damping: RayleighDamping,
    pub excitation: TimeHistoryExcitation,
    /// If true, uses 3D corotational large-deformation Newton-Raphson at every time step.
    pub nonlinear: bool,
    pub max_newton_iters: usize,
    pub newton_tol: f64,
    /// Maximum number of full 3D `Solution` snapshots retained for viewport playback.
    pub max_stored_frames: usize,
    pub stations_per_element: usize,
}

impl Default for DynamicConfig {
    fn default() -> Self {
        Self {
            duration_s: 1.5,
            dt_s: 0.01,
            newmark_beta: 0.25,
            newmark_gamma: 0.5,
            mass_formulation: MassFormulation::Consistent,
            damping: RayleighDamping::ModalRatio { zeta: 0.02 },
            excitation: TimeHistoryExcitation::Step,
            nonlinear: false,
            max_newton_iters: 15,
            newton_tol: 1e-5,
            max_stored_frames: 100,
            stations_per_element: 13,
        }
    }
}

/// Lightweight scalar time-history record stored at EVERY integration step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicHistoryPoint {
    pub time_s: f64,
    pub max_displacement_m: f64,
    pub kinetic_energy_j: f64,
    pub strain_energy_j: f64,
    /// Nodal translational displacements `[ux, uy, uz]` indexed by `node_id` for plotting any selected node.
    pub node_translations: HashMap<usize, [f64; 3]>,
}

/// Full 3D snapshot at a sampled instant in time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicFrame {
    pub time_s: f64,
    pub max_displacement_m: f64,
    pub kinetic_energy_j: f64,
    pub strain_energy_j: f64,
    pub solution: Solution,
}

/// Complete output of a transient dynamic time-history simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicSolution {
    pub frames: Vec<DynamicFrame>,
    pub history: Vec<DynamicHistoryPoint>,
    pub peak_displacement_m: f64,
    pub peak_displacement_time_s: f64,
    pub peak_stress_pa: f64,
    pub rayleigh_alpha: f64,
    pub rayleigh_beta: f64,
    pub time_steps: usize,
    pub nonlinear: bool,
    pub solve_time_ms: f64,
}

/// Assembles sparse triplets for linear `K_ff`, `M_ff`, and total structural mass (kg).
fn assemble_linear_k_and_m(
    model: &StructuralModel,
    prep: &PreparedModel,
    mass_formulation: MassFormulation,
) -> Result<(Vec<Triplet<usize, usize, f64>>, Vec<Triplet<usize, usize, f64>>, f64), SolverError> {
    let lumped = matches!(mass_formulation, MassFormulation::Lumped);
    let mut k_triplets = Vec::with_capacity(prep.beams.len() * 144 + prep.shells.len() * 576);
    let mut m_triplets = Vec::with_capacity(prep.beams.len() * 144 + prep.shells.len() * 576);
    let mut total_mass_kg = 0.0_f64;

    for beam in &prep.beams {
        let t_mat = beam.frame0.transformation_12x12();
        let t_tr = t_mat.transpose();
        let k_glob = t_tr * beam.k_cond_local * t_mat;
        let m_loc = beam_local_mass_matrix(beam.density, &beam.props, beam.frame0.length, lumped);
        let m_glob = t_tr * m_loc * t_mat;
        total_mass_kg += beam.density.max(0.0) * beam.props.area * beam.frame0.length;

        let mut gdofs = [0usize; 12];
        for d in 0..6 {
            gdofs[d] = beam.i_idx * 6 + d;
            gdofs[6 + d] = beam.j_idx * 6 + d;
        }
        for r in 0..12 {
            if let Some(fr) = prep.dof_to_free[gdofs[r]] {
                for c in 0..12 {
                    if let Some(fc) = prep.dof_to_free[gdofs[c]] {
                        let kv = k_glob[(r, c)];
                        if kv.abs() != 0.0 {
                            k_triplets.push(Triplet::new(fr, fc, kv));
                        }
                        let mv = m_glob[(r, c)];
                        if mv.abs() != 0.0 {
                            m_triplets.push(Triplet::new(fr, fc, mv));
                        }
                    }
                }
            }
        }
    }

    for shell in &prep.shells {
        let t_mat = shell.frame0.transformation_matrix();
        let t_tr = t_mat.transpose();
        let k_glob = &t_tr * &shell.k_local0 * &t_mat;
        let m_loc = shell_local_mass_matrix(&shell.frame0, shell.density, shell.thickness, lumped)
            .map_err(|reason| SolverError::InvalidElement {
                element_id: shell.id,
                reason,
            })?;
        let m_glob = &t_tr * &m_loc * &t_mat;
        total_mass_kg += shell.density.max(0.0) * shell.thickness * shell.frame0.area;

        let ndof = shell.node_indices.len() * 6;
        let mut gdofs = Vec::with_capacity(ndof);
        for &n_idx in &shell.node_indices {
            for d in 0..6 {
                gdofs.push(n_idx * 6 + d);
            }
        }
        for r in 0..ndof {
            if let Some(fr) = prep.dof_to_free[gdofs[r]] {
                for c in 0..ndof {
                    if let Some(fc) = prep.dof_to_free[gdofs[c]] {
                        let kv = k_glob[(r, c)];
                        if kv.abs() != 0.0 {
                            k_triplets.push(Triplet::new(fr, fc, kv));
                        }
                        let mv = m_glob[(r, c)];
                        if mv.abs() != 0.0 {
                            m_triplets.push(Triplet::new(fr, fc, mv));
                        }
                    }
                }
            }
        }
    }

    // Add nodal lumped point masses (translational DOFs)
    for (n_idx, node) in model.nodes.iter().enumerate() {
        let pm = node.added_mass.max(0.0);
        if pm > 0.0 {
            total_mass_kg += pm;
            for d in 0..3 {
                if let Some(fr) = prep.dof_to_free[n_idx * 6 + d] {
                    m_triplets.push(Triplet::new(fr, fr, pm));
                }
            }
        }
    }

    Ok((k_triplets, m_triplets, total_mass_kg))
}

/// Sparse matrix-vector multiply `y = A * x`.
fn spmv(mat: &SparseColMat<usize, f64>, x: &[f64]) -> Vec<f64> {
    let n = mat.nrows();
    let x_mat = Mat::from_fn(x.len(), 1, |i, _| x[i]);
    let y_mat = mat.as_ref() * x_mat.as_ref();
    (0..n).map(|i| y_mat[(i, 0)]).collect()
}

/// Solves dense symmetric generalized eigenproblem `K phi = lambda M phi` (SPD `M`).
/// Returns sorted `(lambda, phi)` pairs with `phi^T M phi = 1`.
fn solve_dense_generalized_eigen(
    k: &DMatrix<f64>,
    m: &DMatrix<f64>,
    num_modes: usize,
) -> Result<Vec<(f64, DVector<f64>)>, SolverError> {
    let n = k.nrows();
    if n == 0 {
        return Ok(Vec::new());
    }
    // Enforce exact symmetry and tiny diagonal floor on M if needed
    let mut m_sym = 0.5 * (m + m.transpose());
    let max_m_diag = m_sym
        .diagonal()
        .iter()
        .fold(0.0_f64, |acc, &v| acc.max(v.abs()));
    if max_m_diag <= 1e-18 {
        return Err(SolverError::SingularMassMatrix);
    }
    let chol = match m_sym.clone().cholesky() {
        Some(c) => c,
        None => {
            for i in 0..n {
                m_sym[(i, i)] += 1e-12 * max_m_diag;
            }
            m_sym
                .cholesky()
                .ok_or(SolverError::SingularMassMatrix)?
        }
    };
    let l = chol.l();

    let k_sym = 0.5 * (k + k.transpose());
    // Solve L * Y = K => Y = L^{-1} K
    let y = l
        .solve_lower_triangular(&k_sym)
        .ok_or(SolverError::SingularMassMatrix)?;
    // Solve L * Z = Y^T => Z = L^{-1} K L^{-T}
    let z = l
        .solve_lower_triangular(&y.transpose())
        .ok_or(SolverError::SingularMassMatrix)?;
    let z_t = z.transpose();
    let a = 0.5 * (z + z_t);

    let eig = SymmetricEigen::new(a);
    let mut pairs: Vec<(f64, usize)> = eig
        .eigenvalues
        .iter()
        .enumerate()
        .filter(|&(_, &lam)| lam.is_finite() && lam > 1e-9)
        .map(|(idx, &lam)| (lam, idx))
        .collect();
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));

    let lt = l.transpose();
    let take = num_modes.min(pairs.len());
    let mut out = Vec::with_capacity(take);
    for &(lam, idx) in pairs.iter().take(take) {
        let v = eig.eigenvectors.column(idx).into_owned();
        let phi = lt
            .solve_upper_triangular(&v)
            .ok_or(SolverError::SingularMassMatrix)?;
        out.push((lam, phi));
    }
    Ok(out)
}

/// Bathe subspace iteration for large sparse systems `K phi = lambda M phi`.
fn solve_subspace_eigen(
    k_ff: &SparseColMat<usize, f64>,
    m_ff: &SparseColMat<usize, f64>,
    num_modes: usize,
) -> Result<Vec<(f64, DVector<f64>)>, SolverError> {
    let n = k_ff.nrows();
    let p = num_modes.min(n);
    let q = (2 * p).max(p + 8).min(n);

    // Factorize K_ff once (Cholesky with LU fallback)
    enum KFactor {
        Cholesky(faer::sparse::linalg::solvers::Llt<usize, f64>),
        Lu(faer::sparse::linalg::solvers::Lu<usize, f64>),
    }
    impl KFactor {
        fn solve_mat(&self, rhs: &Mat<f64>) -> Mat<f64> {
            match self {
                KFactor::Cholesky(c) => c.solve(rhs),
                KFactor::Lu(l) => l.solve(rhs),
            }
        }
    }
    let k_factor = match k_ff.sp_cholesky(Side::Lower) {
        Ok(c) => KFactor::Cholesky(c),
        Err(_) => {
            let lu = k_ff.sp_lu().map_err(|_| SolverError::SingularSystem)?;
            KFactor::Lu(lu)
        }
    };

    // Build deterministic starting trial vectors X (n x q)
    let mut x = Mat::<f64>::zeros(n, q);
    for col in 0..q {
        for row in 0..n {
            // Mix of Fourier modes + shifted impulses ensures rich subspace coverage
            let phase = PI * ((col + 1) as f64) * ((row + 1) as f64) / ((n + 1) as f64);
            x[(row, col)] = phase.sin();
        }
        // Normalize column
        let mut norm2 = 0.0;
        for row in 0..n {
            norm2 += x[(row, col)] * x[(row, col)];
        }
        let inv = 1.0 / norm2.sqrt().max(1e-12);
        for row in 0..n {
            x[(row, col)] *= inv;
        }
    }

    let mut prev_lams = vec![0.0_f64; p];
    let mut best_pairs: Vec<(f64, DVector<f64>)> = Vec::new();

    for iter in 0..40 {
        let y = m_ff.as_ref() * x.as_ref(); // Y = M * X
        let x_bar = k_factor.solve_mat(&y); // K * X_bar = Y
        let mx_bar = m_ff.as_ref() * x_bar.as_ref(); // M * X_bar

        // Project K* = X_bar^T * Y and M* = X_bar^T * (M * X_bar)
        let mut k_star = DMatrix::<f64>::zeros(q, q);
        let mut m_star = DMatrix::<f64>::zeros(q, q);
        for i in 0..q {
            for j in i..q {
                let mut ks = 0.0;
                let mut ms = 0.0;
                for r in 0..n {
                    let xb = x_bar[(r, i)];
                    ks += xb * y[(r, j)];
                    ms += xb * mx_bar[(r, j)];
                }
                k_star[(i, j)] = ks;
                k_star[(j, i)] = ks;
                m_star[(i, j)] = ms;
                m_star[(j, i)] = ms;
            }
        }

        let sub_pairs = solve_dense_generalized_eigen(&k_star, &m_star, q)?;
        let n_found = sub_pairs.len();
        if n_found == 0 {
            return Err(SolverError::SingularSystem);
        }

        // Update X = X_bar * Q
        let mut x_next = Mat::<f64>::zeros(n, q);
        for (col, (_, q_vec)) in sub_pairs.iter().enumerate() {
            for r in 0..n {
                let mut sum = 0.0;
                for k_idx in 0..q {
                    sum += x_bar[(r, k_idx)] * q_vec[k_idx];
                }
                x_next[(r, col)] = sum;
            }
        }
        x = x_next;

        // Check convergence of the lowest p eigenvalues
        let check_p = p.min(n_found);
        let mut max_rel_diff = 0.0_f64;
        for i in 0..check_p {
            let lam = sub_pairs[i].0;
            if iter > 0 {
                let diff = (lam - prev_lams[i]).abs() / lam.max(1e-12);
                max_rel_diff = max_rel_diff.max(diff);
            } else {
                max_rel_diff = 1.0;
            }
            prev_lams[i] = lam;
        }

        best_pairs.clear();
        for i in 0..check_p {
            let mut phi = DVector::<f64>::zeros(n);
            for r in 0..n {
                phi[r] = x[(r, i)];
            }
            best_pairs.push((sub_pairs[i].0, phi));
        }

        if iter > 1 && max_rel_diff < 1e-8 {
            break;
        }
    }

    Ok(best_pairs)
}

/// Solves natural frequencies and vibration mode shapes with default settings (6 modes, consistent mass).
pub fn solve_modal(model: &StructuralModel) -> Result<ModalSolution, SolverError> {
    solve_modal_with_config(model, &ModalConfig::default())
}

/// Solves the generalized eigenvalue problem `K phi = omega^2 M phi` for the lowest `config.num_modes` modes.
pub fn solve_modal_with_config(
    model: &StructuralModel,
    config: &ModalConfig,
) -> Result<ModalSolution, SolverError> {
    let t_start = Instant::now();
    if config.num_modes == 0 {
        return Err(SolverError::InvalidConfig(
            "num_modes must be at least 1.".to_string(),
        ));
    }

    let prep = prepare_model(model)?;
    if prep.n_free == 0 {
        return Err(SolverError::InvalidConfig(
            "Model has zero free DOFs for modal analysis.".to_string(),
        ));
    }

    let (k_triplets, m_triplets, total_mass_kg) =
        assemble_linear_k_and_m(model, &prep, config.mass_formulation)?;
    if total_mass_kg <= 1e-12 {
        return Err(SolverError::SingularMassMatrix);
    }

    let k_ff = SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &k_triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;
    let m_ff = SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &m_triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;

    let num_modes = config.num_modes.min(prep.n_free);
    let eigenpairs = if prep.n_free <= 240 {
        let mut k_dense = DMatrix::<f64>::zeros(prep.n_free, prep.n_free);
        let mut m_dense = DMatrix::<f64>::zeros(prep.n_free, prep.n_free);
        for t in &k_triplets {
            k_dense[(t.row, t.col)] += t.val;
        }
        for t in &m_triplets {
            m_dense[(t.row, t.col)] += t.val;
        }
        solve_dense_generalized_eigen(&k_dense, &m_dense, num_modes)?
    } else {
        solve_subspace_eigen(&k_ff, &m_ff, num_modes)?
    };

    // Precompute rigid-body influence vectors r_x, r_y, r_z on free DOFs
    let mut r_vecs = [
        vec![0.0_f64; prep.n_free],
        vec![0.0_f64; prep.n_free],
        vec![0.0_f64; prep.n_free],
    ];
    for n_idx in 0..prep.n_nodes {
        for d in 0..3 {
            if let Some(fr) = prep.dof_to_free[n_idx * 6 + d] {
                r_vecs[d][fr] = 1.0;
            }
        }
    }
    let mr_vecs = [
        spmv(&m_ff, &r_vecs[0]),
        spmv(&m_ff, &r_vecs[1]),
        spmv(&m_ff, &r_vecs[2]),
    ];

    let mut modes = Vec::with_capacity(eigenpairs.len());
    for (idx, (lam, phi)) in eigenpairs.into_iter().enumerate() {
        let omega = lam.sqrt();
        let freq_hz = omega / (2.0 * PI);
        let period_s = if freq_hz > 1e-12 {
            1.0 / freq_hz
        } else {
            f64::INFINITY
        };

        // Effective modal mass m_eff_d = (phi^T M r_d)^2 (since phi^T M phi = 1)
        let mut eff_mass = [0.0_f64; 3];
        let mut eff_ratio = [0.0_f64; 3];
        for d in 0..3 {
            let gamma: f64 = (0..prep.n_free).map(|i| phi[i] * mr_vecs[d][i]).sum();
            eff_mass[d] = gamma * gamma;
            eff_ratio[d] = (eff_mass[d] / total_mass_kg).clamp(0.0, 1.0);
        }

        // Build unit-normalized global mode shape (max translational displacement = 1.0 m, or max DOF = 1.0)
        let mut u_global = vec![0.0_f64; prep.total_dofs];
        for gdof in 0..prep.total_dofs {
            if let Some(fr) = prep.dof_to_free[gdof] {
                u_global[gdof] = phi[fr];
            }
        }
        let mut max_trans = 0.0_f64;
        for n_idx in 0..prep.n_nodes {
            let b = n_idx * 6;
            let mag = (u_global[b].powi(2) + u_global[b + 1].powi(2) + u_global[b + 2].powi(2)).sqrt();
            if mag > max_trans {
                max_trans = mag;
            }
        }
        let norm_scale = if max_trans > 1e-12 {
            1.0 / max_trans
        } else {
            let max_any = u_global
                .iter()
                .fold(0.0_f64, |acc, &v| acc.max(v.abs()))
                .max(1e-12);
            1.0 / max_any
        };
        for v in &mut u_global {
            *v *= norm_scale;
        }

        let sol = recover_linear_solution_from_u(
            model,
            &prep,
            &u_global,
            0.0,
            config.stations_per_element,
            0.0,
        )?;

        modes.push(ModeShape {
            mode_index: idx + 1,
            frequency_hz: freq_hz,
            omega_rad_s: omega,
            period_s,
            effective_mass_kg: eff_mass,
            effective_mass_ratio: eff_ratio,
            solution: sol,
        });
    }

    let solve_time_ms = t_start.elapsed().as_secs_f64() * 1000.0;
    Ok(ModalSolution {
        modes,
        total_mass_kg,
        free_dofs: prep.n_free,
        solve_time_ms,
    })
}

/// Computes Rayleigh damping coefficients `(alpha, beta)`.
fn resolve_rayleigh_damping(
    model: &StructuralModel,
    damping: RayleighDamping,
    mass_formulation: MassFormulation,
) -> (f64, f64) {
    match damping {
        RayleighDamping::None => (0.0, 0.0),
        RayleighDamping::Direct { alpha, beta } => (alpha.max(0.0), beta.max(0.0)),
        RayleighDamping::ModalRatio { zeta } => {
            let z = zeta.clamp(0.0, 0.99);
            if z <= 0.0 {
                return (0.0, 0.0);
            }
            let modal_cfg = ModalConfig {
                num_modes: 2,
                mass_formulation,
                stations_per_element: 2,
            };
            if let Ok(msol) = solve_modal_with_config(model, &modal_cfg) {
                if msol.modes.len() >= 2 {
                    let w1 = msol.modes[0].omega_rad_s;
                    let w2 = msol.modes[1].omega_rad_s.max(w1 * 1.01);
                    let alpha = (2.0 * z * w1 * w2) / (w1 + w2);
                    let beta = (2.0 * z) / (w1 + w2);
                    return (alpha, beta);
                } else if let Some(m1) = msol.modes.first() {
                    let w1 = m1.omega_rad_s.max(1e-6);
                    return (0.0, (2.0 * z) / w1);
                }
            }
            (0.0, 0.0)
        }
    }
}

/// Solves `M_ff * a_0 = rhs_a0` safely even when some free DOFs are massless (e.g. massless beams
/// with lumped nodal point masses, where `M_ff` is positive semi-definite). Massless DOFs get `a_0 = 0`.
fn solve_initial_acceleration(
    n_free: usize,
    m_triplets: &[Triplet<usize, usize, f64>],
    rhs_a0: &[f64],
) -> Result<Vec<f64>, SolverError> {
    if n_free == 0 {
        return Ok(Vec::new());
    }
    let mut row_max = vec![0.0_f64; n_free];
    for t in m_triplets {
        let v = t.val.abs();
        if v > row_max[t.row] {
            row_max[t.row] = v;
        }
    }
    let max_m = row_max.iter().copied().fold(0.0_f64, f64::max);
    if max_m <= 1e-18 {
        return Err(SolverError::SingularMassMatrix);
    }

    let thresh = 1e-15 * max_m;
    let mut full_to_act = vec![None; n_free];
    let mut act_to_full = Vec::with_capacity(n_free);
    for (i, &rm) in row_max.iter().enumerate() {
        if rm > thresh {
            full_to_act[i] = Some(act_to_full.len());
            act_to_full.push(i);
        }
    }

    let n_act = act_to_full.len();
    if n_act == 0 {
        return Err(SolverError::SingularMassMatrix);
    }

    if n_act == n_free {
        if let Ok(a) = solve_sparse_system(n_free, m_triplets, rhs_a0) {
            return Ok(a);
        }
    }

    let mut act_triplets = Vec::with_capacity(m_triplets.len());
    for t in m_triplets {
        if let (Some(ar), Some(ac)) = (full_to_act[t.row], full_to_act[t.col]) {
            act_triplets.push(Triplet::new(ar, ac, t.val));
        }
    }
    let rhs_act: Vec<f64> = act_to_full.iter().map(|&i| rhs_a0[i]).collect();

    let a_act = match solve_sparse_system(n_act, &act_triplets, &rhs_act) {
        Ok(a) => a,
        Err(_) => {
            // Tiny diagonal regularization fallback for semi-definite mass blocks
            let reg = 1e-12 * max_m;
            for i in 0..n_act {
                act_triplets.push(Triplet::new(i, i, reg));
            }
            solve_sparse_system(n_act, &act_triplets, &rhs_act)?
        }
    };

    let mut a_full = vec![0.0_f64; n_free];
    for (ai, &fi) in act_to_full.iter().enumerate() {
        a_full[fi] = a_act[ai];
    }
    Ok(a_full)
}

/// Solves a transient dynamic time-history analysis with default settings.
pub fn solve_dynamic(model: &StructuralModel) -> Result<DynamicSolution, SolverError> {
    solve_dynamic_with_config(model, &DynamicConfig::default())
}

/// Solves a transient dynamic time-history analysis using Newmark-beta time integration.
/// Supports both fast linear dynamics and full 3D corotational large-deformation nonlinear dynamics.
pub fn solve_dynamic_with_config(
    model: &StructuralModel,
    config: &DynamicConfig,
) -> Result<DynamicSolution, SolverError> {
    let t_start = Instant::now();
    if config.duration_s <= 0.0 || config.dt_s <= 0.0 {
        return Err(SolverError::InvalidConfig(
            "duration_s and dt_s must be positive.".to_string(),
        ));
    }
    if config.newmark_beta <= 0.0 || config.newmark_gamma <= 0.0 {
        return Err(SolverError::InvalidConfig(
            "Newmark beta and gamma must be positive.".to_string(),
        ));
    }

    let n_steps = (config.duration_s / config.dt_s).round().max(1.0) as usize;
    let dt = config.dt_s;
    let beta = config.newmark_beta;
    let gamma = config.newmark_gamma;

    let a0 = 1.0 / (beta * dt * dt);
    let a1 = gamma / (beta * dt);
    let a2 = 1.0 / (beta * dt);
    let a3 = 1.0 / (2.0 * beta) - 1.0;
    let a4 = gamma / beta - 1.0;
    let a5 = 0.5 * dt * (gamma / beta - 2.0);
    let a6 = dt * (1.0 - gamma);
    let a7 = gamma * dt;

    let prep = prepare_model(model)?;
    if prep.n_free == 0 {
        return Err(SolverError::InvalidConfig(
            "Model has zero free DOFs for dynamic analysis.".to_string(),
        ));
    }

    let (k_triplets, m_triplets, total_mass_kg) =
        assemble_linear_k_and_m(model, &prep, config.mass_formulation)?;
    if total_mass_kg <= 1e-12 {
        return Err(SolverError::SingularMassMatrix);
    }

    let (ray_alpha, ray_beta) =
        resolve_rayleigh_damping(model, config.damping, config.mass_formulation);

    let k_ff = SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &k_triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;
    let m_ff = SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &m_triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;

    // Rayleigh damping matrix C_ff = alpha * M_ff + beta * K_ff
    let mut c_triplets = Vec::with_capacity(m_triplets.len() + k_triplets.len());
    if ray_alpha > 0.0 {
        for t in &m_triplets {
            c_triplets.push(Triplet::new(t.row, t.col, ray_alpha * t.val));
        }
    }
    if ray_beta > 0.0 {
        for t in &k_triplets {
            c_triplets.push(Triplet::new(t.row, t.col, ray_beta * t.val));
        }
    }
    let c_ff = SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &c_triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;

    // Precompute rigid-body influence vectors M_ff * r_d for base excitation
    let mut mr_vecs = [
        vec![0.0_f64; prep.n_free],
        vec![0.0_f64; prep.n_free],
        vec![0.0_f64; prep.n_free],
    ];
    for d in 0..3 {
        let mut r_vec = vec![0.0_f64; prep.n_free];
        for n_idx in 0..prep.n_nodes {
            if let Some(fr) = prep.dof_to_free[n_idx * 6 + d] {
                r_vec[fr] = 1.0;
            }
        }
        mr_vecs[d] = spmv(&m_ff, &r_vec);
    }

    // External force vector on free DOFs at time t
    let eval_f_ext_free = |t: f64| -> Vec<f64> {
        let scale = config.excitation.load_factor(t);
        let ag = config.excitation.base_accel_global(t);
        let mut f = vec![0.0_f64; prep.n_free];
        if scale.abs() > 0.0 {
            for gdof in 0..prep.total_dofs {
                if let Some(fr) = prep.dof_to_free[gdof] {
                    f[fr] += scale * prep.f_ext_global[gdof];
                }
            }
        }
        for d in 0..3 {
            if ag[d].abs() > 0.0 {
                for i in 0..prep.n_free {
                    f[i] -= ag[d] * mr_vecs[d][i];
                }
            }
        }
        f
    };

    // Initial state (u, v, a) on free DOFs
    let mut u_f = vec![0.0_f64; prep.n_free];
    let mut v_f = vec![0.0_f64; prep.n_free];
    let mut u_global = vec![0.0_f64; prep.total_dofs];
    let mut node_rotations = vec![Matrix3::<f64>::identity(); prep.n_nodes];

    // If FreeVibration (pluck & release), initialize u_0 from static equilibrium!
    if matches!(config.excitation, TimeHistoryExcitation::FreeVibration) {
        if config.nonlinear {
            let nl_cfg = NonlinearStaticConfig {
                load_steps: 8,
                stations_per_element: 2,
                ..Default::default()
            };
            let nl_sol = solve_nonlinear_static_with_config(model, &nl_cfg)?;
            for (n_idx, node) in model.nodes.iter().enumerate() {
                if let Some(nr) = nl_sol.solution.node_results.get(&node.id) {
                    for d in 0..6 {
                        u_global[n_idx * 6 + d] = nr.displacements[d];
                        if let Some(fr) = prep.dof_to_free[n_idx * 6 + d] {
                            u_f[fr] = nr.displacements[d];
                        }
                    }
                    let th = Vector3::new(
                        nr.displacements[3],
                        nr.displacements[4],
                        nr.displacements[5],
                    );
                    node_rotations[n_idx] = exp_so3(th);
                }
            }
        } else {
            let mut rhs0 = vec![0.0_f64; prep.n_free];
            for gdof in 0..prep.total_dofs {
                if let Some(fr) = prep.dof_to_free[gdof] {
                    rhs0[fr] = prep.f_ext_global[gdof];
                }
            }
            u_f = solve_sparse_system(prep.n_free, &k_triplets, &rhs0)?;
            for gdof in 0..prep.total_dofs {
                if let Some(fr) = prep.dof_to_free[gdof] {
                    u_global[gdof] = u_f[fr];
                }
            }
        }
    }

    // Solve initial acceleration M_ff * a_0 = F_ext(0) - C_ff * v_0 - F_int(u_0)
    let f0 = eval_f_ext_free(0.0);
    let f_int0 = if config.nonlinear {
        let (fi_glob, _) = assemble_corotational_system(&prep, &u_global, &node_rotations)?;
        let mut fi_f = vec![0.0_f64; prep.n_free];
        for gdof in 0..prep.total_dofs {
            if let Some(fr) = prep.dof_to_free[gdof] {
                fi_f[fr] = fi_glob[gdof];
            }
        }
        fi_f
    } else {
        spmv(&k_ff, &u_f)
    };
    let rhs_a0: Vec<f64> = (0..prep.n_free).map(|i| f0[i] - f_int0[i]).collect();
    let mut a_f = solve_initial_acceleration(prep.n_free, &m_triplets, &rhs_a0)?;

    // For linear dynamics, pre-factorize K_eff = K_ff + a0 * M_ff + a1 * C_ff ONCE!
    enum EffFactor {
        Cholesky(faer::sparse::linalg::solvers::Llt<usize, f64>),
        Lu(faer::sparse::linalg::solvers::Lu<usize, f64>),
    }
    impl EffFactor {
        fn solve_vec(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
            let n = rhs.len();
            let rhs_m = Mat::from_fn(n, 1, |i, _| rhs[i]);
            let sol_m = match self {
                EffFactor::Cholesky(c) => c.solve(&rhs_m),
                EffFactor::Lu(l) => l.solve(&rhs_m),
            };
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                let v = sol_m[(i, 0)];
                if !v.is_finite() {
                    return Err(SolverError::SingularSystem);
                }
                out.push(v);
            }
            Ok(out)
        }
    }

    let linear_k_eff = if !config.nonlinear {
        let mut eff_triplets =
            Vec::with_capacity(k_triplets.len() + m_triplets.len() + c_triplets.len());
        for t in &k_triplets {
            eff_triplets.push(*t);
        }
        for t in &m_triplets {
            eff_triplets.push(Triplet::new(t.row, t.col, a0 * t.val));
        }
        for t in &c_triplets {
            eff_triplets.push(Triplet::new(t.row, t.col, a1 * t.val));
        }
        let k_eff =
            SparseColMat::<usize, f64>::try_new_from_triplets(prep.n_free, prep.n_free, &eff_triplets)
                .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;
        let fac = match k_eff.sp_cholesky(Side::Lower) {
            Ok(c) => EffFactor::Cholesky(c),
            Err(_) => {
                let lu = k_eff.sp_lu().map_err(|_| SolverError::SingularSystem)?;
                EffFactor::Lu(lu)
            }
        };
        Some(fac)
    } else {
        None
    };

    let max_frames = config.max_stored_frames.max(2);
    let frame_stride = ((n_steps + 1) as f64 / max_frames as f64).ceil().max(1.0) as usize;

    let mut history = Vec::with_capacity(n_steps + 1);
    let mut frames = Vec::with_capacity(max_frames.min(n_steps + 1));
    let mut peak_displacement_m = 0.0_f64;
    let mut peak_displacement_time_s = 0.0_f64;
    let mut peak_stress_pa = 0.0_f64;

    let compute_energies_and_record = |step_idx: usize,
                                       t_now: f64,
                                       u_g: &[f64],
                                       u_free: &[f64],
                                       v_free: &[f64],
                                       node_rots: &[Matrix3<f64>],
                                       history: &mut Vec<DynamicHistoryPoint>,
                                       frames: &mut Vec<DynamicFrame>,
                                       peak_disp: &mut f64,
                                       peak_disp_t: &mut f64,
                                       peak_stress: &mut f64|
     -> Result<(), SolverError> {
        let mv = spmv(&m_ff, v_free);
        let ke: f64 = 0.5 * (0..prep.n_free).map(|i| v_free[i] * mv[i]).sum::<f64>();
        let se: f64 = if config.nonlinear {
            // Approximate elastic strain energy via 0.5 * sum(u_def^T K u_def)
            let mut e_acc = 0.0;
            for beam in &prep.beams {
                let st = crate::nonlinear::eval_corotational_beam(beam, u_g, node_rots)?;
                e_acc += 0.5 * st.u_def_local.dot(&st.f_int_local);
            }
            for shell in &prep.shells {
                let (_, u_def, _, _) =
                    crate::nonlinear::eval_corotational_shell(shell, u_g, node_rots)?;
                let fi = &shell.k_local0 * &u_def;
                e_acc += 0.5 * u_def.dot(&fi);
            }
            e_acc.max(0.0)
        } else {
            let ku = spmv(&k_ff, u_free);
            (0.5 * (0..prep.n_free).map(|i| u_free[i] * ku[i]).sum::<f64>()).max(0.0)
        };

        let mut max_u = 0.0_f64;
        let mut node_translations = HashMap::with_capacity(prep.n_nodes);
        for (n_idx, node) in model.nodes.iter().enumerate() {
            let b = n_idx * 6;
            let tr = [u_g[b], u_g[b + 1], u_g[b + 2]];
            let mag = (tr[0].powi(2) + tr[1].powi(2) + tr[2].powi(2)).sqrt();
            if mag > max_u {
                max_u = mag;
            }
            node_translations.insert(node.id, tr);
        }

        if max_u >= *peak_disp {
            *peak_disp = max_u;
            *peak_disp_t = t_now;
        }

        history.push(DynamicHistoryPoint {
            time_s: t_now,
            max_displacement_m: max_u,
            kinetic_energy_j: ke,
            strain_energy_j: se,
            node_translations,
        });

        let should_store = step_idx == 0 || step_idx == n_steps || (step_idx % frame_stride == 0);
        if should_store {
            let f_scale = config.excitation.load_factor(t_now);
            let sol = if config.nonlinear {
                recover_corotational_solution(
                    model,
                    &prep,
                    u_g,
                    node_rots,
                    f_scale,
                    config.stations_per_element,
                    0.0,
                    0.0,
                )?
            } else {
                recover_linear_solution_from_u(
                    model,
                    &prep,
                    u_g,
                    f_scale,
                    config.stations_per_element,
                    0.0,
                )?
            };
            *peak_stress = peak_stress.max(sol.summary.max_stress_pa);
            frames.push(DynamicFrame {
                time_s: t_now,
                max_displacement_m: max_u,
                kinetic_energy_j: ke,
                strain_energy_j: se,
                solution: sol,
            });
        }
        Ok(())
    };

    // Record initial frame t = 0
    compute_energies_and_record(
        0,
        0.0,
        &u_global,
        &u_f,
        &v_f,
        &node_rotations,
        &mut history,
        &mut frames,
        &mut peak_displacement_m,
        &mut peak_displacement_time_s,
        &mut peak_stress_pa,
    )?;

    // Time integration loop
    for step in 1..=n_steps {
        let t_now = step as f64 * dt;
        let f_ext_f = eval_f_ext_free(t_now);

        if let Some(ref fac) = linear_k_eff {
            // Linear Newmark-beta step
            let m_term_vec: Vec<f64> = (0..prep.n_free)
                .map(|i| a0 * u_f[i] + a2 * v_f[i] + a3 * a_f[i])
                .collect();
            let c_term_vec: Vec<f64> = (0..prep.n_free)
                .map(|i| a1 * u_f[i] + a4 * v_f[i] + a5 * a_f[i])
                .collect();
            let m_rhs = spmv(&m_ff, &m_term_vec);
            let c_rhs = spmv(&c_ff, &c_term_vec);
            let r_eff: Vec<f64> = (0..prep.n_free)
                .map(|i| f_ext_f[i] + m_rhs[i] + c_rhs[i])
                .collect();

            let u_next = fac.solve_vec(&r_eff)?;
            let mut a_next = vec![0.0_f64; prep.n_free];
            let mut v_next = vec![0.0_f64; prep.n_free];
            for i in 0..prep.n_free {
                a_next[i] = a0 * (u_next[i] - u_f[i]) - a2 * v_f[i] - a3 * a_f[i];
                v_next[i] = v_f[i] + a6 * a_f[i] + a7 * a_next[i];
            }

            u_f = u_next;
            v_f = v_next;
            a_f = a_next;
            for gdof in 0..prep.total_dofs {
                if let Some(fr) = prep.dof_to_free[gdof] {
                    u_global[gdof] = u_f[fr];
                }
            }
        } else {
            // Nonlinear Corotational Newmark-beta step with full Newton-Raphson equilibrium iterations
            let u_pred: Vec<f64> = (0..prep.n_free)
                .map(|i| u_f[i] + dt * v_f[i] + dt * dt * (0.5 - beta) * a_f[i])
                .collect();
            let v_pred: Vec<f64> = (0..prep.n_free)
                .map(|i| v_f[i] + dt * (1.0 - gamma) * a_f[i])
                .collect();

            // Track incremental displacement from start of step for consistent SO(3) updates
            let mut u_cur = u_f.clone();
            let mut a_cur = vec![0.0_f64; prep.n_free];
            let mut v_cur = vec![0.0_f64; prep.n_free];
            let mut converged = false;
            let mut rel_res = f64::INFINITY;

            let f_scale_norm = f_ext_f
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt()
                .max(1.0);

            for iter in 1..=config.max_newton_iters {
                for i in 0..prep.n_free {
                    a_cur[i] = a0 * (u_cur[i] - u_pred[i]);
                    v_cur[i] = v_pred[i] + a7 * a_cur[i];
                }

                let (f_int_glob, kt_triplets) =
                    assemble_corotational_system(&prep, &u_global, &node_rotations)?;
                let ma = spmv(&m_ff, &a_cur);
                let cv = spmv(&c_ff, &v_cur);

                let mut res_f = vec![0.0_f64; prep.n_free];
                let mut res_norm2 = 0.0_f64;
                let mut dyn_ref2 = f_scale_norm * f_scale_norm;
                for gdof in 0..prep.total_dofs {
                    if let Some(fr) = prep.dof_to_free[gdof] {
                        let r = f_ext_f[fr] - f_int_glob[gdof] - ma[fr] - cv[fr];
                        res_f[fr] = r;
                        res_norm2 += r * r;
                        dyn_ref2 += ma[fr] * ma[fr] + f_int_glob[gdof] * f_int_glob[gdof];
                    }
                }
                rel_res = res_norm2.sqrt() / dyn_ref2.sqrt().max(1.0);
                if rel_res <= config.newton_tol && iter > 1 {
                    converged = true;
                    break;
                }

                let mut eff_triplets =
                    Vec::with_capacity(kt_triplets.len() + m_triplets.len() + c_triplets.len());
                eff_triplets.extend_from_slice(&kt_triplets);
                for t in &m_triplets {
                    eff_triplets.push(Triplet::new(t.row, t.col, a0 * t.val));
                }
                for t in &c_triplets {
                    eff_triplets.push(Triplet::new(t.row, t.col, a1 * t.val));
                }

                let du = solve_sparse_system(prep.n_free, &eff_triplets, &res_f)?;
                for i in 0..prep.n_free {
                    u_cur[i] += du[i];
                }
                apply_increment(&prep, &mut u_global, &mut node_rotations, &du);

                let du_norm = du.iter().map(|v| v * v).sum::<f64>().sqrt();
                let u_norm = u_cur.iter().map(|v| v * v).sum::<f64>().sqrt().max(1.0);
                if rel_res <= config.newton_tol || (du_norm / u_norm <= config.newton_tol && rel_res <= 1e-3)
                {
                    // Refresh a_cur, v_cur with final u_cur
                    for i in 0..prep.n_free {
                        a_cur[i] = a0 * (u_cur[i] - u_pred[i]);
                        v_cur[i] = v_pred[i] + a7 * a_cur[i];
                    }
                    converged = true;
                    break;
                }
            }

            if !converged {
                return Err(SolverError::ConvergenceFailed {
                    step,
                    residual: rel_res,
                });
            }

            u_f = u_cur;
            v_f = v_cur;
            a_f = a_cur;
        }

        compute_energies_and_record(
            step,
            t_now,
            &u_global,
            &u_f,
            &v_f,
            &node_rotations,
            &mut history,
            &mut frames,
            &mut peak_displacement_m,
            &mut peak_displacement_time_s,
            &mut peak_stress_pa,
        )?;
    }

    let solve_time_ms = t_start.elapsed().as_secs_f64() * 1000.0;
    Ok(DynamicSolution {
        frames,
        history,
        peak_displacement_m,
        peak_displacement_time_s,
        peak_stress_pa,
        rayleigh_alpha: ray_alpha,
        rayleigh_beta: ray_beta,
        time_steps: n_steps,
        nonlinear: config.nonlinear,
        solve_time_ms,
    })
}
