use crate::element::{
    ElementFrame, StationSample, beam_local_geometric_stiffness, condense_element_releases,
    consistent_equivalent_local_loads, recover_uncondensed_local_displacements,
    sample_element_stations, total_local_distributed_load, uncondensed_local_stiffness,
};
use crate::model::{BeamFormulation, StructuralModel};
use crate::section::SectionProperties;
use crate::shell::{
    ShellFrame, consistent_global_surface_forces, recover_shell_result, shell_local_stiffness,
    total_global_surface_traction,
};
use crate::solver::{AnalysisSummary, ElementResult, NodeResult, Solution, SolverError};
use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, Triplet};
use faer::{Mat, Side};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Instant;

/// Configuration for geometrically nonlinear (large-deformation) static analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NonlinearStaticConfig {
    /// Number of incremental load steps from lambda = 0 to lambda = 1.
    pub load_steps: usize,
    /// Maximum full Newton-Raphson iterations per load step.
    pub max_iterations_per_step: usize,
    /// Relative force residual convergence tolerance `||r|| / max(||F_ext||, 1)`.
    pub force_tol: f64,
    /// Relative displacement increment convergence tolerance `||du|| / max(||u||, 1)`.
    pub disp_tol: f64,
    /// Maximum adaptive substep bisections if a load increment fails to converge.
    pub max_bisections: usize,
    /// Number of post-processing stations sampled per beam element.
    pub stations_per_element: usize,
}

impl Default for NonlinearStaticConfig {
    fn default() -> Self {
        Self {
            load_steps: 10,
            max_iterations_per_step: 25,
            force_tol: 1e-6,
            disp_tol: 1e-6,
            max_bisections: 5,
            stations_per_element: 25,
        }
    }
}

/// Convergence and load-displacement record for one converged load increment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadStepRecord {
    pub step: usize,
    /// Applied load factor `lambda in (0, 1]`.
    pub load_factor: f64,
    /// Peak nodal translation magnitude `max |u|` (m) at this load factor.
    pub max_displacement_m: f64,
    /// Number of Newton-Raphson iterations required.
    pub iterations: usize,
    /// Final relative force residual norm.
    pub relative_residual: f64,
}

/// Complete output of a geometrically nonlinear (3D corotational) static analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NonlinearSolution {
    /// Full FEA solution at the final load level (directly renderable in the 3D viewport).
    pub solution: Solution,
    /// Equilibrium path history `(load_factor, max_displacement_m, ...)` for plotting.
    pub load_steps: Vec<LoadStepRecord>,
    pub total_iterations: usize,
    pub converged: bool,
}

/// Skew-symmetric cross-product matrix `[v]_x` such that `[v]_x w = v x w`.
#[inline]
pub fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Exact exponential map `so(3) -> SO(3)` via Rodrigues' formula.
pub fn exp_so3(theta: Vector3<f64>) -> Matrix3<f64> {
    let angle2 = theta.norm_squared();
    let w = skew(theta);
    if angle2 < 1e-14 {
        Matrix3::identity() + w + 0.5 * (w * w)
    } else {
        let angle = angle2.sqrt();
        let c1 = angle.sin() / angle;
        let c2 = (1.0 - angle.cos()) / angle2;
        Matrix3::identity() + c1 * w + c2 * (w * w)
    }
}

/// Exact logarithm map `SO(3) -> so(3)` (inverse Rodrigues).
pub fn log_so3(r: &Matrix3<f64>) -> Vector3<f64> {
    let tr = r[(0, 0)] + r[(1, 1)] + r[(2, 2)];
    let cos_theta = (0.5 * (tr - 1.0)).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();

    let w = Vector3::new(
        0.5 * (r[(2, 1)] - r[(1, 2)]),
        0.5 * (r[(0, 2)] - r[(2, 0)]),
        0.5 * (r[(1, 0)] - r[(0, 1)]),
    );

    if theta < 1e-7 {
        w * (1.0 + theta * theta / 6.0)
    } else if std::f64::consts::PI - theta > 1e-5 {
        let sin_theta = theta.sin();
        w * (theta / sin_theta)
    } else {
        // Near pi: extract unit axis from symmetric part 0.5 * (R + I)
        let sym = 0.5 * (r + Matrix3::identity());
        let mut axis = if sym[(0, 0)] >= sym[(1, 1)] && sym[(0, 0)] >= sym[(2, 2)] {
            let x = sym[(0, 0)].max(0.0).sqrt();
            if x > 1e-12 {
                Vector3::new(x, sym[(0, 1)] / x, sym[(0, 2)] / x)
            } else {
                Vector3::x()
            }
        } else if sym[(1, 1)] >= sym[(2, 2)] {
            let y = sym[(1, 1)].max(0.0).sqrt();
            if y > 1e-12 {
                Vector3::new(sym[(1, 0)] / y, y, sym[(1, 2)] / y)
            } else {
                Vector3::y()
            }
        } else {
            let z = sym[(2, 2)].max(0.0).sqrt();
            if z > 1e-12 {
                Vector3::new(sym[(2, 0)] / z, sym[(2, 1)] / z, z)
            } else {
                Vector3::z()
            }
        };
        let norm = axis.norm();
        if norm > 1e-12 {
            axis /= norm;
        }
        if axis.dot(&w) < 0.0 {
            axis = -axis;
        }
        axis * theta
    }
}

/// Pre-validated model data shared by nonlinear static and dynamic solvers.
pub(crate) struct PreparedBeam {
    pub id: usize,
    pub i_idx: usize,
    pub j_idx: usize,
    pub p_i: [f64; 3],
    pub p_j: [f64; 3],
    pub frame0: ElementFrame,
    pub e: f64,
    pub g: f64,
    pub density: f64,
    pub fy: f64,
    pub props: SectionProperties,
    pub formulation: BeamFormulation,
    pub releases: [bool; 12],
    pub k_uncond_local: SMatrix<f64, 12, 12>,
    pub k_cond_local: SMatrix<f64, 12, 12>,
    pub f_fef_uncond_local: SVector<f64, 12>,
    pub q_local_start: [f64; 3],
    pub q_local_end: [f64; 3],
    pub dloads: Vec<(crate::model::LoadCoordSystem, [f64; 3], [f64; 3])>,
}

pub(crate) struct PreparedShell {
    pub id: usize,
    pub node_indices: Vec<usize>,
    pub frame0: ShellFrame,
    pub e: f64,
    pub g: f64,
    pub nu: f64,
    pub density: f64,
    pub fy: f64,
    pub thickness: f64,
    pub k_local0: DMatrix<f64>,
}

pub(crate) struct PreparedModel {
    pub n_nodes: usize,
    pub total_dofs: usize,
    pub n_free: usize,
    pub dof_to_free: Vec<Option<usize>>,
    pub prescribed_full: Vec<f64>,
    pub f_ext_global: Vec<f64>,
    pub total_applied_force: [f64; 3],
    pub beams: Vec<PreparedBeam>,
    pub shells: Vec<PreparedShell>,
}

pub(crate) fn prepare_model(model: &StructuralModel) -> Result<PreparedModel, SolverError> {
    if model.nodes.is_empty() {
        return Err(SolverError::NoNodes);
    }
    if model.elements.is_empty() && model.shells.is_empty() {
        return Err(SolverError::NoElements);
    }

    let n_nodes = model.nodes.len();
    let total_dofs = n_nodes * 6;

    let mut node_id_to_idx = HashMap::with_capacity(n_nodes);
    for (idx, node) in model.nodes.iter().enumerate() {
        node_id_to_idx.insert(node.id, idx);
    }

    let mut node_connected = vec![false; n_nodes];
    for elem in &model.elements {
        let &i_idx = node_id_to_idx
            .get(&elem.node_i)
            .ok_or(SolverError::MissingNode {
                element_id: elem.id,
                node_id: elem.node_i,
            })?;
        let &j_idx = node_id_to_idx
            .get(&elem.node_j)
            .ok_or(SolverError::MissingNode {
                element_id: elem.id,
                node_id: elem.node_j,
            })?;
        node_connected[i_idx] = true;
        node_connected[j_idx] = true;
    }
    for shell in &model.shells {
        for &nid in shell.nodes.as_slice() {
            let &n_idx = node_id_to_idx
                .get(&nid)
                .ok_or(SolverError::MissingNode {
                    element_id: shell.id,
                    node_id: nid,
                })?;
            node_connected[n_idx] = true;
        }
    }
    for (idx, node) in model.nodes.iter().enumerate() {
        if !node_connected[idx] && node.restraints.iter().any(|&r| !r) {
            return Err(SolverError::OrphanNode {
                node_id: node.id,
                label: node.label.clone(),
            });
        }
    }

    let mut dof_to_free = vec![None; total_dofs];
    let mut prescribed_full = vec![0.0_f64; total_dofs];
    let mut n_free = 0usize;
    let mut n_restrained = 0usize;

    for (n_idx, node) in model.nodes.iter().enumerate() {
        for d in 0..6 {
            let gdof = n_idx * 6 + d;
            if node.restraints[d] {
                prescribed_full[gdof] = node.prescribed[d];
                n_restrained += 1;
            } else {
                dof_to_free[gdof] = Some(n_free);
                n_free += 1;
            }
        }
    }

    if n_restrained == 0 {
        return Err(SolverError::NoSupports);
    }

    let mut elem_dloads: HashMap<usize, Vec<_>> = HashMap::new();
    for dl in &model.distributed_loads {
        elem_dloads
            .entry(dl.element_id)
            .or_default()
            .push((dl.coord_system, dl.q_start, dl.q_end));
    }

    let mut f_ext_global = vec![0.0_f64; total_dofs];
    let mut total_applied_force = [0.0_f64; 3];

    for nl in &model.nodal_loads {
        if let Some(&n_idx) = node_id_to_idx.get(&nl.node_id) {
            let base = n_idx * 6;
            for d in 0..3 {
                f_ext_global[base + d] += nl.forces[d];
                total_applied_force[d] += nl.forces[d];
            }
            for d in 0..3 {
                f_ext_global[base + 3 + d] += nl.moments[d];
            }
        }
    }

    let mut beams = Vec::with_capacity(model.elements.len());
    for elem in &model.elements {
        let i_idx = node_id_to_idx[&elem.node_i];
        let j_idx = node_id_to_idx[&elem.node_j];
        let p_i = model.nodes[i_idx].position;
        let p_j = model.nodes[j_idx].position;

        let mat = model
            .get_material(elem.material_id)
            .ok_or(SolverError::MissingMaterial {
                element_id: elem.id,
                material_id: elem.material_id,
            })?;
        let sec = model
            .get_section(elem.section_id)
            .ok_or(SolverError::MissingSection {
                element_id: elem.id,
                section_id: elem.section_id,
            })?;
        let props = sec.properties();

        let frame0 = ElementFrame::compute(p_i, p_j, elem.roll_angle_deg).map_err(|reason| {
            SolverError::InvalidElement {
                element_id: elem.id,
                reason,
            }
        })?;

        let k_uncond_local =
            uncondensed_local_stiffness(mat.e, mat.g, &props, frame0.length, elem.formulation);

        let dloads = elem_dloads.remove(&elem.id).unwrap_or_default();
        let (q_local_start, q_local_end) = total_local_distributed_load(
            &frame0,
            mat.density,
            props.area,
            &model.gravity,
            &dloads,
        );

        let q_avg_local = Vector3::new(
            0.5 * (q_local_start[0] + q_local_end[0]),
            0.5 * (q_local_start[1] + q_local_end[1]),
            0.5 * (q_local_start[2] + q_local_end[2]),
        );
        let f_elem_global = frame0.r_mat.transpose() * (q_avg_local * frame0.length);
        for d in 0..3 {
            total_applied_force[d] += f_elem_global[d];
        }

        let f_eq_uncond_local = consistent_equivalent_local_loads(
            q_local_start,
            q_local_end,
            mat.e,
            mat.g,
            &props,
            frame0.length,
            elem.formulation,
        );
        let f_fef_uncond_local = -f_eq_uncond_local;

        let mut releases = [false; 12];
        releases[..6].copy_from_slice(&elem.releases_i);
        releases[6..].copy_from_slice(&elem.releases_j);

        let (k_cond_local, f_fef_cond_local) =
            condense_element_releases(&k_uncond_local, &f_fef_uncond_local, &releases);

        let t0 = frame0.transformation_12x12();
        let f_eq_cond_global = t0.transpose() * (-f_fef_cond_local);

        for d in 0..6 {
            f_ext_global[i_idx * 6 + d] += f_eq_cond_global[d];
            f_ext_global[j_idx * 6 + d] += f_eq_cond_global[6 + d];
        }

        beams.push(PreparedBeam {
            id: elem.id,
            i_idx,
            j_idx,
            p_i,
            p_j,
            frame0,
            e: mat.e,
            g: mat.g,
            density: mat.density,
            fy: mat.yield_strength.max(1.0),
            props,
            formulation: elem.formulation,
            releases,
            k_uncond_local,
            k_cond_local,
            f_fef_uncond_local,
            q_local_start,
            q_local_end,
            dloads,
        });
    }

    let mut shell_sloads: HashMap<usize, Vec<_>> = HashMap::new();
    for sl in &model.surface_loads {
        shell_sloads
            .entry(sl.shell_id)
            .or_default()
            .push((sl.coord_system, sl.pressure));
    }

    let mut shells = Vec::with_capacity(model.shells.len());
    for shell in &model.shells {
        let node_ids = shell.nodes.as_slice();
        let node_indices: Vec<usize> = node_ids.iter().map(|nid| node_id_to_idx[nid]).collect();
        let positions: Vec<[f64; 3]> = node_indices
            .iter()
            .map(|&idx| model.nodes[idx].position)
            .collect();

        let mat = model
            .get_material(shell.material_id)
            .ok_or(SolverError::MissingMaterial {
                element_id: shell.id,
                material_id: shell.material_id,
            })?;
        let nu = mat.poisson_ratio();

        let frame0 = ShellFrame::compute(&positions).map_err(|reason| SolverError::InvalidElement {
            element_id: shell.id,
            reason,
        })?;
        let k_local0 = shell_local_stiffness(&frame0, mat.e, mat.g, nu, shell.thickness).map_err(
            |reason| SolverError::InvalidElement {
                element_id: shell.id,
                reason,
            },
        )?;

        let sloads_slice = shell_sloads
            .get(&shell.id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        let p_global = total_global_surface_traction(
            &frame0,
            mat.density,
            shell.thickness,
            &model.gravity,
            sloads_slice,
        );
        for d in 0..3 {
            total_applied_force[d] += p_global[d] * frame0.area;
        }
        let f_eq_global = consistent_global_surface_forces(&frame0, p_global).map_err(|reason| {
            SolverError::InvalidElement {
                element_id: shell.id,
                reason,
            }
        })?;

        for (k, &n_idx) in node_indices.iter().enumerate() {
            for d in 0..6 {
                f_ext_global[n_idx * 6 + d] += f_eq_global[k * 6 + d];
            }
        }

        shells.push(PreparedShell {
            id: shell.id,
            node_indices,
            frame0,
            e: mat.e,
            g: mat.g,
            nu,
            density: mat.density,
            fy: mat.yield_strength.max(1.0),
            thickness: shell.thickness,
            k_local0,
        });
    }

    Ok(PreparedModel {
        n_nodes,
        total_dofs,
        n_free,
        dof_to_free,
        prescribed_full,
        f_ext_global,
        total_applied_force,
        beams,
        shells,
    })
}

/// Corotational deformed frame and deformational local displacements for a 3D beam element.
pub(crate) struct CorotatedBeamState {
    pub frame_def: ElementFrame,
    pub u_def_local: SVector<f64, 12>,
    pub f_int_local: SVector<f64, 12>,
    pub f_int_global: SVector<f64, 12>,
    pub k_t_global: SMatrix<f64, 12, 12>,
}

/// Evaluates 3D corotational internal forces and tangent stiffness for one beam element.
pub(crate) fn eval_corotational_beam(
    beam: &PreparedBeam,
    u_global: &[f64],
    node_rotations: &[Matrix3<f64>],
) -> Result<CorotatedBeamState, SolverError> {
    let bi = beam.i_idx * 6;
    let bj = beam.j_idx * 6;

    let du = Vector3::new(
        u_global[bj] - u_global[bi],
        u_global[bj + 1] - u_global[bi + 1],
        u_global[bj + 2] - u_global[bi + 2],
    );
    let dx0 = Vector3::new(
        beam.p_j[0] - beam.p_i[0],
        beam.p_j[1] - beam.p_i[1],
        beam.p_j[2] - beam.p_i[2],
    );
    let dx = dx0 + du;
    let l = dx.norm();
    if l < 1e-9 {
        return Err(SolverError::InvalidElement {
            element_id: beam.id,
            reason: "Beam collapsed to zero length during large deformation.".to_string(),
        });
    }
    let e1 = dx / l;
    // Cancellation-free exact axial elongation: L - L0 = (2 * dx0 . du + ||du||^2) / (L + L0)
    let elongation = (2.0 * dx0.dot(&du) + du.norm_squared()) / (l + beam.frame0.length);

    // Initial local basis columns Q0 = R0^T = [ex0, ey0, ez0]
    let q0 = beam.frame0.r_mat.transpose();
    let ey0 = q0.column(1).into_owned();
    let ez0 = q0.column(2).into_owned();

    let r1 = node_rotations[beam.i_idx];
    let r2 = node_rotations[beam.j_idx];

    // Crisfield's mean nodal y-axis projection for frame-invariant corotated triad
    let q_y = 0.5 * (r1 * ey0 + r2 * ey0);
    let e3_raw = e1.cross(&q_y);
    let e3 = if e3_raw.norm() > 1e-8 {
        e3_raw.normalize()
    } else {
        let q_z = 0.5 * (r1 * ez0 + r2 * ez0);
        let e2_alt = q_z.cross(&e1);
        if e2_alt.norm() > 1e-8 {
            e1.cross(&e2_alt.normalize()).normalize()
        } else {
            // Ultimate fallback
            let ref_v = if e1.z.abs() < 0.9 {
                Vector3::z()
            } else {
                Vector3::x()
            };
            e1.cross(&ref_v).normalize()
        }
    };
    let e2 = e3.cross(&e1).normalize();

    let r_mat = Matrix3::from_rows(&[e1.transpose(), e2.transpose(), e3.transpose()]);
    let q_e = r_mat.transpose();
    let frame_def = ElementFrame { length: l, r_mat };

    // Relative deformational rotations in corotated frame: R_bar_a = Q_e^T * R_a * Q_0
    let r_bar1 = q_e.transpose() * r1 * q0;
    let r_bar2 = q_e.transpose() * r2 * q0;
    let th1 = log_so3(&r_bar1);
    let th2 = log_so3(&r_bar2);

    let mut u_def_local = SVector::<f64, 12>::zeros();
    u_def_local[3] = th1.x;
    u_def_local[4] = th1.y;
    u_def_local[5] = th1.z;
    u_def_local[6] = elongation;
    u_def_local[9] = th2.x;
    u_def_local[10] = th2.y;
    u_def_local[11] = th2.z;

    // Basic internal forces from condensed local stiffness
    let f_basic = beam.k_cond_local * u_def_local;

    // Enforce exact deformed-configuration equilibrium using current length L
    let axial_n = f_basic[6];
    let my1 = f_basic[4];
    let my2 = f_basic[10];
    let mz1 = f_basic[5];
    let mz2 = f_basic[11];
    let vy1 = (mz1 + mz2) / l;
    let vz1 = -(my1 + my2) / l;

    let mut f_int_local = f_basic;
    f_int_local[0] = -axial_n;
    f_int_local[6] = axial_n;
    f_int_local[1] = vy1;
    f_int_local[7] = -vy1;
    f_int_local[2] = vz1;
    f_int_local[8] = -vz1;

    let t_e = frame_def.transformation_12x12();
    let t_tr = t_e.transpose();
    let f_int_global = t_tr * f_int_local;

    let kg_local = beam_local_geometric_stiffness(axial_n, vy1, vz1, l);
    let k_t_global = t_tr * (beam.k_cond_local + kg_local) * t_e;

    Ok(CorotatedBeamState {
        frame_def,
        u_def_local,
        f_int_local,
        f_int_global,
        k_t_global,
    })
}

/// Evaluates 3D corotational internal forces and tangent stiffness for one shell element.
pub(crate) fn eval_corotational_shell(
    shell: &PreparedShell,
    u_global: &[f64],
    node_rotations: &[Matrix3<f64>],
) -> Result<(ShellFrame, DVector<f64>, DVector<f64>, DMatrix<f64>), SolverError> {
    let n = shell.node_indices.len();
    let ndof = n * 6;

    let mut def_positions = Vec::with_capacity(n);
    for &n_idx in &shell.node_indices {
        let base = n_idx * 6;
        // Reconstruct undeformed position from frame0 or pass positions; we can reconstruct from frame0!
        // Note: frame0.centroid + Q0 * [local_xy0.x, local_xy0.y, 0] is exact for flat shells.
        def_positions.push([0.0; 3]);
        let _ = base;
    }
    // Reconstruct undeformed positions from frame0 + add displacements
    let q0 = shell.frame0.r_mat.transpose();
    let c0 = Vector3::from(shell.frame0.centroid);
    for (k, &n_idx) in shell.node_indices.iter().enumerate() {
        let [lx, ly] = shell.frame0.local_xy[k];
        let p0 = c0 + q0 * Vector3::new(lx, ly, 0.0);
        let base = n_idx * 6;
        def_positions[k] = [
            p0.x + u_global[base],
            p0.y + u_global[base + 1],
            p0.z + u_global[base + 2],
        ];
    }

    let frame_def =
        ShellFrame::compute(&def_positions).map_err(|reason| SolverError::InvalidElement {
            element_id: shell.id,
            reason,
        })?;
    let q_e = frame_def.r_mat.transpose();
    let c_def = Vector3::from(frame_def.centroid);

    let mut u_def_local = DVector::<f64>::zeros(ndof);
    let mut loc_def_coords = Vec::with_capacity(n);
    for (k, &n_idx) in shell.node_indices.iter().enumerate() {
        let rel = Vector3::from(def_positions[k]) - c_def;
        let loc_def = frame_def.r_mat * rel;
        loc_def_coords.push(loc_def);
        let [x0, y0] = shell.frame0.local_xy[k];

        let r_a = node_rotations[n_idx];
        let r_bar = q_e.transpose() * r_a * q0;
        let th = log_so3(&r_bar);

        let base = k * 6;
        u_def_local[base] = loc_def.x - x0;
        u_def_local[base + 1] = loc_def.y - y0;
        u_def_local[base + 2] = loc_def.z;
        u_def_local[base + 3] = th.x;
        u_def_local[base + 4] = th.y;
        u_def_local[base + 5] = th.z;
    }

    let f_basic_local = &shell.k_local0 * &u_def_local;
    let t_e = frame_def.transformation_matrix();

    // Exact corotational frame-spin matrix G (3 x 6n): delta_omega_e = G * delta_u_loc
    let mut g_spin = DMatrix::<f64>::zeros(3, ndof);
    if n == 3 {
        let a = loc_def_coords[1] - loc_def_coords[0];
        let b = loc_def_coords[2] - loc_def_coords[0];
        let n_len = a.cross(&b).norm().max(1e-12);
        let inv_n = 1.0 / n_len;
        let mats = [
            skew(loc_def_coords[2] - loc_def_coords[1]) * inv_n,
            skew(loc_def_coords[0] - loc_def_coords[2]) * inv_n,
            skew(loc_def_coords[1] - loc_def_coords[0]) * inv_n,
        ];
        for (k, m) in mats.iter().enumerate() {
            let col = k * 6;
            for c in 0..3 {
                // delta_omega_x = -delta_e3_y, delta_omega_y = +delta_e3_x
                g_spin[(0, col + c)] = -m[(1, c)];
                g_spin[(1, col + c)] = m[(0, c)];
            }
        }
    } else {
        let d1 = loc_def_coords[2] - loc_def_coords[0];
        let d2 = loc_def_coords[3] - loc_def_coords[1];
        let n_len = d1.cross(&d2).norm().max(1e-12);
        let inv_n = 1.0 / n_len;
        let mats = [
            skew(d2) * inv_n,
            -skew(d1) * inv_n,
            -skew(d2) * inv_n,
            skew(d1) * inv_n,
        ];
        for (k, m) in mats.iter().enumerate() {
            let col = k * 6;
            for c in 0..3 {
                g_spin[(0, col + c)] = -m[(1, c)];
                g_spin[(1, col + c)] = m[(0, c)];
            }
        }
    }
    // In-plane drill spin delta_omega_z from edge (p1 - p0)
    let e01 = loc_def_coords[1] - loc_def_coords[0];
    let l01 = (e01.x * e01.x + e01.y * e01.y).sqrt().max(1e-12);
    let inv_l01 = 1.0 / l01;
    let ux01 = e01.x * inv_l01;
    let uy01 = e01.y * inv_l01;
    g_spin[(2, 0)] = uy01 * inv_l01;
    g_spin[(2, 1)] = -ux01 * inv_l01;
    g_spin[(2, 6)] = -uy01 * inv_l01;
    g_spin[(2, 7)] = ux01 * inv_l01;

    // Rigid-body rotation mode matrix S (6n x 3) and corotational projector P = I - S * G
    let mut s_mat = DMatrix::<f64>::zeros(ndof, 3);
    for k in 0..n {
        let base = k * 6;
        let neg_skew_r = -skew(loc_def_coords[k]);
        for r in 0..3 {
            for c in 0..3 {
                s_mat[(base + r, c)] = neg_skew_r[(r, c)];
            }
            s_mat[(base + 3 + r, r)] = 1.0;
        }
    }
    let p_proj = DMatrix::<f64>::identity(ndof, ndof) - &s_mat * &g_spin;

    // Project local internal forces by P^T to enforce exact deformed-configuration moment equilibrium
    let f_int_local = p_proj.transpose() * &f_basic_local;
    let f_int_global = t_e.transpose() * &f_int_local;

    // Nodal-force spin matrix F_n (translations only) and nodal-moment spin matrix F_m (rotations only).
    // Exact EICR tangent (Felippa & Haugen / Crisfield & Moita):
    //   d(T_e^T)/du * f_l         = -(F_n + F_m) * G
    //   T_e^T * d(P^T)/du * f_b   = -G^T * F_n^T   (since S_rot = I is constant!)
    // Symmetrizing the small moment-spin part gives:
    //   K_gr_sym = -(F_n * G + G^T * F_n^T) - 0.5 * (F_m * G + G^T * F_m^T)
    let mut f_n = DMatrix::<f64>::zeros(ndof, 3);
    let mut f_m = DMatrix::<f64>::zeros(ndof, 3);
    for k in 0..n {
        let base = k * 6;
        let fk = Vector3::new(f_int_local[base], f_int_local[base + 1], f_int_local[base + 2]);
        let mk = Vector3::new(
            f_int_local[base + 3],
            f_int_local[base + 4],
            f_int_local[base + 5],
        );
        let sk_f = skew(fk);
        let sk_m = skew(mk);
        for r in 0..3 {
            for c in 0..3 {
                f_n[(base + r, c)] = sk_f[(r, c)];
                f_m[(base + 3 + r, c)] = sk_m[(r, c)];
            }
        }
    }

    let k_mat_proj = p_proj.transpose() * &shell.k_local0 * &p_proj;
    let kn_g = &f_n * &g_spin;
    let km_g = &f_m * &g_spin;
    let k_gr_sym = (&kn_g + kn_g.transpose()) + 0.5 * (&km_g + km_g.transpose());
    let k_t_local = k_mat_proj - k_gr_sym;
    let k_t_global = t_e.transpose() * k_t_local * &t_e;
    Ok((frame_def, u_def_local, f_int_global, k_t_global))
}

/// Assembles global corotational internal force vector `f_int_global` and sparse free-DOF tangent `K_T,ff`.
pub(crate) fn assemble_corotational_system(
    prep: &PreparedModel,
    u_global: &[f64],
    node_rotations: &[Matrix3<f64>],
) -> Result<(Vec<f64>, Vec<Triplet<usize, usize, f64>>), SolverError> {
    let mut f_int_global = vec![0.0_f64; prep.total_dofs];
    let mut triplets = Vec::with_capacity(prep.beams.len() * 144 + prep.shells.len() * 576);

    for beam in &prep.beams {
        let state = eval_corotational_beam(beam, u_global, node_rotations)?;
        let mut gdofs = [0usize; 12];
        for d in 0..6 {
            gdofs[d] = beam.i_idx * 6 + d;
            gdofs[6 + d] = beam.j_idx * 6 + d;
        }
        for a in 0..12 {
            f_int_global[gdofs[a]] += state.f_int_global[a];
        }
        for r in 0..12 {
            if let Some(fr) = prep.dof_to_free[gdofs[r]] {
                for c in 0..12 {
                    let val = state.k_t_global[(r, c)];
                    if val.abs() == 0.0 {
                        continue;
                    }
                    if let Some(fc) = prep.dof_to_free[gdofs[c]] {
                        triplets.push(Triplet::new(fr, fc, val));
                    }
                }
            }
        }
    }

    for shell in &prep.shells {
        let (_frame, _u_def, f_sh, k_sh) =
            eval_corotational_shell(shell, u_global, node_rotations)?;
        let ndof = shell.node_indices.len() * 6;
        let mut gdofs = Vec::with_capacity(ndof);
        for &n_idx in &shell.node_indices {
            for d in 0..6 {
                gdofs.push(n_idx * 6 + d);
            }
        }
        for a in 0..ndof {
            f_int_global[gdofs[a]] += f_sh[a];
        }
        for r in 0..ndof {
            if let Some(fr) = prep.dof_to_free[gdofs[r]] {
                for c in 0..ndof {
                    let val = k_sh[(r, c)];
                    if val.abs() == 0.0 {
                        continue;
                    }
                    if let Some(fc) = prep.dof_to_free[gdofs[c]] {
                        triplets.push(Triplet::new(fr, fc, val));
                    }
                }
            }
        }
    }

    Ok((f_int_global, triplets))
}

/// Solves a sparse linear system `A x = b` using Cholesky with LU fallback (for indefinite tangent matrices).
pub(crate) fn solve_sparse_system(
    n: usize,
    triplets: &[Triplet<usize, usize, f64>],
    rhs: &[f64],
) -> Result<Vec<f64>, SolverError> {
    if n == 0 {
        return Ok(Vec::new());
    }
    let mat = SparseColMat::<usize, f64>::try_new_from_triplets(n, n, triplets)
        .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;
    let rhs_mat = Mat::from_fn(n, 1, |i, _| rhs[i]);

    let sol_mat = match mat.sp_cholesky(Side::Lower) {
        Ok(llt) => llt.solve(&rhs_mat),
        Err(_) => {
            let lu = mat.sp_lu().map_err(|_| SolverError::SingularSystem)?;
            lu.solve(&rhs_mat)
        }
    };

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let v = sol_mat[(i, 0)];
        if !v.is_finite() {
            return Err(SolverError::SingularSystem);
        }
        out.push(v);
    }
    Ok(out)
}

/// Applies incremental global displacements `du_free` to `u_global` and multiplicatively updates `node_rotations`.
pub(crate) fn apply_increment(
    prep: &PreparedModel,
    u_global: &mut [f64],
    node_rotations: &mut [Matrix3<f64>],
    du_free: &[f64],
) {
    for n_idx in 0..prep.n_nodes {
        let base = n_idx * 6;
        for d in 0..3 {
            if let Some(fr) = prep.dof_to_free[base + d] {
                u_global[base + d] += du_free[fr];
            }
        }
        let mut dth = Vector3::zeros();
        let mut free_rot_count = 0usize;
        for d in 0..3 {
            if let Some(fr) = prep.dof_to_free[base + 3 + d] {
                dth[d] = du_free[fr];
                free_rot_count += 1;
            }
        }
        if free_rot_count == 3 {
            if dth.norm_squared() > 0.0 {
                node_rotations[n_idx] = exp_so3(dth) * node_rotations[n_idx];
                let total_th = log_so3(&node_rotations[n_idx]);
                u_global[base + 3] = total_th.x;
                u_global[base + 4] = total_th.y;
                u_global[base + 5] = total_th.z;
            }
        } else if free_rot_count > 0 && dth.norm_squared() > 0.0 {
            // Partial rotational restraints: preserve exact restrained rotation components
            for d in 0..3 {
                if prep.dof_to_free[base + 3 + d].is_some() {
                    u_global[base + 3 + d] += dth[d];
                }
            }
            let th = Vector3::new(u_global[base + 3], u_global[base + 4], u_global[base + 5]);
            node_rotations[n_idx] = exp_so3(th);
        }
    }
}

/// Sets prescribed DOFs on `u_global` and updates restrained nodal rotation matrices at load factor `lambda`.
fn apply_prescribed_at_lambda(
    model: &StructuralModel,
    prep: &PreparedModel,
    u_global: &mut [f64],
    node_rotations: &mut [Matrix3<f64>],
    lambda: f64,
) {
    for (n_idx, node) in model.nodes.iter().enumerate() {
        let base = n_idx * 6;
        let mut rot_prescribed = false;
        for d in 0..6 {
            if node.restraints[d] {
                u_global[base + d] = lambda * prep.prescribed_full[base + d];
                if d >= 3 {
                    rot_prescribed = true;
                }
            }
        }
        if rot_prescribed {
            let th = Vector3::new(u_global[base + 3], u_global[base + 4], u_global[base + 5]);
            node_rotations[n_idx] = exp_so3(th);
        }
    }
}

/// Recovers a complete `Solution` (node displacements/reactions, beam stations in true deformed geometry, shell results).
#[allow(clippy::too_many_arguments)]
pub(crate) fn recover_corotational_solution(
    model: &StructuralModel,
    prep: &PreparedModel,
    u_global: &[f64],
    node_rotations: &[Matrix3<f64>],
    f_ext_scale: f64,
    stations_per_element: usize,
    solve_time_ms: f64,
    solver_relative_residual: f64,
) -> Result<Solution, SolverError> {
    let (f_int_global, _) = assemble_corotational_system(prep, u_global, node_rotations)?;

    let mut node_results = HashMap::with_capacity(prep.n_nodes);
    let mut total_reaction_force = [0.0_f64; 3];
    let mut max_displacement_m = 0.0_f64;
    let mut max_displacement_node = None;

    for (n_idx, node) in model.nodes.iter().enumerate() {
        let base = n_idx * 6;
        let mut disp = [0.0_f64; 6];
        let mut reac = [0.0_f64; 6];
        for d in 0..6 {
            disp[d] = u_global[base + d];
            if node.restraints[d] {
                reac[d] = f_int_global[base + d] - f_ext_scale * prep.f_ext_global[base + d];
                if d < 3 {
                    total_reaction_force[d] += reac[d];
                }
            }
        }
        let u_mag = (disp[0] * disp[0] + disp[1] * disp[1] + disp[2] * disp[2]).sqrt();
        if u_mag >= max_displacement_m {
            max_displacement_m = u_mag;
            max_displacement_node = Some(node.id);
        }
        node_results.insert(
            node.id,
            NodeResult {
                node_id: node.id,
                displacements: disp,
                reactions: reac,
            },
        );
    }

    let eq_res = [
        f_ext_scale * prep.total_applied_force[0] + total_reaction_force[0],
        f_ext_scale * prep.total_applied_force[1] + total_reaction_force[1],
        f_ext_scale * prep.total_applied_force[2] + total_reaction_force[2],
    ];
    let force_equilibrium_residual_n =
        (eq_res[0] * eq_res[0] + eq_res[1] * eq_res[1] + eq_res[2] * eq_res[2]).sqrt();

    let mut element_results = HashMap::with_capacity(prep.beams.len());
    let mut max_axial_n = 0.0_f64;
    let mut min_axial_n = 0.0_f64;
    let mut max_shear_n = 0.0_f64;
    let mut max_moment_nm = 0.0_f64;
    let mut max_torque_nm = 0.0_f64;
    let mut max_stress_pa = 0.0_f64;
    let mut max_utilization = 0.0_f64;
    let mut max_utilization_element = None;

    for beam in &prep.beams {
        let state = eval_corotational_beam(beam, u_global, node_rotations)?;
        let (q_loc_s_raw, q_loc_e_raw) = total_local_distributed_load(
            &state.frame_def,
            beam.density,
            beam.props.area,
            &model.gravity,
            &beam.dloads,
        );
        // Conservative dead load total force = q * L0 distributed over current length L_def
        let len_ratio = beam.frame0.length / state.frame_def.length.max(1e-9);
        let q_scale = f_ext_scale * len_ratio;
        let q_loc_s = [
            q_scale * q_loc_s_raw[0],
            q_scale * q_loc_s_raw[1],
            q_scale * q_loc_s_raw[2],
        ];
        let q_loc_e = [
            q_scale * q_loc_e_raw[0],
            q_scale * q_loc_e_raw[1],
            q_scale * q_loc_e_raw[2],
        ];

        // Rotate undeformed fixed-end force vector from frame0 into frame_def so it matches
        // the conservative dead loads balanced in global equilibrium!
        let r_rel = state.frame_def.r_mat * beam.frame0.r_mat.transpose();
        let mut f_fef_rot = SVector::<f64, 12>::zeros();
        for blk in 0..4 {
            let b = blk * 3;
            let v0 = Vector3::new(
                beam.f_fef_uncond_local[b],
                beam.f_fef_uncond_local[b + 1],
                beam.f_fef_uncond_local[b + 2],
            ) * f_ext_scale;
            let v_def = r_rel * v0;
            f_fef_rot[b] = v_def.x;
            f_fef_rot[b + 1] = v_def.y;
            f_fef_rot[b + 2] = v_def.z;
        }

        let u_uncond_local = recover_uncondensed_local_displacements(
            &state.u_def_local,
            &beam.k_uncond_local,
            &f_fef_rot,
            &beam.releases,
        );
        let mut f_int_local = beam.k_uncond_local * u_uncond_local;
        // Enforce deformed-configuration equilibrium on elastic part using current length L_def
        let l_def = state.frame_def.length.max(1e-9);
        let n_el = f_int_local[6];
        let vy_el = (f_int_local[5] + f_int_local[11]) / l_def;
        let vz_el = -(f_int_local[4] + f_int_local[10]) / l_def;
        f_int_local[0] = -n_el;
        f_int_local[6] = n_el;
        f_int_local[1] = vy_el;
        f_int_local[7] = -vy_el;
        f_int_local[2] = vz_el;
        f_int_local[8] = -vz_el;
        f_int_local += f_fef_rot;

        let mut stations = sample_element_stations(
            beam.p_i,
            &state.frame_def,
            beam.e,
            beam.g,
            &beam.props,
            beam.formulation,
            &u_uncond_local,
            &f_int_local,
            q_loc_s,
            q_loc_e,
            stations_per_element,
        );

        // Map station displacements to exact true global deformed geometry:
        // pos_def(xi) = x1 + (xi * L_def) * e1 + uy_loc * e2 + uz_loc * e3
        // disp_global(xi) = pos_def(xi) - pos_undef(xi)
        let bi = beam.i_idx * 6;
        let x1 = Vector3::new(
            beam.p_i[0] + u_global[bi],
            beam.p_i[1] + u_global[bi + 1],
            beam.p_i[2] + u_global[bi + 2],
        );
        let pi0 = Vector3::from(beam.p_i);
        let pj0 = Vector3::from(beam.p_j);
        let e1 = Vector3::from(state.frame_def.ex());
        let e2 = Vector3::from(state.frame_def.ey());
        let e3 = Vector3::from(state.frame_def.ez());

        let mut elem_max_sigma = 0.0_f64;
        let mut elem_max_vm = 0.0_f64;

        for s in &mut stations {
            let pos_u = pi0 + (pj0 - pi0) * s.xi;
            let pos_def = x1
                + e1 * (s.xi * state.frame_def.length)
                + e2 * s.disp_local[1]
                + e3 * s.disp_local[2];
            let disp_g = pos_def - pos_u;
            s.pos_undeformed = [pos_u.x, pos_u.y, pos_u.z];
            s.disp_global = [disp_g.x, disp_g.y, disp_g.z];

            let disp_mag = disp_g.norm();
            if disp_mag > max_displacement_m {
                max_displacement_m = disp_mag;
            }
            max_axial_n = max_axial_n.max(s.axial_n);
            min_axial_n = min_axial_n.min(s.axial_n);
            max_shear_n = max_shear_n.max(s.shear_vy.abs()).max(s.shear_vz.abs());
            max_moment_nm = max_moment_nm.max(s.moment_my.abs()).max(s.moment_mz.abs());
            max_torque_nm = max_torque_nm.max(s.torque_t.abs());
            max_stress_pa = max_stress_pa.max(s.max_normal_stress);
            elem_max_sigma = elem_max_sigma.max(s.max_normal_stress);
            elem_max_vm = elem_max_vm.max(s.von_mises_stress);
        }

        let elem_util = elem_max_sigma / beam.fy;
        let elem_vm_util = elem_max_vm / beam.fy;
        if elem_util >= max_utilization {
            max_utilization = elem_util;
            max_utilization_element = Some(beam.id);
        }

        let mut end_forces_arr = [0.0_f64; 12];
        let mut end_disp_arr = [0.0_f64; 12];
        end_forces_arr.copy_from_slice(f_int_local.as_slice());
        end_disp_arr.copy_from_slice(u_uncond_local.as_slice());

        element_results.insert(
            beam.id,
            ElementResult {
                element_id: beam.id,
                length: state.frame_def.length,
                ex: state.frame_def.ex(),
                ey: state.frame_def.ey(),
                ez: state.frame_def.ez(),
                end_forces_local: end_forces_arr,
                end_displacements_local: end_disp_arr,
                stations,
                max_utilization: elem_util,
                max_vm_utilization: elem_vm_util,
            },
        );
    }

    let mut shell_results = HashMap::with_capacity(prep.shells.len());
    for shell in &prep.shells {
        let (frame_def, u_def_local, _, _) =
            eval_corotational_shell(shell, u_global, node_rotations)?;
        let mut res = recover_shell_result(
            shell.id,
            &shell.frame0,
            shell.e,
            shell.g,
            shell.nu,
            shell.fy,
            shell.thickness,
            &u_def_local,
        )
        .map_err(|reason| SolverError::InvalidElement {
            element_id: shell.id,
            reason,
        })?;
        // Report true global centroid displacement magnitude
        let mut u_c = Vector3::<f64>::zeros();
        let inv_n = 1.0 / shell.node_indices.len() as f64;
        for &n_idx in &shell.node_indices {
            let b = n_idx * 6;
            u_c.x += u_global[b] * inv_n;
            u_c.y += u_global[b + 1] * inv_n;
            u_c.z += u_global[b + 2] * inv_n;
        }
        res.centroid_disp_mag_m = u_c.norm();
        res.normal = frame_def.normal();

        max_stress_pa = max_stress_pa.max(res.max_von_mises_pa);
        if res.utilization >= max_utilization {
            max_utilization = res.utilization;
        }
        shell_results.insert(shell.id, res);
    }

    Ok(Solution {
        node_results,
        element_results,
        shell_results,
        summary: AnalysisSummary {
            solve_time_ms,
            total_dofs: prep.total_dofs,
            free_dofs: prep.n_free,
            max_displacement_m,
            max_displacement_node,
            max_axial_n,
            min_axial_n,
            max_shear_n,
            max_moment_nm,
            max_torque_nm,
            max_stress_pa,
            max_utilization,
            max_utilization_element,
            force_equilibrium_residual_n,
            solver_relative_residual,
        },
    })
}

/// Solves a geometrically nonlinear (large-deformation 3D corotational) static analysis with default settings.
pub fn solve_nonlinear_static(model: &StructuralModel) -> Result<NonlinearSolution, SolverError> {
    solve_nonlinear_static_with_config(model, &NonlinearStaticConfig::default())
}

/// Solves a geometrically nonlinear (large-deformation 3D corotational) static analysis with incremental
/// load stepping, full Newton-Raphson iterations, and automatic substep bisection.
pub fn solve_nonlinear_static_with_config(
    model: &StructuralModel,
    config: &NonlinearStaticConfig,
) -> Result<NonlinearSolution, SolverError> {
    let t_start = Instant::now();
    if config.load_steps == 0 {
        return Err(SolverError::InvalidConfig(
            "load_steps must be at least 1.".to_string(),
        ));
    }
    if config.max_iterations_per_step == 0 {
        return Err(SolverError::InvalidConfig(
            "max_iterations_per_step must be at least 1.".to_string(),
        ));
    }

    let prep = prepare_model(model)?;
    let mut u_global = vec![0.0_f64; prep.total_dofs];
    let mut node_rotations = vec![Matrix3::<f64>::identity(); prep.n_nodes];

    let mut records = Vec::with_capacity(config.load_steps);
    let mut total_iterations = 0usize;
    let mut last_rel_res = 0.0_f64;

    let mut lambda_cur = 0.0_f64;
    let mut d_lambda = 1.0 / config.load_steps as f64;
    let mut bisections = 0usize;
    let mut step_counter = 0usize;

    while lambda_cur < 1.0 - 1e-12 {
        let lambda_target = (lambda_cur + d_lambda).min(1.0);
        let u_saved = u_global.clone();
        let r_saved = node_rotations.clone();

        apply_prescribed_at_lambda(model, &prep, &mut u_global, &mut node_rotations, lambda_target);

        let mut step_converged = false;
        let mut step_iters = 0usize;
        let mut rel_res = f64::INFINITY;

        // Compute target external force norm on free DOFs
        let mut f_ext_norm2 = 0.0_f64;
        for gdof in 0..prep.total_dofs {
            if prep.dof_to_free[gdof].is_some() {
                let fe = lambda_target * prep.f_ext_global[gdof];
                f_ext_norm2 += fe * fe;
            }
        }
        let f_ext_norm = f_ext_norm2.sqrt().max(1.0);

        for iter in 1..=config.max_iterations_per_step {
            step_iters = iter;
            let (f_int, triplets) = match assemble_corotational_system(&prep, &u_global, &node_rotations) {
                Ok(v) => v,
                Err(_) => {
                    step_converged = false;
                    break;
                }
            };

            let mut residual_free = vec![0.0_f64; prep.n_free];
            let mut res_norm2 = 0.0_f64;
            let mut f_int_all_norm2 = 0.0_f64;
            for gdof in 0..prep.total_dofs {
                f_int_all_norm2 += f_int[gdof] * f_int[gdof];
                if let Some(fr) = prep.dof_to_free[gdof] {
                    let r = lambda_target * prep.f_ext_global[gdof] - f_int[gdof];
                    residual_free[fr] = r;
                    res_norm2 += r * r;
                }
            }
            let f_ref = if f_ext_norm2 > 0.0 {
                f_ext_norm
            } else {
                f_int_all_norm2.sqrt().max(1.0)
            };
            rel_res = res_norm2.sqrt() / f_ref;

            if rel_res <= config.force_tol && (iter > 1 || f_ext_norm2 == 0.0) {
                step_converged = true;
                break;
            }

            let du_free = match solve_sparse_system(prep.n_free, &triplets, &residual_free) {
                Ok(v) => v,
                Err(_) => {
                    step_converged = false;
                    break;
                }
            };

            let du_norm = du_free.iter().map(|v| v * v).sum::<f64>().sqrt();
            apply_increment(&prep, &mut u_global, &mut node_rotations, &du_free);

            let u_norm = u_global.iter().map(|v| v * v).sum::<f64>().sqrt().max(1.0);
            let rel_du = du_norm / u_norm;

            // Check convergence after update if force residual was already small
            if rel_res <= config.force_tol && rel_du <= config.disp_tol {
                step_converged = true;
                break;
            }
            if !du_norm.is_finite() || du_norm > 1e6 {
                step_converged = false;
                break;
            }
        }

        // Final residual verification if converged on last iter
        if step_converged {
            step_counter += 1;
            total_iterations += step_iters;
            last_rel_res = rel_res;
            lambda_cur = lambda_target;

            let mut max_u = 0.0_f64;
            for n_idx in 0..prep.n_nodes {
                let b = n_idx * 6;
                let um = (u_global[b].powi(2) + u_global[b + 1].powi(2) + u_global[b + 2].powi(2))
                    .sqrt();
                if um > max_u {
                    max_u = um;
                }
            }

            records.push(LoadStepRecord {
                step: step_counter,
                load_factor: lambda_cur,
                max_displacement_m: max_u,
                iterations: step_iters,
                relative_residual: rel_res,
            });

            // Gently grow step size back if we had bisected earlier
            if bisections > 0 && step_iters <= 6 {
                d_lambda = (d_lambda * 1.5).min(1.0 / config.load_steps as f64);
            }
        } else {
            // Rollback and bisect load increment
            u_global = u_saved;
            node_rotations = r_saved;
            if bisections >= config.max_bisections {
                return Err(SolverError::ConvergenceFailed {
                    step: step_counter + 1,
                    residual: rel_res,
                });
            }
            bisections += 1;
            d_lambda *= 0.5;
        }
    }

    let solve_time_ms = t_start.elapsed().as_secs_f64() * 1000.0;
    let solution = recover_corotational_solution(
        model,
        &prep,
        &u_global,
        &node_rotations,
        1.0,
        config.stations_per_element,
        solve_time_ms,
        last_rel_res,
    )?;

    Ok(NonlinearSolution {
        solution,
        load_steps: records,
        total_iterations,
        converged: true,
    })
}

/// Helper to build `StationSample` vectors for linear displacements (used by modal / linear dynamic recovery).
pub(crate) fn recover_linear_solution_from_u(
    model: &StructuralModel,
    prep: &PreparedModel,
    u_global: &[f64],
    f_ext_scale: f64,
    stations_per_element: usize,
    solve_time_ms: f64,
) -> Result<Solution, SolverError> {
    let mut ku_global = vec![0.0_f64; prep.total_dofs];

    for beam in &prep.beams {
        let t_mat = beam.frame0.transformation_12x12();
        let k_glob = t_mat.transpose() * beam.k_cond_local * t_mat;
        let mut u_elem = SVector::<f64, 12>::zeros();
        let mut gdofs = [0usize; 12];
        for d in 0..6 {
            gdofs[d] = beam.i_idx * 6 + d;
            gdofs[6 + d] = beam.j_idx * 6 + d;
        }
        for a in 0..12 {
            u_elem[a] = u_global[gdofs[a]];
        }
        let f_elem = k_glob * u_elem;
        for a in 0..12 {
            ku_global[gdofs[a]] += f_elem[a];
        }
    }

    for shell in &prep.shells {
        let t_mat = shell.frame0.transformation_matrix();
        let k_glob = t_mat.transpose() * &shell.k_local0 * &t_mat;
        let ndof = shell.node_indices.len() * 6;
        let mut u_sh = DVector::<f64>::zeros(ndof);
        let mut gdofs = Vec::with_capacity(ndof);
        for (k, &n_idx) in shell.node_indices.iter().enumerate() {
            for d in 0..6 {
                let g = n_idx * 6 + d;
                gdofs.push(g);
                u_sh[k * 6 + d] = u_global[g];
            }
        }
        let f_sh = k_glob * u_sh;
        for a in 0..ndof {
            ku_global[gdofs[a]] += f_sh[a];
        }
    }

    let mut node_results = HashMap::with_capacity(prep.n_nodes);
    let mut total_reaction_force = [0.0_f64; 3];
    let mut max_displacement_m = 0.0_f64;
    let mut max_displacement_node = None;

    for (n_idx, node) in model.nodes.iter().enumerate() {
        let base = n_idx * 6;
        let mut disp = [0.0_f64; 6];
        let mut reac = [0.0_f64; 6];
        for d in 0..6 {
            disp[d] = u_global[base + d];
            if node.restraints[d] {
                reac[d] = ku_global[base + d] - f_ext_scale * prep.f_ext_global[base + d];
                if d < 3 {
                    total_reaction_force[d] += reac[d];
                }
            }
        }
        let u_mag = (disp[0].powi(2) + disp[1].powi(2) + disp[2].powi(2)).sqrt();
        if u_mag >= max_displacement_m {
            max_displacement_m = u_mag;
            max_displacement_node = Some(node.id);
        }
        node_results.insert(
            node.id,
            NodeResult {
                node_id: node.id,
                displacements: disp,
                reactions: reac,
            },
        );
    }

    let eq_res = [
        f_ext_scale * prep.total_applied_force[0] + total_reaction_force[0],
        f_ext_scale * prep.total_applied_force[1] + total_reaction_force[1],
        f_ext_scale * prep.total_applied_force[2] + total_reaction_force[2],
    ];
    let force_equilibrium_residual_n =
        (eq_res[0].powi(2) + eq_res[1].powi(2) + eq_res[2].powi(2)).sqrt();

    let mut element_results = HashMap::with_capacity(prep.beams.len());
    let mut max_axial_n = 0.0_f64;
    let mut min_axial_n = 0.0_f64;
    let mut max_shear_n = 0.0_f64;
    let mut max_moment_nm = 0.0_f64;
    let mut max_torque_nm = 0.0_f64;
    let mut max_stress_pa = 0.0_f64;
    let mut max_utilization = 0.0_f64;
    let mut max_utilization_element = None;

    for beam in &prep.beams {
        let mut u_elem_global = SVector::<f64, 12>::zeros();
        for d in 0..6 {
            u_elem_global[d] = u_global[beam.i_idx * 6 + d];
            u_elem_global[6 + d] = u_global[beam.j_idx * 6 + d];
        }
        let t_mat = beam.frame0.transformation_12x12();
        let u_nodal_local = t_mat * u_elem_global;
        let f_fef_scaled = beam.f_fef_uncond_local * f_ext_scale;
        let u_uncond_local = recover_uncondensed_local_displacements(
            &u_nodal_local,
            &beam.k_uncond_local,
            &f_fef_scaled,
            &beam.releases,
        );
        let f_int_local = beam.k_uncond_local * u_uncond_local + f_fef_scaled;

        let q_s = [
            f_ext_scale * beam.q_local_start[0],
            f_ext_scale * beam.q_local_start[1],
            f_ext_scale * beam.q_local_start[2],
        ];
        let q_e = [
            f_ext_scale * beam.q_local_end[0],
            f_ext_scale * beam.q_local_end[1],
            f_ext_scale * beam.q_local_end[2],
        ];
        let stations: Vec<StationSample> = sample_element_stations(
            beam.p_i,
            &beam.frame0,
            beam.e,
            beam.g,
            &beam.props,
            beam.formulation,
            &u_uncond_local,
            &f_int_local,
            q_s,
            q_e,
            stations_per_element,
        );

        let mut elem_max_sigma = 0.0_f64;
        let mut elem_max_vm = 0.0_f64;
        for s in &stations {
            let disp_mag = (s.disp_global[0].powi(2)
                + s.disp_global[1].powi(2)
                + s.disp_global[2].powi(2))
            .sqrt();
            if disp_mag > max_displacement_m {
                max_displacement_m = disp_mag;
            }
            max_axial_n = max_axial_n.max(s.axial_n);
            min_axial_n = min_axial_n.min(s.axial_n);
            max_shear_n = max_shear_n.max(s.shear_vy.abs()).max(s.shear_vz.abs());
            max_moment_nm = max_moment_nm.max(s.moment_my.abs()).max(s.moment_mz.abs());
            max_torque_nm = max_torque_nm.max(s.torque_t.abs());
            max_stress_pa = max_stress_pa.max(s.max_normal_stress);
            elem_max_sigma = elem_max_sigma.max(s.max_normal_stress);
            elem_max_vm = elem_max_vm.max(s.von_mises_stress);
        }

        let elem_util = elem_max_sigma / beam.fy;
        let elem_vm_util = elem_max_vm / beam.fy;
        if elem_util >= max_utilization {
            max_utilization = elem_util;
            max_utilization_element = Some(beam.id);
        }

        let mut end_forces_arr = [0.0_f64; 12];
        let mut end_disp_arr = [0.0_f64; 12];
        end_forces_arr.copy_from_slice(f_int_local.as_slice());
        end_disp_arr.copy_from_slice(u_uncond_local.as_slice());

        element_results.insert(
            beam.id,
            ElementResult {
                element_id: beam.id,
                length: beam.frame0.length,
                ex: beam.frame0.ex(),
                ey: beam.frame0.ey(),
                ez: beam.frame0.ez(),
                end_forces_local: end_forces_arr,
                end_displacements_local: end_disp_arr,
                stations,
                max_utilization: elem_util,
                max_vm_utilization: elem_vm_util,
            },
        );
    }

    let mut shell_results = HashMap::with_capacity(prep.shells.len());
    for shell in &prep.shells {
        let ndof = shell.node_indices.len() * 6;
        let mut u_sh_g = DVector::<f64>::zeros(ndof);
        for (k, &n_idx) in shell.node_indices.iter().enumerate() {
            for d in 0..6 {
                u_sh_g[k * 6 + d] = u_global[n_idx * 6 + d];
            }
        }
        let u_sh_l = shell.frame0.transformation_matrix() * u_sh_g;
        let res = recover_shell_result(
            shell.id,
            &shell.frame0,
            shell.e,
            shell.g,
            shell.nu,
            shell.fy,
            shell.thickness,
            &u_sh_l,
        )
        .map_err(|reason| SolverError::InvalidElement {
            element_id: shell.id,
            reason,
        })?;
        max_stress_pa = max_stress_pa.max(res.max_von_mises_pa);
        if res.utilization >= max_utilization {
            max_utilization = res.utilization;
        }
        shell_results.insert(shell.id, res);
    }

    Ok(Solution {
        node_results,
        element_results,
        shell_results,
        summary: AnalysisSummary {
            solve_time_ms,
            total_dofs: prep.total_dofs,
            free_dofs: prep.n_free,
            max_displacement_m,
            max_displacement_node,
            max_axial_n,
            min_axial_n,
            max_shear_n,
            max_moment_nm,
            max_torque_nm,
            max_stress_pa,
            max_utilization,
            max_utilization_element,
            force_equilibrium_residual_n,
            solver_relative_residual: 0.0,
        },
    })
}
