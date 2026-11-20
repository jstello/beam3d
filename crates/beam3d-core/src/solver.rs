use crate::element::{
    ElementFrame, StationSample, condense_element_releases, consistent_equivalent_local_loads,
    recover_uncondensed_local_displacements, sample_element_stations, total_local_distributed_load,
    uncondensed_local_stiffness,
};
use crate::model::StructuralModel;
use crate::shell::{
    ShellFrame, ShellResult, consistent_global_surface_forces, recover_shell_result,
    shell_local_stiffness, total_global_surface_traction,
};
use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, Triplet};
use faer::{Mat, Side};
use nalgebra::{DMatrix, DVector, SVector};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Instant;
use thiserror::Error;

#[derive(Debug, Error, Clone)]
pub enum SolverError {
    #[error("Model has no beam or shell elements.")]
    NoElements,
    #[error("Model has no nodes.")]
    NoNodes,
    #[error("Model has no restrained DOFs (supports); structure can move as a rigid body.")]
    NoSupports,
    #[error("Element {element_id} references missing node {node_id}.")]
    MissingNode { element_id: usize, node_id: usize },
    #[error("Element {element_id} references missing material {material_id}.")]
    MissingMaterial {
        element_id: usize,
        material_id: usize,
    },
    #[error("Element {element_id} references missing cross-section {section_id}.")]
    MissingSection {
        element_id: usize,
        section_id: usize,
    },
    #[error("Element {element_id}: {reason}")]
    InvalidElement { element_id: usize, reason: String },
    #[error("Unconnected / orphan node '{label}' (ID {node_id}) has unrestrained DOFs.")]
    OrphanNode { node_id: usize, label: String },
    #[error(
        "Global stiffness matrix is singular or structure is a mechanism (check supports and member end releases)."
    )]
    SingularSystem,
    #[error("Sparse matrix assembly failed: {0}")]
    SparseAssembly(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeResult {
    pub node_id: usize,
    /// Global displacements `[ux, uy, uz]` (m) and rotations `[rx, ry, rz]` (rad).
    pub displacements: [f64; 6],
    /// Support reaction forces `[Rx, Ry, Rz]` (N) and moments `[Mx, My, Mz]` (N·m) in global coordinates.
    /// Non-zero only on restrained DOFs.
    pub reactions: [f64; 6],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElementResult {
    pub element_id: usize,
    pub length: f64,
    pub ex: [f64; 3],
    pub ey: [f64; 3],
    pub ez: [f64; 3],
    /// Uncondensed local member end forces `[Fx1, Fy1, Fz1, Mx1, My1, Mz1, Fx2, Fy2, Fz2, Mx2, My2, Mz2]`.
    pub end_forces_local: [f64; 12],
    /// Uncondensed local member end displacements (including hinge rotations).
    pub end_displacements_local: [f64; 12],
    /// Sampled interior stations for deflected shape, force diagrams, and stresses.
    pub stations: Vec<StationSample>,
    /// Peak combined normal stress utilization `max_sigma / fy`.
    pub max_utilization: f64,
    /// Peak von Mises stress utilization `max_vm / fy`.
    pub max_vm_utilization: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisSummary {
    pub solve_time_ms: f64,
    pub total_dofs: usize,
    pub free_dofs: usize,
    pub max_displacement_m: f64,
    pub max_displacement_node: Option<usize>,
    pub max_axial_n: f64,
    pub min_axial_n: f64,
    pub max_shear_n: f64,
    pub max_moment_nm: f64,
    pub max_torque_nm: f64,
    pub max_stress_pa: f64,
    pub max_utilization: f64,
    pub max_utilization_element: Option<usize>,
    /// Global force equilibrium residual norm `[Fx, Fy, Fz]` in N.
    pub force_equilibrium_residual_n: f64,
    /// Relative linear solver residual `||K_ff u_f - rhs|| / max(||rhs||, 1)`.
    pub solver_relative_residual: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Solution {
    pub node_results: HashMap<usize, NodeResult>,
    pub element_results: HashMap<usize, ElementResult>,
    #[serde(default)]
    pub shell_results: HashMap<usize, ShellResult>,
    pub summary: AnalysisSummary,
}

pub struct SolverConfig {
    pub stations_per_element: usize,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            stations_per_element: 25,
        }
    }
}

pub fn solve_linear_static(model: &StructuralModel) -> Result<Solution, SolverError> {
    solve_linear_static_with_config(model, &SolverConfig::default())
}

pub fn solve_linear_static_with_config(
    model: &StructuralModel,
    config: &SolverConfig,
) -> Result<Solution, SolverError> {
    let t_start = Instant::now();

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

    // Check orphan nodes that have unrestrained DOFs
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

    // Partition DOFs into free vs prescribed
    let mut dof_to_free: Vec<Option<usize>> = vec![None; total_dofs];
    let mut u_global = vec![0.0_f64; total_dofs];
    let mut n_free = 0usize;
    let mut n_restrained = 0usize;

    for (n_idx, node) in model.nodes.iter().enumerate() {
        for d in 0..6 {
            let gdof = n_idx * 6 + d;
            if node.restraints[d] {
                u_global[gdof] = node.prescribed[d];
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

    // Pre-collect distributed loads per element
    let mut elem_dloads: HashMap<usize, Vec<_>> = HashMap::new();
    for dl in &model.distributed_loads {
        elem_dloads
            .entry(dl.element_id)
            .or_default()
            .push((dl.coord_system, dl.q_start, dl.q_end));
    }

    // Assemble external equivalent nodal load vector F_ext (global)
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

    // Cache element assembly artifacts so post-processing doesn't recompute everything
    struct CachedElement {
        id: usize,
        i_idx: usize,
        j_idx: usize,
        p_i: [f64; 3],
        frame: ElementFrame,
        e: f64,
        g: f64,
        fy: f64,
        props: crate::section::SectionProperties,
        formulation: crate::model::BeamFormulation,
        releases: [bool; 12],
        k_uncond_local: nalgebra::SMatrix<f64, 12, 12>,
        f_fef_uncond_local: SVector<f64, 12>,
        q_local_start: [f64; 3],
        q_local_end: [f64; 3],
        k_global: nalgebra::SMatrix<f64, 12, 12>,
    }

    let mut cached_elements = Vec::with_capacity(model.elements.len());
    let mut triplets_ff = Vec::with_capacity(model.elements.len() * 144);
    let mut rhs_free = vec![0.0_f64; n_free];

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

        let frame = ElementFrame::compute(p_i, p_j, elem.roll_angle_deg).map_err(|reason| {
            SolverError::InvalidElement {
                element_id: elem.id,
                reason,
            }
        })?;

        let k_uncond_local =
            uncondensed_local_stiffness(mat.e, mat.g, &props, frame.length, elem.formulation);

        let dloads_slice = elem_dloads
            .get(&elem.id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        let (q_local_start, q_local_end) = total_local_distributed_load(
            &frame,
            mat.density,
            props.area,
            &model.gravity,
            dloads_slice,
        );

        // Track total global applied distributed force for equilibrium check
        let q_avg_local = nalgebra::Vector3::new(
            0.5 * (q_local_start[0] + q_local_end[0]),
            0.5 * (q_local_start[1] + q_local_end[1]),
            0.5 * (q_local_start[2] + q_local_end[2]),
        );
        let f_elem_global = frame.r_mat.transpose() * (q_avg_local * frame.length);
        for d in 0..3 {
            total_applied_force[d] += f_elem_global[d];
        }

        let f_eq_uncond_local = consistent_equivalent_local_loads(
            q_local_start,
            q_local_end,
            mat.e,
            mat.g,
            &props,
            frame.length,
            elem.formulation,
        );
        let f_fef_uncond_local = -f_eq_uncond_local;

        let mut releases = [false; 12];
        releases[..6].copy_from_slice(&elem.releases_i);
        releases[6..].copy_from_slice(&elem.releases_j);

        let (k_cond_local, f_fef_cond_local) =
            condense_element_releases(&k_uncond_local, &f_fef_uncond_local, &releases);

        let t_mat = frame.transformation_12x12();
        let t_tr = t_mat.transpose();
        let k_global = t_tr * k_cond_local * t_mat;
        let f_eq_cond_global = t_tr * (-f_fef_cond_local);

        let mut elem_gdofs = [0usize; 12];
        for d in 0..6 {
            elem_gdofs[d] = i_idx * 6 + d;
            elem_gdofs[6 + d] = j_idx * 6 + d;
        }

        for a in 0..12 {
            f_ext_global[elem_gdofs[a]] += f_eq_cond_global[a];
        }

        // Scatter K_global into K_ff triplets and -K_fp * u_p into rhs_free
        for r in 0..12 {
            let gr = elem_gdofs[r];
            if let Some(fr) = dof_to_free[gr] {
                for c in 0..12 {
                    let val = k_global[(r, c)];
                    if val.abs() == 0.0 {
                        continue;
                    }
                    let gc = elem_gdofs[c];
                    if let Some(fc) = dof_to_free[gc] {
                        triplets_ff.push(Triplet::new(fr, fc, val));
                    } else {
                        rhs_free[fr] -= val * u_global[gc];
                    }
                }
            }
        }

        cached_elements.push(CachedElement {
            id: elem.id,
            i_idx,
            j_idx,
            p_i,
            frame,
            e: mat.e,
            g: mat.g,
            fy: mat.yield_strength.max(1.0),
            props,
            formulation: elem.formulation,
            releases,
            k_uncond_local,
            f_fef_uncond_local,
            q_local_start,
            q_local_end,
            k_global,
        });
    }

    // Pre-collect 2D surface loads per shell
    let mut shell_sloads: HashMap<usize, Vec<_>> = HashMap::new();
    for sl in &model.surface_loads {
        shell_sloads
            .entry(sl.shell_id)
            .or_default()
            .push((sl.coord_system, sl.pressure));
    }

    struct CachedShell {
        id: usize,
        node_indices: Vec<usize>,
        frame: ShellFrame,
        e: f64,
        g: f64,
        nu: f64,
        fy: f64,
        thickness: f64,
        t_mat: DMatrix<f64>,
        k_global: DMatrix<f64>,
    }

    let mut cached_shells = Vec::with_capacity(model.shells.len());
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

        let frame = ShellFrame::compute(&positions).map_err(|reason| SolverError::InvalidElement {
            element_id: shell.id,
            reason,
        })?;
        let k_local = shell_local_stiffness(&frame, mat.e, mat.g, nu, shell.thickness).map_err(
            |reason| SolverError::InvalidElement {
                element_id: shell.id,
                reason,
            },
        )?;
        let t_mat = frame.transformation_matrix();
        let k_global = t_mat.transpose() * &k_local * &t_mat;

        let sloads_slice = shell_sloads
            .get(&shell.id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        let p_global = total_global_surface_traction(
            &frame,
            mat.density,
            shell.thickness,
            &model.gravity,
            sloads_slice,
        );
        for d in 0..3 {
            total_applied_force[d] += p_global[d] * frame.area;
        }

        let f_eq_global = consistent_global_surface_forces(&frame, p_global).map_err(|reason| {
            SolverError::InvalidElement {
                element_id: shell.id,
                reason,
            }
        })?;

        let ndof = node_indices.len() * 6;
        let mut shell_gdofs = Vec::with_capacity(ndof);
        for &n_idx in &node_indices {
            for d in 0..6 {
                shell_gdofs.push(n_idx * 6 + d);
            }
        }

        for a in 0..ndof {
            f_ext_global[shell_gdofs[a]] += f_eq_global[a];
        }

        for r in 0..ndof {
            let gr = shell_gdofs[r];
            if let Some(fr) = dof_to_free[gr] {
                for c in 0..ndof {
                    let val = k_global[(r, c)];
                    if val.abs() == 0.0 {
                        continue;
                    }
                    let gc = shell_gdofs[c];
                    if let Some(fc) = dof_to_free[gc] {
                        triplets_ff.push(Triplet::new(fr, fc, val));
                    } else {
                        rhs_free[fr] -= val * u_global[gc];
                    }
                }
            }
        }

        cached_shells.push(CachedShell {
            id: shell.id,
            node_indices,
            frame,
            e: mat.e,
            g: mat.g,
            nu,
            fy: mat.yield_strength.max(1.0),
            thickness: shell.thickness,
            t_mat,
            k_global,
        });
    }

    for gdof in 0..total_dofs {
        if let Some(fr) = dof_to_free[gdof] {
            rhs_free[fr] += f_ext_global[gdof];
        }
    }

    let mut solver_relative_residual = 0.0;

    if n_free > 0 {
        let k_ff = SparseColMat::<usize, f64>::try_new_from_triplets(n_free, n_free, &triplets_ff)
            .map_err(|e| SolverError::SparseAssembly(format!("{e:?}")))?;

        // Quick zero-diagonal check for mechanisms (e.g., all elements meeting at a node have hinges)
        let mut diag = vec![0.0_f64; n_free];
        for t in &triplets_ff {
            if t.row == t.col {
                diag[t.row] += t.val;
            }
        }
        if diag.iter().any(|&d| d <= 1e-12) {
            return Err(SolverError::SingularSystem);
        }

        let rhs_mat = Mat::from_fn(n_free, 1, |i, _| rhs_free[i]);

        let sol_mat = match k_ff.sp_cholesky(Side::Lower) {
            Ok(llt) => llt.solve(&rhs_mat),
            Err(_) => {
                let lu = k_ff.sp_lu().map_err(|_| SolverError::SingularSystem)?;
                lu.solve(&rhs_mat)
            }
        };

        for i in 0..n_free {
            if !sol_mat[(i, 0)].is_finite() {
                return Err(SolverError::SingularSystem);
            }
        }

        // Verify linear system residual ||K_ff * u - rhs|| / max(||rhs||, 1)
        let ku = k_ff.as_ref() * sol_mat.as_ref();
        let mut res_norm2 = 0.0_f64;
        let mut rhs_norm2 = 0.0_f64;
        for i in 0..n_free {
            let diff = ku[(i, 0)] - rhs_mat[(i, 0)];
            res_norm2 += diff * diff;
            rhs_norm2 += rhs_mat[(i, 0)] * rhs_mat[(i, 0)];
        }
        solver_relative_residual = res_norm2.sqrt() / rhs_norm2.sqrt().max(1.0);
        if solver_relative_residual > 1e-4 {
            return Err(SolverError::SingularSystem);
        }

        for gdof in 0..total_dofs {
            if let Some(fr) = dof_to_free[gdof] {
                u_global[gdof] = sol_mat[(fr, 0)];
            }
        }
    }

    // Recover global reactions R = K * u - F_ext
    let mut ku_global = vec![0.0_f64; total_dofs];
    for ce in &cached_elements {
        let mut elem_gdofs = [0usize; 12];
        let mut u_elem = SVector::<f64, 12>::zeros();
        for d in 0..6 {
            elem_gdofs[d] = ce.i_idx * 6 + d;
            elem_gdofs[6 + d] = ce.j_idx * 6 + d;
        }
        for a in 0..12 {
            u_elem[a] = u_global[elem_gdofs[a]];
        }
        let f_elem = ce.k_global * u_elem;
        for a in 0..12 {
            ku_global[elem_gdofs[a]] += f_elem[a];
        }
    }
    for cs in &cached_shells {
        let ndof = cs.node_indices.len() * 6;
        let mut shell_gdofs = Vec::with_capacity(ndof);
        let mut u_shell = DVector::<f64>::zeros(ndof);
        for (k, &n_idx) in cs.node_indices.iter().enumerate() {
            for d in 0..6 {
                let gdof = n_idx * 6 + d;
                shell_gdofs.push(gdof);
                u_shell[k * 6 + d] = u_global[gdof];
            }
        }
        let f_shell = &cs.k_global * u_shell;
        for a in 0..ndof {
            ku_global[shell_gdofs[a]] += f_shell[a];
        }
    }

    let mut node_results = HashMap::with_capacity(n_nodes);
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
                reac[d] = ku_global[base + d] - f_ext_global[base + d];
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
        total_applied_force[0] + total_reaction_force[0],
        total_applied_force[1] + total_reaction_force[1],
        total_applied_force[2] + total_reaction_force[2],
    ];
    let force_equilibrium_residual_n =
        (eq_res[0] * eq_res[0] + eq_res[1] * eq_res[1] + eq_res[2] * eq_res[2]).sqrt();

    // Post-process each element: uncondensed displacements, end forces, and interior stations
    let mut element_results = HashMap::with_capacity(cached_elements.len());
    let mut max_axial_n = 0.0_f64;
    let mut min_axial_n = 0.0_f64;
    let mut max_shear_n = 0.0_f64;
    let mut max_moment_nm = 0.0_f64;
    let mut max_torque_nm = 0.0_f64;
    let mut max_stress_pa = 0.0_f64;
    let mut max_utilization = 0.0_f64;
    let mut max_utilization_element = None;

    for ce in &cached_elements {
        let mut u_elem_global = SVector::<f64, 12>::zeros();
        for d in 0..6 {
            u_elem_global[d] = u_global[ce.i_idx * 6 + d];
            u_elem_global[6 + d] = u_global[ce.j_idx * 6 + d];
        }
        let t_mat = ce.frame.transformation_12x12();
        let u_nodal_local = t_mat * u_elem_global;
        let u_uncond_local = recover_uncondensed_local_displacements(
            &u_nodal_local,
            &ce.k_uncond_local,
            &ce.f_fef_uncond_local,
            &ce.releases,
        );
        let f_int_local = ce.k_uncond_local * u_uncond_local + ce.f_fef_uncond_local;

        let stations = sample_element_stations(
            ce.p_i,
            &ce.frame,
            ce.e,
            ce.g,
            &ce.props,
            ce.formulation,
            &u_uncond_local,
            &f_int_local,
            ce.q_local_start,
            ce.q_local_end,
            config.stations_per_element,
        );

        let mut elem_max_sigma = 0.0_f64;
        let mut elem_max_vm = 0.0_f64;

        for s in &stations {
            let disp_mag = (s.disp_global[0] * s.disp_global[0]
                + s.disp_global[1] * s.disp_global[1]
                + s.disp_global[2] * s.disp_global[2])
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

        let elem_util = elem_max_sigma / ce.fy;
        let elem_vm_util = elem_max_vm / ce.fy;
        if elem_util >= max_utilization {
            max_utilization = elem_util;
            max_utilization_element = Some(ce.id);
        }

        let mut end_forces_arr = [0.0_f64; 12];
        let mut end_disp_arr = [0.0_f64; 12];
        end_forces_arr.copy_from_slice(f_int_local.as_slice());
        end_disp_arr.copy_from_slice(u_uncond_local.as_slice());

        element_results.insert(
            ce.id,
            ElementResult {
                element_id: ce.id,
                length: ce.frame.length,
                ex: ce.frame.ex(),
                ey: ce.frame.ey(),
                ez: ce.frame.ez(),
                end_forces_local: end_forces_arr,
                end_displacements_local: end_disp_arr,
                stations,
                max_utilization: elem_util,
                max_vm_utilization: elem_vm_util,
            },
        );
    }

    let mut shell_results = HashMap::with_capacity(cached_shells.len());
    for cs in &cached_shells {
        let ndof = cs.node_indices.len() * 6;
        let mut u_shell_global = DVector::<f64>::zeros(ndof);
        for (k, &n_idx) in cs.node_indices.iter().enumerate() {
            for d in 0..6 {
                u_shell_global[k * 6 + d] = u_global[n_idx * 6 + d];
            }
        }
        let u_shell_local = &cs.t_mat * u_shell_global;
        let res = recover_shell_result(
            cs.id,
            &cs.frame,
            cs.e,
            cs.g,
            cs.nu,
            cs.fy,
            cs.thickness,
            &u_shell_local,
        )
        .map_err(|reason| SolverError::InvalidElement {
            element_id: cs.id,
            reason,
        })?;

        max_stress_pa = max_stress_pa.max(res.max_von_mises_pa);
        if res.utilization >= max_utilization {
            max_utilization = res.utilization;
        }
        shell_results.insert(cs.id, res);
    }

    let solve_time_ms = t_start.elapsed().as_secs_f64() * 1000.0;

    Ok(Solution {
        node_results,
        element_results,
        shell_results,
        summary: AnalysisSummary {
            solve_time_ms,
            total_dofs,
            free_dofs: n_free,
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
