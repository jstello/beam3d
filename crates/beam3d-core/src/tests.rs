use crate::{
    BeamFormulation, CrossSection, DistributedLoad, DynamicConfig, LoadCoordSystem,
    MassFormulation, Material, ModalConfig, NodalLoad, NonlinearStaticConfig, RayleighDamping,
    SectionProfile, SolverError, StructuralModel, TemplateKind, TimeHistoryExcitation,
    solve_dynamic_with_config, solve_linear_static, solve_modal_with_config,
    solve_nonlinear_static, solve_nonlinear_static_with_config,
};

fn assert_rel_close(actual: f64, expected: f64, rel_tol: f64, label: &str) {
    let scale = expected.abs().max(1.0);
    let err = (actual - expected).abs() / scale;
    assert!(
        err <= rel_tol,
        "{label}: actual = {actual:.9e}, expected = {expected:.9e}, rel_err = {err:.3e}"
    );
}

#[test]
fn test_cantilever_euler_bernoulli_and_timoshenko() {
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Test I-Beam".to_string(),
        profile: SectionProfile::IBeam {
            depth: 0.400,
            width: 0.200,
            web_thickness: 0.010,
            flange_thickness: 0.016,
        },
    }];

    let l = 3.2;
    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([l, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    let b1 = m.add_beam(n1, n2, 1, 1).unwrap();

    let px = 50_000.0;
    let py = 30_000.0;
    let pz = -80_000.0;
    let mx = 12_000.0;
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n2,
        forces: [px, py, pz],
        moments: [mx, 0.0, 0.0],
    });

    let props = m.sections[0].properties();
    let e = m.materials[0].e;
    let g = m.materials[0].g;

    // 1. Euler-Bernoulli check
    m.get_element_mut(b1).unwrap().formulation = BeamFormulation::EulerBernoulli;
    let sol_eb = solve_linear_static(&m).expect("EB cantilever solve");
    let d_eb = sol_eb.node_results[&n2].displacements;

    assert_rel_close(d_eb[0], px * l / (e * props.area), 1e-11, "EB ux");
    assert_rel_close(d_eb[1], py * l.powi(3) / (3.0 * e * props.iz), 1e-11, "EB uy");
    assert_rel_close(d_eb[2], pz * l.powi(3) / (3.0 * e * props.iy), 1e-11, "EB uz");
    assert_rel_close(d_eb[3], mx * l / (g * props.j), 1e-11, "EB rx");
    assert_rel_close(
        d_eb[4],
        -pz * l.powi(2) / (2.0 * e * props.iy),
        1e-11,
        "EB ry",
    );
    assert_rel_close(
        d_eb[5],
        py * l.powi(2) / (2.0 * e * props.iz),
        1e-11,
        "EB rz",
    );

    // 2. Timoshenko check (includes exact shear deformation!)
    m.get_element_mut(b1).unwrap().formulation = BeamFormulation::Timoshenko;
    let sol_timo = solve_linear_static(&m).expect("Timoshenko cantilever solve");
    let d_timo = sol_timo.node_results[&n2].displacements;

    let expected_uy = py * l.powi(3) / (3.0 * e * props.iz) + py * l / (g * props.asy);
    let expected_uz = pz * l.powi(3) / (3.0 * e * props.iy) + pz * l / (g * props.asz);
    assert_rel_close(d_timo[1], expected_uy, 1e-11, "Timoshenko uy");
    assert_rel_close(d_timo[2], expected_uz, 1e-11, "Timoshenko uz");

    // Verify interior station endpoint matches nodal displacement
    let last_station = sol_timo.element_results[&b1].stations.last().unwrap();
    assert_rel_close(last_station.disp_global[1], expected_uy, 1e-11, "Station tip uy");
    assert_rel_close(last_station.disp_global[2], expected_uz, 1e-11, "Station tip uz");
}

#[test]
fn test_simply_supported_uniform_and_subdivided_mesh() {
    let mut m = StructuralModel::default();
    let l = 6.0;
    let w = 15_000.0; // downward uniform load magnitude (N/m)

    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([l, 0.0, 0.0]);
    // Simply supported in 3D: restrain ux,uy,uz,rx at left; uy,uz at right
    m.get_node_mut(n1).unwrap().restraints = [true, true, true, true, false, false];
    m.get_node_mut(n2).unwrap().restraints = [false, true, true, false, false, false];

    let b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    m.distributed_loads.push(DistributedLoad {
        id: 1,
        element_id: b1,
        coord_system: LoadCoordSystem::Global,
        q_start: [0.0, 0.0, -w],
        q_end: [0.0, 0.0, -w],
    });

    let props = m.sections[0].properties();
    let e = m.materials[0].e;
    let g = m.materials[0].g;

    // Solve on a SINGLE 2-node beam element!
    let sol_1 = solve_linear_static(&m).expect("1-element SS beam");
    let stations = &sol_1.element_results[&b1].stations;
    let mid = &stations[stations.len() / 2]; // xi = 0.5 since 25 stations -> index 12 is xi=0.5!

    let expected_mid_uz =
        -(5.0 * w * l.powi(4)) / (384.0 * e * props.iy) - (w * l.powi(2)) / (8.0 * g * props.asz);
    let expected_mid_my = w * l.powi(2) / 8.0;

    assert_rel_close(mid.disp_global[2], expected_mid_uz, 1e-10, "1-elem midspan uz");
    assert_rel_close(mid.moment_my, expected_mid_my, 1e-10, "1-elem midspan My");

    // Subdivide into 6 elements and verify midspan node matches 1-element interior recovery!
    m.subdivide_element(b1, 6);
    let sol_6 = solve_linear_static(&m).expect("6-element SS beam");
    assert_rel_close(
        sol_6.summary.max_displacement_m,
        expected_mid_uz.abs(),
        1e-10,
        "6-elem max displacement",
    );
    assert_rel_close(
        sol_6.summary.max_moment_nm,
        expected_mid_my,
        1e-10,
        "6-elem max moment",
    );
}

#[test]
fn test_propped_cantilever_member_hinge_condensation() {
    // Fixed at Both Ends structurally, EXCEPT beam has a moment hinge (ry released) at node 2!
    // Equivalent to a propped cantilever under uniform load w:
    // Analytical: R1 = 5/8 wL, R2 = 3/8 wL, M_fixed = -1/8 wL^2, M_hinge = 0 (for Euler-Bernoulli).
    let mut m = StructuralModel::default();
    let l = 5.0;
    let w = 20_000.0;

    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([l, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    m.get_node_mut(n2).unwrap().restraints = [true; 6];

    let b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    {
        let el = m.get_element_mut(b1).unwrap();
        el.formulation = BeamFormulation::EulerBernoulli;
        el.releases_j = [false, false, false, false, true, false]; // release ry at node j!
    }
    m.distributed_loads.push(DistributedLoad {
        id: 1,
        element_id: b1,
        coord_system: LoadCoordSystem::Global,
        q_start: [0.0, 0.0, -w],
        q_end: [0.0, 0.0, -w],
    });

    let sol = solve_linear_static(&m).expect("Propped cantilever with release");
    let r1 = sol.node_results[&n1].reactions;
    let r2 = sol.node_results[&n2].reactions;

    assert_rel_close(r1[2], 0.625 * w * l, 1e-10, "Fixed support Rz = 5/8 wL");
    assert_rel_close(r2[2], 0.375 * w * l, 1e-10, "Propped support Rz = 3/8 wL");
    assert!(r2[4].abs() < 1e-6, "Moment reaction at released end must be 0");

    let stations = &sol.element_results[&b1].stations;
    assert_rel_close(
        stations[0].moment_my,
        -0.125 * w * l * l,
        1e-10,
        "Fixed end hogging moment -1/8 wL^2",
    );
    assert!(
        stations.last().unwrap().moment_my.abs() < 1e-6,
        "Hinged end moment must be 0"
    );
}

#[test]
fn test_3d_cranked_l_cantilever_coupled_bending_torsion() {
    let mut m = StructuralModel::default();
    let l1 = 4.0;
    let l2 = 2.5;
    let p = 18_000.0;

    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([l1, 0.0, 0.0]);
    let n3 = m.add_node([l1, l2, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];

    let e1 = m.add_beam(n1, n2, 1, 1).unwrap();
    let e2 = m.add_beam(n2, n3, 1, 1).unwrap();
    m.get_element_mut(e1).unwrap().formulation = BeamFormulation::EulerBernoulli;
    m.get_element_mut(e2).unwrap().formulation = BeamFormulation::EulerBernoulli;

    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n3,
        forces: [0.0, 0.0, -p],
        moments: [0.0; 3],
    });

    let props = m.sections[0].properties();
    let e = m.materials[0].e;
    let g = m.materials[0].g;

    let sol = solve_linear_static(&m).expect("3D cranked cantilever");
    let tip_uz = sol.node_results[&n3].displacements[2];

    let expected_uz = -(p * l1.powi(3) / (3.0 * e * props.iy)
        + p * l2.powi(3) / (3.0 * e * props.iy)
        + p * l1 * l2.powi(2) / (g * props.j));

    assert_rel_close(tip_uz, expected_uz, 1e-10, "Castigliano 3D tip uz");
}

#[test]
fn test_shell_membrane_patch_quad4_and_tri3() {
    // 2 m x 1 m plate under uniform uniaxial tension sigma_xx = 10 MPa (t = 0.01 m)
    let e = 210.0e9;
    let nu = 0.3;
    let g = e / (2.0 * (1.0 + nu));
    let t = 0.01;
    let sigma = 10.0e6;
    let lx = 2.0;
    let ly = 1.0;

    for use_tri in [false, true] {
        let mut m = StructuralModel::default();
        m.materials = vec![Material {
            id: 1,
            name: "Steel".to_string(),
            e,
            g,
            density: 7850.0,
            yield_strength: 355e6,
        }];
        let n1 = m.add_node([0.0, 0.0, 0.0]);
        let n2 = m.add_node([lx, 0.0, 0.0]);
        let n3 = m.add_node([lx, ly, 0.0]);
        let n4 = m.add_node([0.0, ly, 0.0]);

        // Restrain ux at x=0, uy at (0,0), uz/rx/ry everywhere on plane
        m.get_node_mut(n1).unwrap().restraints = [true, true, true, true, true, false];
        m.get_node_mut(n4).unwrap().restraints = [true, false, true, true, true, false];
        m.get_node_mut(n2).unwrap().restraints = [false, false, true, true, true, false];
        m.get_node_mut(n3).unwrap().restraints = [false, false, true, true, true, false];

        if use_tri {
            m.add_shell_tri([n1, n2, n3], 1, t);
            m.add_shell_tri([n1, n3, n4], 1, t);
        } else {
            m.add_shell_quad([n1, n2, n3, n4], 1, t);
        }

        // Uniform edge traction F_total = sigma * t * ly split equally between n2 and n3
        let f_half = 0.5 * sigma * t * ly;
        m.nodal_loads.push(NodalLoad {
            id: 1,
            node_id: n2,
            forces: [f_half, 0.0, 0.0],
            moments: [0.0; 3],
        });
        m.nodal_loads.push(NodalLoad {
            id: 2,
            node_id: n3,
            forces: [f_half, 0.0, 0.0],
            moments: [0.0; 3],
        });

        let sol = solve_linear_static(&m).expect("membrane patch solve");
        let expected_ux = (sigma / e) * lx;
        let expected_uy = -nu * (sigma / e) * ly;

        assert_rel_close(
            sol.node_results[&n2].displacements[0],
            expected_ux,
            1e-9,
            "Membrane ux",
        );
        assert_rel_close(
            sol.node_results[&n4].displacements[1],
            expected_uy,
            1e-9,
            "Membrane Poisson uy",
        );
        for sr in sol.shell_results.values() {
            assert_rel_close(sr.max_von_mises_pa, sigma, 1e-8, "Membrane von Mises stress");
        }
    }
}

#[test]
fn test_simply_supported_square_plate_navier_benchmark() {
    // 4 m x 4 m simply supported steel plate (t = 40 mm), 8x8 Quad4 mesh, q = -10 kN/m^2
    // Analytical Navier thin-plate series: w_max = 0.00406235 * q * a^4 / D
    let model = TemplateKind::SimplySupportedPlate.build();
    let sol = solve_linear_static(&model).expect("plate benchmark solve");

    let a = 4.0_f64;
    let t = 0.040_f64;
    let q = 10_000.0_f64;
    let mat = &model.materials[0];
    let nu = mat.poisson_ratio();
    let d_flex = (mat.e * t.powi(3)) / (12.0 * (1.0 - nu * nu));
    let expected_w_navier = 0.00406235 * q * a.powi(4) / d_flex;

    // 8x8 Q4-SRI Mindlin plate matches Kirchhoff Navier thin-plate solution within ~1.5%
    assert_rel_close(
        sol.summary.max_displacement_m,
        expected_w_navier,
        0.02,
        "Navier simply supported plate center deflection",
    );
    assert!(
        sol.summary.force_equilibrium_residual_n < 1e-6,
        "Plate vertical equilibrium residual too high: {}",
        sol.summary.force_equilibrium_residual_n
    );
}

#[test]
fn test_all_templates_solve_and_satisfy_equilibrium() {
    for kind in TemplateKind::ALL {
        let model = kind.build();
        let sol = solve_linear_static(&model)
            .unwrap_or_else(|e| panic!("Template {:?} failed to solve: {e}", kind));
        assert!(
            sol.summary.force_equilibrium_residual_n < 1e-5,
            "Template {:?} equilibrium residual too high: {}",
            kind,
            sol.summary.force_equilibrium_residual_n
        );
        assert!(sol.summary.max_displacement_m > 0.0);
    }
}

#[test]
fn test_nonlinear_cantilever_90deg_circular_elastica() {
    // Classic large-rotation benchmark: cantilever beam under pure tip moment My = pi*EI / (2*L)
    // Exact analytical elastica bends into an exact 90-degree quarter-circle:
    //   x_tip = (2/pi)*L, z_tip = -(2/pi)*L, theta_y_tip = +pi/2 (right-hand rule: +My bends toward -Z)
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Slender Strip".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.04,
            width: 0.08,
        },
    }];

    let l = 4.0;
    let n_elem = 16;
    let mut nodes = Vec::with_capacity(n_elem + 1);
    for i in 0..=n_elem {
        let x = l * (i as f64 / n_elem as f64);
        nodes.push(m.add_node([x, 0.0, 0.0]));
    }
    m.get_node_mut(nodes[0]).unwrap().restraints = [true; 6];
    for i in 0..n_elem {
        let eid = m.add_beam(nodes[i], nodes[i + 1], 1, 1).unwrap();
        m.get_element_mut(eid).unwrap().formulation = BeamFormulation::EulerBernoulli;
    }

    let props = m.sections[0].properties();
    let ei = m.materials[0].e * props.iy;
    let m_90 = std::f64::consts::FRAC_PI_2 * ei / l;

    // Positive My at tip bends beam downward (-Z) with positive ry = +pi/2
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: *nodes.last().unwrap(),
        forces: [0.0, 0.0, 0.0],
        moments: [0.0, m_90, 0.0],
    });

    let cfg = NonlinearStaticConfig {
        load_steps: 10,
        max_iterations_per_step: 25,
        force_tol: 1e-7,
        disp_tol: 1e-7,
        ..Default::default()
    };
    let nl = solve_nonlinear_static_with_config(&m, &cfg).expect("90-deg elastica solve");
    assert!(nl.converged);
    assert_eq!(nl.load_steps.len(), 10);

    let tip = nl.solution.node_results[nodes.last().unwrap()].displacements;
    let x_tip = l + tip[0];
    let z_tip = tip[2];
    let expected_coord = 2.0 * l / std::f64::consts::PI;

    assert_rel_close(x_tip, expected_coord, 5e-3, "90-deg elastica x_tip");
    assert_rel_close(z_tip, -expected_coord, 5e-3, "90-deg elastica z_tip");
    assert_rel_close(
        tip[4],
        std::f64::consts::FRAC_PI_2,
        5e-3,
        "90-deg elastica tip rotation ry",
    );
    assert!(
        nl.solution.summary.force_equilibrium_residual_n < 1e-4,
        "Elastica equilibrium residual: {}",
        nl.solution.summary.force_equilibrium_residual_n
    );
}

#[test]
fn test_nonlinear_p_delta_column_amplification_and_small_load_equivalence() {
    // Cantilever column of length L under compressive axial load P = 0.5 * P_cr + small lateral load H.
    // Exact Euler buckling load P_cr = pi^2 * EI / (4 L^2).
    // P-Delta amplification factor at P = 0.5 * P_cr is approximately 1 / (1 - 0.5) = 2.0!
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Square Column".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.10,
            width: 0.10,
        },
    }];

    let l = 5.0;
    let n_elem = 8;
    let mut nodes = Vec::new();
    for i in 0..=n_elem {
        nodes.push(m.add_node([0.0, 0.0, l * (i as f64 / n_elem as f64)]));
    }
    m.get_node_mut(nodes[0]).unwrap().restraints = [true; 6];
    for i in 0..n_elem {
        let eid = m.add_beam(nodes[i], nodes[i + 1], 1, 1).unwrap();
        m.get_element_mut(eid).unwrap().formulation = BeamFormulation::EulerBernoulli;
    }

    let props = m.sections[0].properties();
    let ei = m.materials[0].e * props.iy;
    let p_cr = std::f64::consts::PI.powi(2) * ei / (4.0 * l * l);
    let top_id = *nodes.last().unwrap();

    // 1. Small load equivalence: linear vs nonlinear should match closely
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: top_id,
        forces: [10.0, 0.0, 0.0],
        moments: [0.0; 3],
    });
    let lin_small = solve_linear_static(&m).expect("linear small");
    let nl_small = solve_nonlinear_static(&m).expect("nonlinear small");
    assert_rel_close(
        nl_small.solution.node_results[&top_id].displacements[0],
        lin_small.node_results[&top_id].displacements[0],
        1e-5,
        "Small-load nonlinear == linear",
    );

    // 2. Add compressive axial load P = 0.5 * P_cr -> expect ~2.0x lateral deflection amplification
    m.nodal_loads[0].forces = [500.0, 0.0, -0.5 * p_cr];
    let lin = solve_linear_static(&m).expect("linear P-Delta");
    let nl = solve_nonlinear_static(&m).expect("nonlinear P-Delta");

    let ux_lin = lin.node_results[&top_id].displacements[0];
    let ux_nl = nl.solution.node_results[&top_id].displacements[0];
    let amp = ux_nl / ux_lin;
    // Exact analytical cantilever P-Delta magnification at P/P_cr = 0.5 is 3*(tan(kL)-kL)/(kL)^3
    // with kL = (pi/2)*sqrt(0.5) => amp = 2.0273
    let kl = std::f64::consts::FRAC_PI_2 * 0.5_f64.sqrt();
    let expected_amp = 3.0 * (kl.tan() - kl) / kl.powi(3);
    assert_rel_close(amp, expected_amp, 1.5e-2, "P-Delta amplification at 0.5 P_cr");
}

#[test]
fn test_nonlinear_clamped_shell_membrane_stress_stiffening() {
    // Thin square plate clamped on all 4 edges under large uniform pressure:
    // Nonlinear corotational shell develops membrane tension so w_nl < w_lin!
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    let a = 2.0;
    let t = 0.010; // 10 mm thin plate
    let q = -25_000.0; // large pressure -> w_lin > t
    m.add_shell_mesh_quad(
        [[0.0, 0.0, 0.0], [a, 0.0, 0.0], [a, a, 0.0], [0.0, a, 0.0]],
        4,
        4,
        1,
        t,
        Some((LoadCoordSystem::Global, [0.0, 0.0, q])),
    );
    for n in &mut m.nodes {
        let on_edge = n.position[0].abs() < 1e-6
            || (n.position[0] - a).abs() < 1e-6
            || n.position[1].abs() < 1e-6
            || (n.position[1] - a).abs() < 1e-6;
        if on_edge {
            n.restraints = [true; 6];
        }
    }

    let lin = solve_linear_static(&m).expect("linear thin shell");
    let nl = solve_nonlinear_static_with_config(
        &m,
        &NonlinearStaticConfig {
            force_tol: 1e-9,
            disp_tol: 1e-9,
            ..Default::default()
        },
    )
    .expect("nonlinear thin shell");
    assert!(nl.converged);
    // Membrane tension stiffens the plate, reducing transverse deflection relative to linear theory
    assert!(
        nl.solution.summary.max_displacement_m < 0.85 * lin.summary.max_displacement_m,
        "Expected membrane stress-stiffening: nl={} vs lin={}",
        nl.solution.summary.max_displacement_m,
        lin.summary.max_displacement_m
    );
    assert!(
        nl.solution.summary.force_equilibrium_residual_n < 1e-3,
        "Shell nonlinear equilibrium residual = {}",
        nl.solution.summary.force_equilibrium_residual_n
    );
}

#[test]
fn test_modal_cantilever_analytical_frequencies_and_subspace_path() {
    // Exact Euler-Bernoulli cantilever bending natural frequencies:
    // f_n = (beta_n^2 / (2*pi*L^2)) * sqrt(EI / (rho*A))
    // beta_1 = 1.87510407, beta_2 = 4.69409113, beta_3 = 7.85475744
    let build_cantilever = |n_elem: usize| -> (StructuralModel, f64, f64, f64) {
        let mut m = StructuralModel::default();
        m.materials = vec![Material::structural_steel_s355(1)];
        // Square section so bending in Y and Z have identical frequencies (double multiplicity!)
        m.sections = vec![CrossSection {
            id: 1,
            name: "Square 100x100".to_string(),
            profile: SectionProfile::SolidRect {
                depth: 0.10,
                width: 0.10,
            },
        }];
        let l = 4.0;
        let mut nodes = Vec::new();
        for i in 0..=n_elem {
            nodes.push(m.add_node([l * (i as f64 / n_elem as f64), 0.0, 0.0]));
        }
        m.get_node_mut(nodes[0]).unwrap().restraints = [true; 6];
        for i in 0..n_elem {
            let eid = m.add_beam(nodes[i], nodes[i + 1], 1, 1).unwrap();
            m.get_element_mut(eid).unwrap().formulation = BeamFormulation::EulerBernoulli;
        }
        let props = m.sections[0].properties();
        let mat = &m.materials[0];
        let coeff = (mat.e * props.iy / (mat.density * props.area)).sqrt()
            / (2.0 * std::f64::consts::PI * l * l);
        let f1 = 1.87510407_f64.powi(2) * coeff;
        let f2 = 4.69409113_f64.powi(2) * coeff;
        let f3 = 7.85475744_f64.powi(2) * coeff;
        (m, f1, f2, f3)
    };

    // 1. Dense Cholesky + SymmetricEigen path (12 elements -> 72 free DOFs)
    let (m_dense, f1, f2, f3) = build_cantilever(12);
    let modal_cons = solve_modal_with_config(
        &m_dense,
        &ModalConfig {
            num_modes: 6,
            mass_formulation: MassFormulation::Consistent,
            stations_per_element: 9,
        },
    )
    .expect("modal consistent");
    // Modes 1&2 are bending mode 1 (Y & Z), modes 3&4 are bending mode 2, modes 5&6 are bending mode 3
    assert_rel_close(modal_cons.modes[0].frequency_hz, f1, 2e-3, "Cantilever f1 (consistent)");
    assert_rel_close(modal_cons.modes[1].frequency_hz, f1, 2e-3, "Cantilever f1 duplicate");
    assert_rel_close(modal_cons.modes[2].frequency_hz, f2, 5e-3, "Cantilever f2 (consistent)");
    assert_rel_close(modal_cons.modes[4].frequency_hz, f3, 1e-2, "Cantilever f3 (consistent)");

    // Lumped mass check
    let modal_lump = solve_modal_with_config(
        &m_dense,
        &ModalConfig {
            num_modes: 4,
            mass_formulation: MassFormulation::Lumped,
            stations_per_element: 5,
        },
    )
    .expect("modal lumped");
    assert_rel_close(modal_lump.modes[0].frequency_hz, f1, 2e-2, "Cantilever f1 (lumped)");

    // 2. Large-DOF Bathe Subspace Iteration path (45 elements -> 270 free DOFs > 240)
    let (m_large, f1_l, f2_l, _) = build_cantilever(45);
    let modal_sub = solve_modal_with_config(
        &m_large,
        &ModalConfig {
            num_modes: 4,
            mass_formulation: MassFormulation::Consistent,
            stations_per_element: 3,
        },
    )
    .expect("modal subspace iteration");
    assert!(modal_sub.free_dofs > 240);
    assert_rel_close(modal_sub.modes[0].frequency_hz, f1_l, 2e-3, "Subspace cantilever f1");
    assert_rel_close(modal_sub.modes[2].frequency_hz, f2_l, 5e-3, "Subspace cantilever f2");
}

#[test]
fn test_modal_simply_supported_plate_navier_frequency() {
    // Exact Navier fundamental frequency for a simply supported square plate:
    // f_11 = (pi / a^2) * sqrt(D / (rho * t))
    let model = TemplateKind::SimplySupportedPlate.build();
    let modal = solve_modal_with_config(
        &model,
        &ModalConfig {
            num_modes: 3,
            mass_formulation: MassFormulation::Consistent,
            stations_per_element: 3,
        },
    )
    .expect("plate modal solve");

    let a = 4.0_f64;
    let t = 0.040_f64;
    let mat = &model.materials[0];
    let nu = mat.poisson_ratio();
    let d_flex = (mat.e * t.powi(3)) / (12.0 * (1.0 - nu * nu));
    let expected_f11 = (std::f64::consts::PI / (a * a)) * (d_flex / (mat.density * t)).sqrt();

    assert_rel_close(
        modal.modes[0].frequency_hz,
        expected_f11,
        0.03,
        "Navier SS plate fundamental frequency f11",
    );
}

#[test]
fn test_dynamic_step_load_daf_2_and_energy_conservation() {
    // Tip-mass cantilever oscillator (dominated by lumped tip mass m_tip >> beam mass):
    // Under undamped Heaviside step load Fz, exact analytical peak displacement = 2.0 * u_static!
    let mut m = StructuralModel::default();
    let mut light_steel = Material::structural_steel_s355(1);
    light_steel.density = 1.0; // negligible distributed beam mass so it's a clean SDOF oscillator
    m.materials = vec![light_steel];
    m.sections = vec![CrossSection {
        id: 1,
        name: "IPE 300".to_string(),
        profile: SectionProfile::IBeam {
            depth: 0.300,
            width: 0.150,
            web_thickness: 0.0071,
            flange_thickness: 0.0107,
        },
    }];

    let l = 4.0;
    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([l, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    let m_tip = 5_000.0; // 5000 kg lumped mass at tip
    m.get_node_mut(n2).unwrap().added_mass = m_tip;
    let b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    m.get_element_mut(b1).unwrap().formulation = BeamFormulation::EulerBernoulli;

    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n2,
        forces: [0.0, 0.0, -20_000.0],
        moments: [0.0; 3],
    });

    let stat = solve_linear_static(&m).expect("static ref");
    let u_static = stat.node_results[&n2].displacements[2].abs();
    let k_eff = 20_000.0 / u_static;
    let period = 2.0 * std::f64::consts::PI * (m_tip / k_eff).sqrt();

    // 1. Undamped step load: DAF = u_peak / u_static == 2.0
    let dyn_step = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 1.5 * period,
            dt_s: period / 100.0,
            damping: RayleighDamping::None,
            excitation: TimeHistoryExcitation::Step,
            nonlinear: false,
            max_stored_frames: 30,
            ..Default::default()
        },
    )
    .expect("undamped step dynamic");

    let daf = dyn_step.peak_displacement_m / u_static;
    assert_rel_close(daf, 2.0, 1.5e-2, "Undamped SDOF step-load DAF = 2.0");

    // 2. Undamped free vibration: total mechanical energy (KE + SE) is conserved!
    let dyn_free = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 2.0 * period,
            dt_s: period / 80.0,
            damping: RayleighDamping::None,
            excitation: TimeHistoryExcitation::FreeVibration,
            nonlinear: false,
            max_stored_frames: 20,
            ..Default::default()
        },
    )
    .expect("undamped free vibration");

    let e0 = dyn_free.history[0].kinetic_energy_j + dyn_free.history[0].strain_energy_j;
    assert!(e0 > 0.0);
    for pt in &dyn_free.history {
        let e = pt.kinetic_energy_j + pt.strain_energy_j;
        assert_rel_close(e, e0, 2e-2, "Undamped energy conservation");
    }

    // 3. Damped free vibration: energy decays significantly
    let dyn_damped = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 2.0 * period,
            dt_s: period / 60.0,
            damping: RayleighDamping::ModalRatio { zeta: 0.08 },
            excitation: TimeHistoryExcitation::FreeVibration,
            nonlinear: false,
            max_stored_frames: 15,
            ..Default::default()
        },
    )
    .expect("damped free vibration");

    let e_end = dyn_damped.history.last().unwrap().kinetic_energy_j
        + dyn_damped.history.last().unwrap().strain_energy_j;
    assert!(
        e_end < 0.25 * e0,
        "Expected damped energy decay: e_end={e_end} vs e0={e0}"
    );
}

#[test]
fn test_nonlinear_large_deformation_dynamics() {
    // Slender cantilever under large step load solved with nonlinear corotational dynamics
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Flexible Strip".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.03,
            width: 0.06,
        },
    }];

    let l = 3.0;
    let n_elem = 6;
    let mut nodes = Vec::new();
    for i in 0..=n_elem {
        nodes.push(m.add_node([l * (i as f64 / n_elem as f64), 0.0, 0.0]));
    }
    m.get_node_mut(nodes[0]).unwrap().restraints = [true; 6];
    for i in 0..n_elem {
        m.add_beam(nodes[i], nodes[i + 1], 1, 1);
    }
    let tip_id = *nodes.last().unwrap();
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: tip_id,
        forces: [0.0, 0.0, -3_500.0],
        moments: [0.0; 3],
    });

    let dyn_nl = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 0.4,
            dt_s: 0.005,
            damping: RayleighDamping::ModalRatio { zeta: 0.05 },
            excitation: TimeHistoryExcitation::Step,
            nonlinear: true,
            max_stored_frames: 20,
            ..Default::default()
        },
    )
    .expect("nonlinear dynamic solve");

    assert!(dyn_nl.nonlinear);
    assert!(dyn_nl.peak_displacement_m > 0.2);
    // Verify genuine axial foreshortening (ux < -0.01 m) during large swing!
    let min_ux = dyn_nl
        .history
        .iter()
        .map(|h| h.node_translations[&tip_id][0])
        .fold(0.0_f64, f64::min);
    assert!(
        min_ux < -0.01,
        "Expected nonlinear axial foreshortening ux < -0.01 m, got {min_ux}"
    );
}

#[test]
fn test_nonlinear_and_dynamic_error_edge_cases() {
    let m = TemplateKind::CantileverBenchmark.build();

    // Zero load steps -> InvalidConfig
    let err = solve_nonlinear_static_with_config(
        &m,
        &NonlinearStaticConfig {
            load_steps: 0,
            ..Default::default()
        },
    );
    assert!(matches!(err, Err(SolverError::InvalidConfig(_))));

    // Zero modes -> InvalidConfig
    let err = solve_modal_with_config(
        &m,
        &ModalConfig {
            num_modes: 0,
            ..Default::default()
        },
    );
    assert!(matches!(err, Err(SolverError::InvalidConfig(_))));

    // Zero duration -> InvalidConfig
    let err = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 0.0,
            ..Default::default()
        },
    );
    assert!(matches!(err, Err(SolverError::InvalidConfig(_))));

    // Zero density -> SingularMassMatrix
    let mut m_zero_mass = m.clone();
    for mat in &mut m_zero_mass.materials {
        mat.density = 0.0;
    }
    let err = solve_modal_with_config(&m_zero_mass, &ModalConfig::default());
    assert!(matches!(err, Err(SolverError::SingularMassMatrix)));
}

#[test]
fn test_nonlinear_udl_station_equilibrium_consistency() {
    // Cantilever under large global -Z UDL: free tip shear and moment must vanish at station[last]
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Strip".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.02,
            width: 0.05,
        },
    }];
    let l = 3.0;
    let n_elem = 10;
    let mut nodes = Vec::new();
    for i in 0..=n_elem {
        nodes.push(m.add_node([l * (i as f64 / n_elem as f64), 0.0, 0.0]));
    }
    m.get_node_mut(nodes[0]).unwrap().restraints = [true; 6];
    let mut eids = Vec::new();
    for i in 0..n_elem {
        let eid = m.add_beam(nodes[i], nodes[i + 1], 1, 1).unwrap();
        m.get_element_mut(eid).unwrap().formulation = BeamFormulation::EulerBernoulli;
        eids.push(eid);
    }
    for (idx, &eid) in eids.iter().enumerate() {
        m.distributed_loads.push(DistributedLoad {
            id: idx + 1,
            element_id: eid,
            coord_system: LoadCoordSystem::Global,
            q_start: [0.0, 0.0, -400.0],
            q_end: [0.0, 0.0, -400.0],
        });
    }

    let nl = solve_nonlinear_static(&m).expect("nonlinear UDL solve");
    let last_er = &nl.solution.element_results[eids.last().unwrap()];
    let last_st = last_er.stations.last().unwrap();
    assert!(
        last_st.moment_my.abs() < 1e-3,
        "Free tip moment should vanish, got {}",
        last_st.moment_my
    );
    assert!(
        last_st.shear_vz.abs() < 1e-3,
        "Free tip shear should vanish, got {}",
        last_st.shear_vz
    );
}

#[test]
fn test_dynamic_and_modal_massless_beam_with_lumped_point_mass() {
    // Massless cantilever (density = 0) with lumped tip mass (singular M_ff on rotational DOFs)
    let mut m = StructuralModel::default();
    let mut mat = Material::structural_steel_s355(1);
    mat.density = 0.0;
    m.materials = vec![mat];
    m.sections = vec![CrossSection {
        id: 1,
        name: "IPE 300".to_string(),
        profile: SectionProfile::IBeam {
            depth: 0.300,
            width: 0.150,
            web_thickness: 0.0071,
            flange_thickness: 0.0107,
        },
    }];
    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([4.0, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    let m_tip = 1_000.0;
    m.get_node_mut(n2).unwrap().added_mass = m_tip;
    let b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    m.get_element_mut(b1).unwrap().formulation = BeamFormulation::EulerBernoulli;
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n2,
        forces: [0.0, 0.0, -10_000.0],
        moments: [0.0; 3],
    });

    let stat = solve_linear_static(&m).expect("static ref");
    let u_static = stat.node_results[&n2].displacements[2].abs();
    let k_eff = 10_000.0 / u_static;
    let period = 2.0 * std::f64::consts::PI * (m_tip / k_eff).sqrt();

    let modal = solve_modal_with_config(
        &m,
        &ModalConfig {
            num_modes: 2,
            ..Default::default()
        },
    )
    .expect("modal on massless beam + point mass");
    assert!(!modal.modes.is_empty());

    let dyn_sol = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 1.5 * period,
            dt_s: period / 80.0,
            damping: RayleighDamping::None,
            excitation: TimeHistoryExcitation::Step,
            nonlinear: false,
            max_stored_frames: 15,
            ..Default::default()
        },
    )
    .expect("dynamic on massless beam + point mass");
    assert_rel_close(
        dyn_sol.peak_displacement_m / u_static,
        2.0,
        2e-2,
        "Massless beam + tip mass DAF = 2.0",
    );
}

#[test]
fn test_nonlinear_partial_rotational_restraints_and_prescribed_displacements() {
    // 1. Partial rotational restraint (rx fixed = 0, ry & rz free) under 3D biaxial load
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Square".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.04,
            width: 0.04,
        },
    }];
    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([1.5, 0.0, 0.0]);
    let n3 = m.add_node([3.0, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    m.get_node_mut(n2).unwrap().restraints = [false, false, false, true, false, false];
    m.get_node_mut(n3).unwrap().restraints = [false, false, false, true, false, false];
    m.add_beam(n1, n2, 1, 1).unwrap();
    m.add_beam(n2, n3, 1, 1).unwrap();
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n3,
        forces: [0.0, 2000.0, -2000.0],
        moments: [0.0; 3],
    });

    let nl = solve_nonlinear_static(&m).expect("partial rot restraint nonlinear");
    assert!(
        nl.solution.node_results[&n3].displacements[3].abs() < 1e-12,
        "Restrained rx DOF must remain 0"
    );

    // 2. Zero external force with non-zero prescribed support displacement
    let mut mp = StructuralModel::default();
    mp.materials = vec![Material::structural_steel_s355(1)];
    mp.sections = m.sections.clone();
    let p1 = mp.add_node([0.0, 0.0, 0.0]);
    let p2 = mp.add_node([2.0, 0.0, 0.0]);
    let p3 = mp.add_node([4.0, 0.0, 0.0]);
    mp.get_node_mut(p1).unwrap().restraints = [true; 6];
    mp.get_node_mut(p3).unwrap().restraints = [true, true, true, false, false, false];
    mp.get_node_mut(p3).unwrap().prescribed = [0.0, 0.0, -0.3, 0.0, 0.0, 0.0];
    mp.add_beam(p1, p2, 1, 1).unwrap();
    mp.add_beam(p2, p3, 1, 1).unwrap();

    let nlp = solve_nonlinear_static(&mp).expect("prescribed disp nonlinear");
    assert!(nlp.converged);
    assert!((nlp.solution.node_results[&p3].displacements[2] - (-0.3)).abs() < 1e-12);
    assert!(nlp.solution.node_results[&p2].displacements[2] < -0.05);
}

#[test]
fn test_nonlinear_shell_quad4_and_tri3_cantilever_large_bending() {
    // 1. Quad4 cantilever strip under large tip load (genuine axial foreshortening)
    let mut ms = StructuralModel::default();
    ms.materials = vec![Material::structural_steel_s355(1)];
    ms.add_shell_mesh_quad(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 0.2, 0.0], [0.0, 0.2, 0.0]],
        8,
        1,
        1,
        0.015,
        None,
    );
    let mut tip_nodes = Vec::new();
    for node in &mut ms.nodes {
        if node.position[0].abs() < 1e-6 {
            node.restraints = [true; 6];
        } else if (node.position[0] - 2.0).abs() < 1e-6 {
            tip_nodes.push(node.id);
        }
    }
    for (idx, &nid) in tip_nodes.iter().enumerate() {
        ms.nodal_loads.push(NodalLoad {
            id: idx + 1,
            node_id: nid,
            forces: [0.0, 0.0, -1_500.0],
            moments: [0.0; 3],
        });
    }
    let nl_q = solve_nonlinear_static(&ms).expect("Quad4 strip nonlinear");
    let tip_u = nl_q.solution.node_results[&tip_nodes[0]].displacements;
    assert!(tip_u[0] < -0.08, "Expected Quad4 foreshortening, got ux={}", tip_u[0]);
    assert!(nl_q.solution.summary.force_equilibrium_residual_n < 1e-2);

    // 2. Tri3 cantilever strip under large tip load
    let mut mt = StructuralModel::default();
    mt.materials = vec![Material::structural_steel_s355(1)];
    let nx = 8;
    let mut bot = Vec::new();
    let mut top = Vec::new();
    for i in 0..=nx {
        let x = 2.0 * i as f64 / nx as f64;
        let nb = mt.add_node([x, 0.0, 0.0]);
        let nt = mt.add_node([x, 0.2, 0.0]);
        if i == 0 {
            mt.get_node_mut(nb).unwrap().restraints = [true; 6];
            mt.get_node_mut(nt).unwrap().restraints = [true; 6];
        }
        bot.push(nb);
        top.push(nt);
    }
    for i in 0..nx {
        mt.add_shell_tri([bot[i], bot[i + 1], top[i + 1]], 1, 0.015);
        mt.add_shell_tri([bot[i], top[i + 1], top[i]], 1, 0.015);
    }
    for (idx, &nid) in [*bot.last().unwrap(), *top.last().unwrap()].iter().enumerate() {
        mt.nodal_loads.push(NodalLoad {
            id: idx + 1,
            node_id: nid,
            forces: [0.0, 0.0, -2_000.0],
            moments: [0.0; 3],
        });
    }
    let nl_t = solve_nonlinear_static(&mt).expect("Tri3 strip nonlinear");
    let tip_ut = nl_t.solution.node_results[bot.last().unwrap()].displacements;
    assert!(tip_ut[0] < -0.05, "Expected Tri3 foreshortening, got ux={}", tip_ut[0]);
    assert!(nl_t.solution.summary.force_equilibrium_residual_n < 1e-2);
}

#[test]
fn test_dynamic_excitations_harmonic_resonance_and_base_sine() {
    let m = TemplateKind::FlexibleCantileverElastica.build();
    let modal = crate::solve_modal(&m).expect("modal");
    let f1 = modal.modes[0].frequency_hz;

    let res_on = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 3.0 / f1,
            dt_s: 1.0 / (40.0 * f1),
            damping: RayleighDamping::ModalRatio { zeta: 0.02 },
            excitation: TimeHistoryExcitation::Harmonic { frequency_hz: f1 },
            nonlinear: false,
            max_stored_frames: 8,
            ..Default::default()
        },
    )
    .expect("harmonic on-resonance");

    let res_off = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 3.0 / f1,
            dt_s: 1.0 / (40.0 * f1),
            damping: RayleighDamping::ModalRatio { zeta: 0.02 },
            excitation: TimeHistoryExcitation::Harmonic {
                frequency_hz: 3.0 * f1,
            },
            nonlinear: false,
            max_stored_frames: 8,
            ..Default::default()
        },
    )
    .expect("harmonic off-resonance");
    assert!(res_on.peak_displacement_m > 3.0 * res_off.peak_displacement_m);

    let base_res = solve_dynamic_with_config(
        &m,
        &DynamicConfig {
            duration_s: 2.0 / f1,
            dt_s: 1.0 / (40.0 * f1),
            damping: RayleighDamping::ModalRatio { zeta: 0.02 },
            excitation: TimeHistoryExcitation::BaseSine {
                direction: [0.0, 0.0, 1.0],
                amplitude_g: 0.5,
                frequency_hz: f1,
                duration_s: 2.0 / f1,
            },
            nonlinear: false,
            max_stored_frames: 8,
            ..Default::default()
        },
    )
    .expect("base sine");
    assert!(base_res.peak_displacement_m > 0.05);

    for exc in [
        TimeHistoryExcitation::Impulse { duration_s: 0.1 },
        TimeHistoryExcitation::HalfSinePulse { duration_s: 0.15 },
        TimeHistoryExcitation::Ramp { rise_time_s: 0.2 },
        TimeHistoryExcitation::FreeVibration,
    ] {
        let r = solve_dynamic_with_config(
            &m,
            &DynamicConfig {
                duration_s: 0.3,
                dt_s: 0.01,
                damping: RayleighDamping::ModalRatio { zeta: 0.02 },
                excitation: exc,
                nonlinear: true,
                max_stored_frames: 6,
                ..Default::default()
            },
        );
        assert!(r.is_ok(), "Nonlinear dynamic failed for {:?}: {:?}", exc, r.err());
    }
}

#[test]
fn test_nonlinear_member_end_releases_hinge() {
    let mut m = StructuralModel::default();
    m.materials = vec![Material::structural_steel_s355(1)];
    m.sections = vec![CrossSection {
        id: 1,
        name: "Strip".to_string(),
        profile: SectionProfile::SolidRect {
            depth: 0.04,
            width: 0.04,
        },
    }];
    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([2.0, 0.0, 0.0]);
    let n3 = m.add_node([4.0, 0.0, 0.0]);
    m.get_node_mut(n1).unwrap().restraints = [true; 6];
    m.get_node_mut(n3).unwrap().restraints = [false, true, true, false, false, false];
    let _b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    let b2 = m.add_beam(n2, n3, 1, 1).unwrap();
    m.get_element_mut(b2).unwrap().releases_i[4] = true;
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n2,
        forces: [0.0, 0.0, -5000.0],
        moments: [0.0; 3],
    });

    let nl = solve_nonlinear_static(&m).expect("hinged beam nonlinear");
    let er2 = &nl.solution.element_results[&b2];
    assert!(er2.stations[0].moment_my.abs() < 1e-2);
    assert!(nl.solution.summary.force_equilibrium_residual_n < 1e-2);
}
