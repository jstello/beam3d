use crate::{
    BeamFormulation, CrossSection, DistributedLoad, LoadCoordSystem, Material, NodalLoad,
    SectionProfile, StructuralModel, TemplateKind, solve_linear_static,
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
