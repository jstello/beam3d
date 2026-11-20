use crate::model::{
    DistributedLoad, GravitySettings, LoadCoordSystem, Material, NodalLoad, StructuralModel,
    SupportPreset,
};
use crate::section::{CrossSection, SectionProfile};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TemplateKind {
    MultiStoryBuilding,
    CompositeSlabOnBeams,
    SimplySupportedPlate,
    TiedArchBridge,
    PitchedPortalFrame,
    CrankedCantilever3D,
    CantileverBenchmark,
    FlexibleCantileverElastica,
}

impl TemplateKind {
    pub const ALL: [TemplateKind; 8] = [
        TemplateKind::MultiStoryBuilding,
        TemplateKind::CompositeSlabOnBeams,
        TemplateKind::SimplySupportedPlate,
        TemplateKind::TiedArchBridge,
        TemplateKind::PitchedPortalFrame,
        TemplateKind::CrankedCantilever3D,
        TemplateKind::CantileverBenchmark,
        TemplateKind::FlexibleCantileverElastica,
    ];

    pub fn title(self) -> &'static str {
        match self {
            TemplateKind::MultiStoryBuilding => "3D Braced Multi-Story Steel Frame",
            TemplateKind::CompositeSlabOnBeams => "3D Concrete Floor Slab on Steel Beams (Shell + Beam)",
            TemplateKind::SimplySupportedPlate => "2D Simply Supported Square Plate (Navier Benchmark)",
            TemplateKind::TiedArchBridge => "3D Tied-Arch Pedestrian Bridge",
            TemplateKind::PitchedPortalFrame => "Pitched Roof Portal Frame (Snow + Wind)",
            TemplateKind::CrankedCantilever3D => "3D L-Cantilever (Coupled Bending & Torsion)",
            TemplateKind::CantileverBenchmark => "Cantilever Beam (Timoshenko Benchmark)",
            TemplateKind::FlexibleCantileverElastica => "Flexible Cantilever Strip (Large Deform. & Dynamics)",
        }
    }

    pub fn build(self) -> StructuralModel {
        match self {
            TemplateKind::MultiStoryBuilding => multi_story_building(),
            TemplateKind::CompositeSlabOnBeams => composite_slab_on_beams(),
            TemplateKind::SimplySupportedPlate => simply_supported_plate(),
            TemplateKind::TiedArchBridge => tied_arch_bridge(),
            TemplateKind::PitchedPortalFrame => pitched_portal_frame(),
            TemplateKind::CrankedCantilever3D => cranked_cantilever_3d(),
            TemplateKind::CantileverBenchmark => cantilever_benchmark(),
            TemplateKind::FlexibleCantileverElastica => flexible_cantilever_elastica(),
        }
    }
}

pub fn multi_story_building() -> StructuralModel {
    let mut m = StructuralModel {
        name: "3-Story Braced Space Frame".to_string(),
        description: "2x2 bay, 3-story structural steel frame with HEB columns, IPE girders, HSS lateral bracing, floor live loads, self-weight, and wind lateral loads.".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![
            CrossSection {
                id: 1,
                name: "HEB 300 (Column)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.300,
                    width: 0.300,
                    web_thickness: 0.011,
                    flange_thickness: 0.019,
                },
            },
            CrossSection {
                id: 2,
                name: "IPE 330 (Girder)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.330,
                    width: 0.160,
                    web_thickness: 0.0075,
                    flange_thickness: 0.0115,
                },
            },
            CrossSection {
                id: 3,
                name: "HSS 140x140x8 (Brace)".to_string(),
                profile: SectionProfile::HollowBox {
                    depth: 0.140,
                    width: 0.140,
                    thickness: 0.008,
                },
            },
        ],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: true,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let xs = [0.0, 6.0, 12.0];
    let ys = [0.0, 5.5, 11.0];
    let zs = [0.0, 4.0, 7.6, 11.2];

    let mut grid = [[[0usize; 4]; 3]; 3];
    for (iz, &z) in zs.iter().enumerate() {
        for (iy, &y) in ys.iter().enumerate() {
            for (ix, &x) in xs.iter().enumerate() {
                let nid = m.add_node([x, y, z]);
                grid[ix][iy][iz] = nid;
                if iz == 0 {
                    if let Some(node) = m.get_node_mut(nid) {
                        node.restraints = SupportPreset::Fixed.to_restraints().unwrap();
                    }
                }
            }
        }
    }

    // Columns (section 1)
    for ix in 0..3 {
        for iy in 0..3 {
            for iz in 0..3 {
                m.add_beam(grid[ix][iy][iz], grid[ix][iy][iz + 1], 1, 1);
            }
        }
    }

    // X-direction primary girders (section 2) with uniform floor distributed loads
    for iz in 1..=3 {
        let qz = if iz == 3 { -12_000.0 } else { -22_000.0 }; // N/m
        for iy in 0..3 {
            for ix in 0..2 {
                if let Some(eid) = m.add_beam(grid[ix][iy][iz], grid[ix + 1][iy][iz], 1, 2) {
                    let dl_id = m.next_distributed_load_id();
                    m.distributed_loads.push(DistributedLoad {
                        id: dl_id,
                        element_id: eid,
                        coord_system: LoadCoordSystem::Global,
                        q_start: [0.0, 0.0, qz],
                        q_end: [0.0, 0.0, qz],
                    });
                }
            }
        }
    }

    // Y-direction girders (section 2)
    for iz in 1..=3 {
        let qz = if iz == 3 { -8_000.0 } else { -14_000.0 };
        for ix in 0..3 {
            for iy in 0..2 {
                if let Some(eid) = m.add_beam(grid[ix][iy][iz], grid[ix][iy + 1][iz], 1, 2) {
                    let dl_id = m.next_distributed_load_id();
                    m.distributed_loads.push(DistributedLoad {
                        id: dl_id,
                        element_id: eid,
                        coord_system: LoadCoordSystem::Global,
                        q_start: [0.0, 0.0, qz],
                        q_end: [0.0, 0.0, qz],
                    });
                }
            }
        }
    }

    // Pin-ended diagonal HSS braces in perimeter bays
    for iz in 0..3 {
        let pairs = [
            (grid[0][0][iz], grid[1][0][iz + 1]),
            (grid[2][2][iz], grid[1][2][iz + 1]),
            (grid[0][0][iz], grid[0][1][iz + 1]),
            (grid[2][2][iz], grid[2][1][iz + 1]),
        ];
        for (na, nb) in pairs {
            if let Some(eid) = m.add_beam(na, nb, 1, 3) {
                if let Some(el) = m.get_element_mut(eid) {
                    // Release bending moments at both ends (ry, rz) so brace acts primarily axially
                    el.releases_i = [false, false, false, false, true, true];
                    el.releases_j = [false, false, false, false, true, true];
                }
            }
        }
    }

    // Lateral wind loads on X=0 facade (+X + slight +Y oblique wind)
    for iz in 1..=3 {
        let fx = 38_000.0 * (iz as f64 / 2.0);
        let fy = 12_000.0 * (iz as f64 / 2.0);
        for iy in 0..3 {
            let nl_id = m.next_nodal_load_id();
            m.nodal_loads.push(NodalLoad {
                id: nl_id,
                node_id: grid[0][iy][iz],
                forces: [fx, fy, 0.0],
                moments: [0.0; 3],
            });
        }
    }

    m
}

pub fn tied_arch_bridge() -> StructuralModel {
    let mut m = StructuralModel {
        name: "3D Tied-Arch Pedestrian Bridge".to_string(),
        description: "24 m span parabolic tubular steel arch bridge with I-beam tie girders, cross beams, pin-ended vertical hangers, crown wind bracing, and pedestrian deck loading.".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![
            CrossSection {
                id: 1,
                name: "Pipe Ø355x16 (Arch Rib)".to_string(),
                profile: SectionProfile::Pipe {
                    outer_diameter: 0.355,
                    thickness: 0.016,
                },
            },
            CrossSection {
                id: 2,
                name: "IPE 450 (Tie Girder)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.450,
                    width: 0.190,
                    web_thickness: 0.0094,
                    flange_thickness: 0.0146,
                },
            },
            CrossSection {
                id: 3,
                name: "IPE 300 (Cross Beam)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.300,
                    width: 0.150,
                    web_thickness: 0.0071,
                    flange_thickness: 0.0107,
                },
            },
            CrossSection {
                id: 4,
                name: "Rod Ø50 (Hanger)".to_string(),
                profile: SectionProfile::SolidCircle { diameter: 0.050 },
            },
            CrossSection {
                id: 5,
                name: "Pipe Ø168x8 (Crown Strut)".to_string(),
                profile: SectionProfile::Pipe {
                    outer_diameter: 0.168,
                    thickness: 0.008,
                },
            },
        ],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: true,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let span = 24.0;
    let width = 4.2;
    let rise = 5.2;
    let n_panels = 8usize;

    let mut deck_left = Vec::with_capacity(n_panels + 1);
    let mut deck_right = Vec::with_capacity(n_panels + 1);
    let mut arch_left = Vec::with_capacity(n_panels + 1);
    let mut arch_right = Vec::with_capacity(n_panels + 1);

    for i in 0..=n_panels {
        let t = i as f64 / n_panels as f64;
        let x = t * span;
        let z_arch = 4.0 * rise * t * (1.0 - t);

        let dl = m.add_node([x, 0.0, 0.0]);
        let dr = m.add_node([x, width, 0.0]);
        deck_left.push(dl);
        deck_right.push(dr);

        if i == 0 || i == n_panels {
            arch_left.push(dl);
            arch_right.push(dr);
        } else {
            arch_left.push(m.add_node([x, 0.0, z_arch]));
            arch_right.push(m.add_node([x, width, z_arch]));
        }
    }

    // Bridge bearings: pinned at x=0 left & right (with Y free on right), vertical rollers at x=span
    if let Some(n) = m.get_node_mut(deck_left[0]) {
        n.restraints = [true, true, true, false, false, false];
    }
    if let Some(n) = m.get_node_mut(deck_right[0]) {
        n.restraints = [true, false, true, false, false, false];
    }
    if let Some(n) = m.get_node_mut(deck_left[n_panels]) {
        n.restraints = [false, true, true, false, false, false];
    }
    if let Some(n) = m.get_node_mut(deck_right[n_panels]) {
        n.restraints = [false, false, true, false, false, false];
    }

    // Tie girders (section 2) + asymmetric pedestrian + crowd loading
    for i in 0..n_panels {
        let q_left = if i < n_panels / 2 { -18_000.0 } else { -12_000.0 };
        let q_right = -14_000.0;

        if let Some(eid) = m.add_beam(deck_left[i], deck_left[i + 1], 1, 2) {
            let lid = m.next_distributed_load_id();
            m.distributed_loads.push(DistributedLoad {
                id: lid,
                element_id: eid,
                coord_system: LoadCoordSystem::Global,
                q_start: [0.0, 0.0, q_left],
                q_end: [0.0, 0.0, q_left],
            });
        }
        if let Some(eid) = m.add_beam(deck_right[i], deck_right[i + 1], 1, 2) {
            let lid = m.next_distributed_load_id();
            m.distributed_loads.push(DistributedLoad {
                id: lid,
                element_id: eid,
                coord_system: LoadCoordSystem::Global,
                q_start: [0.0, 0.0, q_right],
                q_end: [0.0, 0.0, q_right],
            });
        }

        // Arch rib segments (section 1)
        m.add_beam(arch_left[i], arch_left[i + 1], 1, 1);
        m.add_beam(arch_right[i], arch_right[i + 1], 1, 1);
    }

    // Transverse deck cross beams (section 3)
    for i in 0..=n_panels {
        m.add_beam(deck_left[i], deck_right[i], 1, 3);
    }

    // Vertical hangers (section 4, moment-released at top & bottom) + overhead arch struts
    for i in 1..n_panels {
        for &(dn, an) in &[(deck_left[i], arch_left[i]), (deck_right[i], arch_right[i])] {
            if let Some(eid) = m.add_beam(dn, an, 1, 4) {
                if let Some(el) = m.get_element_mut(eid) {
                    el.releases_i = [false, false, false, false, true, true];
                    el.releases_j = [false, false, false, false, true, true];
                }
            }
        }
        if (2..=n_panels - 2).contains(&i) {
            m.add_beam(arch_left[i], arch_right[i], 1, 5);
        }
    }

    // Crown K/X wind bracing between central arch ribs + lateral wind on arch
    for i in 2..n_panels - 2 {
        m.add_beam(arch_left[i], arch_right[i + 1], 1, 5);
        m.add_beam(arch_right[i], arch_left[i + 1], 1, 5);
        let nl_id = m.next_nodal_load_id();
        m.nodal_loads.push(NodalLoad {
            id: nl_id,
            node_id: arch_left[i],
            forces: [0.0, 15_000.0, 0.0],
            moments: [0.0; 3],
        });
    }

    m
}

pub fn pitched_portal_frame() -> StructuralModel {
    let mut m = StructuralModel {
        name: "Pitched Steel Portal Frame".to_string(),
        description: "16 m clear-spanindustrial portal frame with pinned bases, IPE 400 columns & rafters, drifted trapezoidal roof snow load, and windward column pressure.".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![CrossSection {
            id: 1,
            name: "IPE 400".to_string(),
            profile: SectionProfile::IBeam {
                depth: 0.400,
                width: 0.180,
                web_thickness: 0.0086,
                flange_thickness: 0.0135,
            },
        }],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: true,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let n1 = m.add_node([0.0, 0.0, 0.0]);
    let n2 = m.add_node([0.0, 0.0, 6.0]);
    let n3 = m.add_node([8.0, 0.0, 8.2]);
    let n4 = m.add_node([16.0, 0.0, 6.0]);
    let n5 = m.add_node([16.0, 0.0, 0.0]);

    // Restrain out-of-plane Y & Rx/Rz at eaves/apex for 2D planar stability, pinned bases in X-Z
    for &nid in &[n1, n5] {
        if let Some(n) = m.get_node_mut(nid) {
            n.restraints = [true, true, true, true, false, true];
        }
    }
    for &nid in &[n2, n3, n4] {
        if let Some(n) = m.get_node_mut(nid) {
            n.restraints = [false, true, false, true, false, true];
        }
    }

    let c1 = m.add_beam(n1, n2, 1, 1).unwrap();
    let r1 = m.add_beam(n2, n3, 1, 1).unwrap();
    let r2 = m.add_beam(n3, n4, 1, 1).unwrap();
    let _c2 = m.add_beam(n5, n4, 1, 1).unwrap();

    // Windward horizontal pressure on left column (+X)
    m.distributed_loads.push(DistributedLoad {
        id: 1,
        element_id: c1,
        coord_system: LoadCoordSystem::Global,
        q_start: [6_000.0, 0.0, 0.0],
        q_end: [9_500.0, 0.0, 0.0],
    });
    // Trapezoidal drifted snow load on rafters (-Z)
    m.distributed_loads.push(DistributedLoad {
        id: 2,
        element_id: r1,
        coord_system: LoadCoordSystem::Global,
        q_start: [0.0, 0.0, -14_000.0],
        q_end: [0.0, 0.0, -22_000.0],
    });
    m.distributed_loads.push(DistributedLoad {
        id: 3,
        element_id: r2,
        coord_system: LoadCoordSystem::Global,
        q_start: [0.0, 0.0, -28_000.0],
        q_end: [0.0, 0.0, -16_000.0],
    });

    m
}

pub fn cranked_cantilever_3d() -> StructuralModel {
    let mut m = StructuralModel {
        name: "3D L-Shaped Cranked Cantilever".to_string(),
        description: "Right-angle horizontal space cantilever subjected to vertical tip force and self-weight, inducing simultaneous bending, shear, and St. Venant torsion in leg 1.".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![CrossSection {
            id: 1,
            name: "HSS 250x150x10".to_string(),
            profile: SectionProfile::HollowBox {
                depth: 0.250,
                width: 0.150,
                thickness: 0.010,
            },
        }],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: true,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let n1 = m.add_node([0.0, 0.0, 2.0]);
    let n2 = m.add_node([4.0, 0.0, 2.0]);
    let n3 = m.add_node([4.0, 3.0, 2.0]);

    if let Some(n) = m.get_node_mut(n1) {
        n.restraints = [true; 6];
    }

    let e1 = m.add_beam(n1, n2, 1, 1).unwrap();
    let e2 = m.add_beam(n2, n3, 1, 1).unwrap();

    // Subdivide for visual node markers if desired, though stations already interpolate smoothly
    let _ = (e1, e2);

    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n3,
        forces: [0.0, 0.0, -35_000.0],
        moments: [0.0, 0.0, 0.0],
    });

    m
}

pub fn cantilever_benchmark() -> StructuralModel {
    let mut m = StructuralModel {
        name: "Cantilever Beam Benchmark".to_string(),
        description: "3.0 m deep I-beam cantilever (HEA 400) with 100 kN downward tip load. Switch between Timoshenko and Euler-Bernoulli in the inspector to compare shear + bending deflection!".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![CrossSection {
            id: 1,
            name: "HEA 400".to_string(),
            profile: SectionProfile::IBeam {
                depth: 0.390,
                width: 0.300,
                web_thickness: 0.011,
                flange_thickness: 0.019,
            },
        }],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: false,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let n1 = m.add_node([0.0, 0.0, 1.5]);
    let n2 = m.add_node([3.0, 0.0, 1.5]);
    if let Some(n) = m.get_node_mut(n1) {
        n.restraints = [true; 6];
    }

    let _b1 = m.add_beam(n1, n2, 1, 1).unwrap();
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: n2,
        forces: [0.0, 0.0, -100_000.0],
        moments: [0.0; 3],
    });

    m
}

/// 6 m × 5 m Concrete C30 floor slab (6×5 Quad4 Mindlin shells, t = 180 mm)
/// supported on perimeter IPE 330 steel girders + 4 HEB 260 columns with -8 kN/m² uniform 2D surface load.
pub fn composite_slab_on_beams() -> StructuralModel {
    let mut m = StructuralModel {
        name: "3D Concrete Floor Slab on Steel Beams".to_string(),
        description: "6 m × 5 m concrete slab (30 Quad4 Mindlin-Reissner shell elements, t = 180 mm) coupled to perimeter IPE 330 steel beams and 4 HEB 260 columns with -8 kN/m² uniform 2D surface live load + self-weight.".to_string(),
        nodes: Vec::new(),
        materials: vec![
            Material::structural_steel_s355(1),
            Material::concrete_c30(2),
        ],
        sections: vec![
            CrossSection {
                id: 1,
                name: "HEB 260 (Column)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.260,
                    width: 0.260,
                    web_thickness: 0.010,
                    flange_thickness: 0.0175,
                },
            },
            CrossSection {
                id: 2,
                name: "IPE 330 (Edge Girder)".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.330,
                    width: 0.160,
                    web_thickness: 0.0075,
                    flange_thickness: 0.0115,
                },
            },
        ],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: true,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let lx = 6.0;
    let ly = 5.0;
    let h = 3.5;
    let nx = 6usize;
    let ny = 5usize;

    // 1. Generate 6×5 Quad4 concrete floor slab at Z = h with -8 kN/m² surface live load
    let corners = [
        [0.0, 0.0, h],
        [lx, 0.0, h],
        [lx, ly, h],
        [0.0, ly, h],
    ];
    m.add_shell_mesh_quad(
        corners,
        nx,
        ny,
        2, // Concrete C30
        0.180,
        Some((LoadCoordSystem::Global, [0.0, 0.0, -8_000.0])),
    );

    // 2. Connect perimeter IPE 330 steel girders directly along the slab's shared boundary nodes
    for i in 0..nx {
        let x0 = lx * (i as f64 / nx as f64);
        let x1 = lx * ((i + 1) as f64 / nx as f64);
        // Bottom Y=0 edge
        let a = m.find_or_add_node([x0, 0.0, h], 1e-3);
        let b = m.find_or_add_node([x1, 0.0, h], 1e-3);
        m.add_beam(a, b, 1, 2);
        // Top Y=ly edge
        let c = m.find_or_add_node([x0, ly, h], 1e-3);
        let d = m.find_or_add_node([x1, ly, h], 1e-3);
        m.add_beam(c, d, 1, 2);
    }
    for j in 0..ny {
        let y0 = ly * (j as f64 / ny as f64);
        let y1 = ly * ((j + 1) as f64 / ny as f64);
        // Left X=0 edge
        let a = m.find_or_add_node([0.0, y0, h], 1e-3);
        let b = m.find_or_add_node([0.0, y1, h], 1e-3);
        m.add_beam(a, b, 1, 2);
        // Right X=lx edge
        let c = m.find_or_add_node([lx, y0, h], 1e-3);
        let d = m.find_or_add_node([lx, y1, h], 1e-3);
        m.add_beam(c, d, 1, 2);
    }

    // 3. Add 4 fixed-base HEB 260 steel columns at the corners
    for &[cx, cy] in &[[0.0, 0.0], [lx, 0.0], [lx, ly], [0.0, ly]] {
        let base = m.add_node([cx, cy, 0.0]);
        if let Some(n) = m.get_node_mut(base) {
            n.restraints = SupportPreset::Fixed.to_restraints().unwrap();
        }
        let top = m.find_or_add_node([cx, cy, h], 1e-3);
        m.add_beam(base, top, 1, 1);
    }

    m
}

/// 4 m × 4 m simply supported steel plate (8×8 Quad4 shells, t = 40 mm)
/// under uniform -10 kN/m² 2D pressure — classic Navier plate benchmark.
pub fn simply_supported_plate() -> StructuralModel {
    let mut m = StructuralModel {
        name: "2D Simply Supported Plate (Navier Benchmark)".to_string(),
        description: "4 m × 4 m simply supported steel plate (t = 40 mm, 8×8 Quad4 Mindlin shells) under uniform -10 kN/m² 2D surface pressure. Compare center deflection against Navier's analytical w_max = 0.004062·q·a⁴/D.".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: Vec::new(),
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: false,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let a = 4.0;
    let n = 8usize;
    m.add_shell_mesh_quad(
        [[0.0, 0.0, 0.0], [a, 0.0, 0.0], [a, a, 0.0], [0.0, a, 0.0]],
        n,
        n,
        1,
        0.040,
        Some((LoadCoordSystem::Global, [0.0, 0.0, -10_000.0])),
    );

    // Simply supported perimeter: restrain uz along all 4 edges + ux/uy at two corners to prevent in-plane rigid body motion
    for node in &mut m.nodes {
        let on_x_edge = node.position[0].abs() < 1e-4 || (node.position[0] - a).abs() < 1e-4;
        let on_y_edge = node.position[1].abs() < 1e-4 || (node.position[1] - a).abs() < 1e-4;
        if on_x_edge || on_y_edge {
            node.restraints[2] = true; // uz = 0
        }
        if node.position[0].abs() < 1e-4 && node.position[1].abs() < 1e-4 {
            node.restraints[0] = true;
            node.restraints[1] = true;
        }
        if (node.position[0] - a).abs() < 1e-4 && node.position[1].abs() < 1e-4 {
            node.restraints[1] = true;
        }
    }

    m
}

/// Slender 10-element flexible steel cantilever strip (L = 4 m, 30×80 mm) under large tip load.
/// Ideal for demonstrating 3D corotational large deformations (axial foreshortening & >45° rotation),
/// modal natural frequencies, and linear vs. nonlinear transient dynamics.
pub fn flexible_cantilever_elastica() -> StructuralModel {
    let mut m = StructuralModel {
        name: "Flexible Cantilever (Large Deform. & Dynamics)".to_string(),
        description: "Slender 4 m steel strip (10 elements, 30×80 mm) under a large downward tip force (-4.5 kN). Switch Analysis Mode to Nonlinear Static (True 1:1 scale) to see large-rotation elastica foreshortening, or Dynamic Time-History for nonlinear swings!".to_string(),
        nodes: Vec::new(),
        materials: vec![Material::structural_steel_s355(1)],
        sections: vec![CrossSection {
            id: 1,
            name: "Slender Strip 30×80 mm".to_string(),
            profile: SectionProfile::SolidRect {
                depth: 0.030,
                width: 0.080,
            },
        }],
        elements: Vec::new(),
        shells: Vec::new(),
        nodal_loads: Vec::new(),
        distributed_loads: Vec::new(),
        surface_loads: Vec::new(),
        gravity: GravitySettings {
            enabled: false,
            acceleration: [0.0, 0.0, -9.80665],
        },
    };

    let l = 4.0;
    let n_elem = 10usize;
    let mut ids = Vec::with_capacity(n_elem + 1);
    for i in 0..=n_elem {
        ids.push(m.add_node([l * (i as f64 / n_elem as f64), 0.0, 0.0]));
    }
    if let Some(n0) = m.get_node_mut(ids[0]) {
        n0.restraints = SupportPreset::Fixed.to_restraints().unwrap();
    }
    for i in 0..n_elem {
        m.add_beam(ids[i], ids[i + 1], 1, 1);
    }

    let tip = *ids.last().unwrap();
    if let Some(nt) = m.get_node_mut(tip) {
        nt.added_mass = 25.0; // 25 kg tip mass for crisp dynamic oscillation
    }
    m.nodal_loads.push(NodalLoad {
        id: 1,
        node_id: tip,
        forces: [0.0, 0.0, -4_500.0],
        moments: [0.0; 3],
    });

    m
}
