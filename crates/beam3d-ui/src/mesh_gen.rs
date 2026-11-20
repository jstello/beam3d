use crate::camera::WorkplaneKind;
use crate::renderer::{LineVertex, MeshVertex, SceneGeometry};
use beam3d_core::{
    ElementFrame, LoadCoordSystem, Solution, StationSample, StructuralModel, SupportPreset,
};
use nalgebra::{Matrix3, Rotation3, Unit, Vector3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorField {
    SectionStyle,
    DisplacementMag,
    AxialForce,
    ShearVz,
    ShearVy,
    MomentMy,
    MomentMz,
    TorqueT,
    NormalStress,
    VonMisesStress,
    Utilization,
}

impl ColorField {
    pub const ALL: [ColorField; 11] = [
        ColorField::SectionStyle,
        ColorField::DisplacementMag,
        ColorField::VonMisesStress,
        ColorField::NormalStress,
        ColorField::Utilization,
        ColorField::MomentMy,
        ColorField::MomentMz,
        ColorField::ShearVz,
        ColorField::ShearVy,
        ColorField::AxialForce,
        ColorField::TorqueT,
    ];

    pub fn label(self) -> &'static str {
        match self {
            ColorField::SectionStyle => "Section / Material Color",
            ColorField::DisplacementMag => "Displacement |u| (mm)",
            ColorField::AxialForce => "Axial Force N (kN)",
            ColorField::ShearVz => "Shear |Vz| (kN)",
            ColorField::ShearVy => "Shear |Vy| (kN)",
            ColorField::MomentMy => "Moment |My| (kN·m)",
            ColorField::MomentMz => "Moment |Mz| (kN·m)",
            ColorField::TorqueT => "Torque |T| (kN·m)",
            ColorField::NormalStress => "Combined Normal Stress (MPa)",
            ColorField::VonMisesStress => "Von Mises Stress (MPa)",
            ColorField::Utilization => "Yield Utilization (%)",
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            ColorField::SectionStyle => "",
            ColorField::DisplacementMag => "mm",
            ColorField::AxialForce | ColorField::ShearVz | ColorField::ShearVy => "kN",
            ColorField::MomentMy | ColorField::MomentMz | ColorField::TorqueT => "kN·m",
            ColorField::NormalStress | ColorField::VonMisesStress => "MPa",
            ColorField::Utilization => "%",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagramRibbon {
    None,
    MomentMy,
    MomentMz,
    ShearVz,
    ShearVy,
    AxialN,
    TorqueT,
}

impl DiagramRibbon {
    pub const ALL: [DiagramRibbon; 7] = [
        DiagramRibbon::None,
        DiagramRibbon::MomentMy,
        DiagramRibbon::MomentMz,
        DiagramRibbon::ShearVz,
        DiagramRibbon::ShearVy,
        DiagramRibbon::AxialN,
        DiagramRibbon::TorqueT,
    ];

    pub fn label(self) -> &'static str {
        match self {
            DiagramRibbon::None => "None",
            DiagramRibbon::MomentMy => "Bending Moment My",
            DiagramRibbon::MomentMz => "Bending Moment Mz",
            DiagramRibbon::ShearVz => "Shear Force Vz",
            DiagramRibbon::ShearVy => "Shear Force Vy",
            DiagramRibbon::AxialN => "Axial Force N",
            DiagramRibbon::TorqueT => "Torsion T",
        }
    }
}

pub struct ViewportDisplayOptions {
    pub show_grid: bool,
    pub workplane_kind: WorkplaneKind,
    pub workplane_offset: f64,
    pub show_extruded: bool,
    pub profile_scale: f32,
    pub show_deformed: bool,
    pub show_undeformed_ghost: bool,
    pub disp_scale: f64,
    pub show_supports: bool,
    pub show_loads: bool,
    pub show_reactions: bool,
    pub show_local_axes: bool,
    pub color_field: ColorField,
    pub diagram_ribbon: DiagramRibbon,
    pub diagram_scale: f32,
    pub selected_node: Option<usize>,
    pub hovered_node: Option<usize>,
    pub selected_element: Option<usize>,
    pub hovered_element: Option<usize>,
    pub selected_shell: Option<usize>,
    pub hovered_shell: Option<usize>,
    pub preview_point: Option<[f64; 3]>,
    pub beam_draw_start_node: Option<usize>,
    pub shell_draw_points: Vec<[f64; 3]>,
}

/// Range metadata returned for drawing the 2D HUD colorbar legend.
#[derive(Debug, Clone, Copy)]
pub struct LegendRange {
    pub field: ColorField,
    pub min_display: f64,
    pub max_display: f64,
    pub diverging: bool,
}

/// Smooth scientific Turbo-inspired colormap for `t in [0, 1]`.
pub fn colormap_scientific(t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let stops: [[f32; 3]; 6] = [
        [0.14, 0.34, 0.88], // Deep Royal Blue
        [0.10, 0.72, 0.88], // Cyan
        [0.18, 0.82, 0.45], // Emerald Green
        [0.94, 0.84, 0.18], // Warm Yellow
        [0.96, 0.46, 0.14], // Orange
        [0.86, 0.14, 0.18], // Crimson Red
    ];
    let scaled = t * (stops.len() - 1) as f32;
    let idx = (scaled.floor() as usize).min(stops.len() - 2);
    let frac = scaled - idx as f32;
    let a = stops[idx];
    let b = stops[idx + 1];
    [
        a[0] + frac * (b[0] - a[0]),
        a[1] + frac * (b[1] - a[1]),
        a[2] + frac * (b[2] - a[2]),
        1.0,
    ]
}

/// Diverging Blue (-) -> Cool Silver (0) -> Crimson (+) colormap for `t in [-1, 1]`.
pub fn colormap_diverging(t_signed: f32) -> [f32; 4] {
    let t = t_signed.clamp(-1.0, 1.0);
    let zero = [0.84_f32, 0.87, 0.91];
    if t < 0.0 {
        let u = -t;
        let neg = [0.16_f32, 0.46, 0.95];
        [
            zero[0] + u * (neg[0] - zero[0]),
            zero[1] + u * (neg[1] - zero[1]),
            zero[2] + u * (neg[2] - zero[2]),
            1.0,
        ]
    } else {
        let u = t;
        let pos = [0.92_f32, 0.22, 0.22];
        [
            zero[0] + u * (pos[0] - zero[0]),
            zero[1] + u * (pos[1] - zero[1]),
            zero[2] + u * (pos[2] - zero[2]),
            1.0,
        ]
    }
}

fn section_palette(section_id: usize) -> [f32; 4] {
    const PALETTE: [[f32; 4]; 6] = [
        [0.42, 0.62, 0.82, 1.0], // Structural Steel Blue
        [0.52, 0.74, 0.64, 1.0], // Sage Teal
        [0.78, 0.64, 0.44, 1.0], // Warm Bronze
        [0.64, 0.56, 0.80, 1.0], // Slate Violet
        [0.76, 0.48, 0.52, 1.0], // Oxide Rose
        [0.68, 0.72, 0.76, 1.0], // Brushed Aluminum
    ];
    PALETTE[(section_id.saturating_sub(1)) % PALETTE.len()]
}

pub fn build_scene_geometry(
    model: &StructuralModel,
    solution: Option<&Solution>,
    opts: &ViewportDisplayOptions,
    out: &mut SceneGeometry,
) -> Option<LegendRange> {
    out.clear();

    let char_len = model.characteristic_length() as f32;
    let glyph_size = (char_len * 0.032).clamp(0.12, 1.2);

    if opts.show_grid {
        build_workplane_grid(
            model,
            opts.workplane_kind,
            opts.workplane_offset as f32,
            out,
        );
    }

    let eff_disp_scale = if opts.show_deformed && solution.is_some() {
        opts.disp_scale
    } else {
        0.0
    };

    // Determine color legend range if solution is active
    let legend = compute_legend_range(opts.color_field, solution);

    // 1. Undeformed ghost wireframe when deformed view is active
    if eff_disp_scale.abs() > 1e-9 && opts.show_undeformed_ghost {
        for elem in &model.elements {
            if let (Some(ni), Some(nj)) = (model.get_node(elem.node_i), model.get_node(elem.node_j))
            {
                push_line(
                    &mut out.lines,
                    to_f32(ni.position),
                    to_f32(nj.position),
                    [0.42, 0.47, 0.56, 0.45],
                );
            }
        }
    }

    // 2. Render all 3D Beam Elements
    for elem in &model.elements {
        let (Some(ni), Some(nj)) = (model.get_node(elem.node_i), model.get_node(elem.node_j))
        else {
            continue;
        };
        let Some(sec) = model.get_section(elem.section_id) else {
            continue;
        };
        let mat_fy = model
            .get_material(elem.material_id)
            .map(|m| m.yield_strength.max(1.0))
            .unwrap_or(355e6);

        let Ok(frame) = ElementFrame::compute(ni.position, nj.position, elem.roll_angle_deg) else {
            continue;
        };

        let is_selected = opts.selected_element == Some(elem.id);
        let is_hovered = opts.hovered_element == Some(elem.id);

        let elem_res = solution.and_then(|s| s.element_results.get(&elem.id));

        // Build centerline station frames (position, rotated ey, rotated ez, color)
        let mut ring_centers: Vec<Vector3<f32>> = Vec::new();
        let mut ring_ey: Vec<Vector3<f32>> = Vec::new();
        let mut ring_ez: Vec<Vector3<f32>> = Vec::new();
        let mut ring_colors: Vec<[f32; 4]> = Vec::new();

        let base_ex = Vector3::from(to_f32(frame.ex()));
        let base_ey = Vector3::from(to_f32(frame.ey()));
        let base_ez = Vector3::from(to_f32(frame.ez()));

        if let Some(er) = elem_res {
            for st in &er.stations {
                let p = Vector3::new(
                    (st.pos_undeformed[0] + eff_disp_scale * st.disp_global[0]) as f32,
                    (st.pos_undeformed[1] + eff_disp_scale * st.disp_global[1]) as f32,
                    (st.pos_undeformed[2] + eff_disp_scale * st.disp_global[2]) as f32,
                );
                let (ey_rot, ez_rot) = if eff_disp_scale.abs() > 1e-9 {
                    rotate_section_axes(
                        &frame.r_mat,
                        base_ey,
                        base_ez,
                        st.rot_local,
                        eff_disp_scale,
                    )
                } else {
                    (base_ey, base_ez)
                };
                let mut col = sample_color(opts.color_field, st, mat_fy, elem.section_id, legend);
                if is_selected {
                    col = mix_color(col, [1.0, 0.80, 0.22, 1.0], 0.42);
                } else if is_hovered {
                    col = mix_color(col, [0.55, 0.88, 1.0, 1.0], 0.30);
                }
                ring_centers.push(p);
                ring_ey.push(ey_rot);
                ring_ez.push(ez_rot);
                ring_colors.push(col);
            }
        } else {
            let mut col = section_palette(elem.section_id);
            if is_selected {
                col = [1.0, 0.78, 0.22, 1.0];
            } else if is_hovered {
                col = mix_color(col, [0.55, 0.88, 1.0, 1.0], 0.35);
            }
            ring_centers.push(Vector3::from(to_f32(ni.position)));
            ring_centers.push(Vector3::from(to_f32(nj.position)));
            ring_ey.push(base_ey);
            ring_ey.push(base_ey);
            ring_ez.push(base_ez);
            ring_ez.push(base_ez);
            ring_colors.push(col);
            ring_colors.push(col);
        }

        if opts.show_extruded {
            let (edges, caps) = sec.profile.extrusion_geometry();
            let pscale = opts.profile_scale;
            let edge_line_col = if is_selected {
                [1.0, 0.86, 0.30, 0.95]
            } else if is_hovered {
                [0.65, 0.92, 1.0, 0.85]
            } else {
                [0.10, 0.13, 0.18, 0.55]
            };

            let n_rings = ring_centers.len();
            for k in 0..n_rings - 1 {
                let c0 = ring_centers[k];
                let c1 = ring_centers[k + 1];
                let ey0 = ring_ey[k];
                let ez0 = ring_ez[k];
                let ey1 = ring_ey[k + 1];
                let ez1 = ring_ez[k + 1];
                let col0 = ring_colors[k];
                let col1 = ring_colors[k + 1];

                for edge in &edges {
                    let v00 = c0 + ey0 * (edge.p0[0] * pscale) + ez0 * (edge.p0[1] * pscale);
                    let v01 = c0 + ey0 * (edge.p1[0] * pscale) + ez0 * (edge.p1[1] * pscale);
                    let v10 = c1 + ey1 * (edge.p0[0] * pscale) + ez1 * (edge.p0[1] * pscale);
                    let v11 = c1 + ey1 * (edge.p1[0] * pscale) + ez1 * (edge.p1[1] * pscale);

                    let n00 = (ey0 * edge.n0[0] + ez0 * edge.n0[1]).normalize();
                    let n01 = (ey0 * edge.n1[0] + ez0 * edge.n1[1]).normalize();
                    let n10 = (ey1 * edge.n0[0] + ez1 * edge.n0[1]).normalize();
                    let n11 = (ey1 * edge.n1[0] + ez1 * edge.n1[1]).normalize();

                    out.opaque_tris.extend_from_slice(&[
                        MeshVertex {
                            pos: v00.into(),
                            normal: n00.into(),
                            color: col0,
                        },
                        MeshVertex {
                            pos: v10.into(),
                            normal: n10.into(),
                            color: col1,
                        },
                        MeshVertex {
                            pos: v11.into(),
                            normal: n11.into(),
                            color: col1,
                        },
                        MeshVertex {
                            pos: v00.into(),
                            normal: n00.into(),
                            color: col0,
                        },
                        MeshVertex {
                            pos: v11.into(),
                            normal: n11.into(),
                            color: col1,
                        },
                        MeshVertex {
                            pos: v01.into(),
                            normal: n01.into(),
                            color: col0,
                        },
                    ]);

                    if edge.sharp_edge {
                        push_line(&mut out.lines, v00.into(), v10.into(), edge_line_col);
                    }
                }
            }

            // End caps at start and end rings
            let start_n = (-base_ex).into();
            let end_n = base_ex.into();
            let last = n_rings - 1;
            for tri in &caps {
                let s0 = ring_centers[0]
                    + ring_ey[0] * (tri[0][0] * pscale)
                    + ring_ez[0] * (tri[0][1] * pscale);
                let s1 = ring_centers[0]
                    + ring_ey[0] * (tri[1][0] * pscale)
                    + ring_ez[0] * (tri[1][1] * pscale);
                let s2 = ring_centers[0]
                    + ring_ey[0] * (tri[2][0] * pscale)
                    + ring_ez[0] * (tri[2][1] * pscale);
                out.opaque_tris.extend_from_slice(&[
                    MeshVertex {
                        pos: s0.into(),
                        normal: start_n,
                        color: ring_colors[0],
                    },
                    MeshVertex {
                        pos: s2.into(),
                        normal: start_n,
                        color: ring_colors[0],
                    },
                    MeshVertex {
                        pos: s1.into(),
                        normal: start_n,
                        color: ring_colors[0],
                    },
                ]);

                let e0 = ring_centers[last]
                    + ring_ey[last] * (tri[0][0] * pscale)
                    + ring_ez[last] * (tri[0][1] * pscale);
                let e1 = ring_centers[last]
                    + ring_ey[last] * (tri[1][0] * pscale)
                    + ring_ez[last] * (tri[1][1] * pscale);
                let e2 = ring_centers[last]
                    + ring_ey[last] * (tri[2][0] * pscale)
                    + ring_ez[last] * (tri[2][1] * pscale);
                out.opaque_tris.extend_from_slice(&[
                    MeshVertex {
                        pos: e0.into(),
                        normal: end_n,
                        color: ring_colors[last],
                    },
                    MeshVertex {
                        pos: e1.into(),
                        normal: end_n,
                        color: ring_colors[last],
                    },
                    MeshVertex {
                        pos: e2.into(),
                        normal: end_n,
                        color: ring_colors[last],
                    },
                ]);
            }
        } else {
            // Crisp centerline stick view
            for k in 0..ring_centers.len() - 1 {
                push_line(
                    &mut out.lines,
                    ring_centers[k].into(),
                    ring_centers[k + 1].into(),
                    ring_colors[k],
                );
            }
        }

        // Member end-release hinge indicators (white/amber octahedra inset slightly from node)
        let inset = (frame.length * 0.06).clamp(0.06, 0.24) as f32;
        if elem.has_releases_i() {
            let dir = (ring_centers.last().unwrap() - ring_centers[0]).normalize();
            let hp = ring_centers[0] + dir * inset;
            push_octahedron(&mut out.opaque_tris, hp, glyph_size * 0.28, [0.96, 0.96, 0.98, 1.0]);
        }
        if elem.has_releases_j() {
            let last_c = *ring_centers.last().unwrap();
            let dir = (ring_centers[0] - last_c).normalize();
            let hp = last_c + dir * inset;
            push_octahedron(&mut out.opaque_tris, hp, glyph_size * 0.28, [0.96, 0.96, 0.98, 1.0]);
        }

        // Local coordinate axis triad at element midpoint
        if opts.show_local_axes {
            let mid_idx = ring_centers.len() / 2;
            let mid_p = ring_centers[mid_idx];
            let triad_len = glyph_size * 1.4;
            push_line(
                &mut out.lines,
                mid_p.into(),
                (mid_p + base_ex * triad_len).into(),
                [0.96, 0.26, 0.26, 1.0],
            );
            push_line(
                &mut out.lines,
                mid_p.into(),
                (mid_p + ring_ey[mid_idx] * triad_len).into(),
                [0.24, 0.90, 0.38, 1.0],
            );
            push_line(
                &mut out.lines,
                mid_p.into(),
                (mid_p + ring_ez[mid_idx] * triad_len).into(),
                [0.28, 0.58, 0.98, 1.0],
            );
        }

        // 3. 3D Internal Force / Moment Diagram Ribbon
        if opts.diagram_ribbon != DiagramRibbon::None {
            if let (Some(sol), Some(er)) = (solution, elem_res) {
                build_element_diagram_ribbon(
                    opts.diagram_ribbon,
                    opts.diagram_scale,
                    char_len,
                    &sol.summary,
                    &er.stations,
                    &ring_centers,
                    base_ey,
                    base_ez,
                    out,
                );
            }
        }
    }

    // 4. Nodes & Support Glyphs
    for node in &model.nodes {
        let mut pos = Vector3::from(to_f32(node.position));
        if eff_disp_scale.abs() > 1e-9 {
            if let Some(nr) = solution.and_then(|s| s.node_results.get(&node.id)) {
                pos.x += (eff_disp_scale * nr.displacements[0]) as f32;
                pos.y += (eff_disp_scale * nr.displacements[1]) as f32;
                pos.z += (eff_disp_scale * nr.displacements[2]) as f32;
            }
        }

        let is_sel = opts.selected_node == Some(node.id)
            || opts.beam_draw_start_node == Some(node.id);
        let is_hov = opts.hovered_node == Some(node.id);

        let (radius, col) = if is_sel {
            (glyph_size * 0.42, [1.0, 0.80, 0.20, 1.0])
        } else if is_hov {
            (glyph_size * 0.38, [0.35, 0.90, 1.0, 1.0])
        } else {
            (glyph_size * 0.24, [0.85, 0.90, 0.96, 1.0])
        };
        push_octahedron(&mut out.opaque_tris, pos, radius, col);

        if opts.show_supports && node.is_supported() {
            let sup_pos = Vector3::from(to_f32(node.position));
            build_support_glyph(sup_pos, &node.restraints, glyph_size, out);
        }
    }

    // 4b. Render 2D Shell / Surface Elements (Tri3 & Quad4)
    build_shell_surfaces(
        model,
        solution,
        opts,
        eff_disp_scale,
        legend,
        glyph_size,
        out,
    );

    // 5. Applied Nodal, Beam Distributed, and 2D Surface Loads
    if opts.show_loads {
        build_load_glyphs(model, solution, eff_disp_scale, glyph_size, out);
    }

    // 6. Support Reaction Arrows
    if opts.show_reactions {
        if let Some(sol) = solution {
            build_reaction_glyphs(model, sol, glyph_size, out);
        }
    }

    // 7. Interactive AddNode / DrawBeam / DrawShell 3D Cursor Preview + Vertical Z Plumb-Line
    if let Some(pt) = opts.preview_point {
        let p = Vector3::from(to_f32(pt));
        push_octahedron(
            &mut out.opaque_tris,
            p,
            glyph_size * 0.36,
            [0.25, 0.92, 0.98, 0.92],
        );
        if pt[2].abs() > 1e-3 {
            // Vertical plumb-line to ground Z=0 so elevated 3D points align effortlessly over ground nodes
            push_line(
                &mut out.lines,
                [p.x, p.y, 0.0],
                p.into(),
                [0.25, 0.85, 0.98, 0.55],
            );
        }
        if let Some(start_id) = opts.beam_draw_start_node {
            if let Some(sn) = model.get_node(start_id) {
                push_line(
                    &mut out.lines,
                    to_f32(sn.position),
                    p.into(),
                    [0.25, 0.95, 0.85, 0.95],
                );
            }
        }
    }

    if !opts.shell_draw_points.is_empty() {
        let mut pts: Vec<Vector3<f32>> = opts
            .shell_draw_points
            .iter()
            .map(|&p| Vector3::from(to_f32(p)))
            .collect();
        for &p in &pts {
            push_octahedron(
                &mut out.opaque_tris,
                p,
                glyph_size * 0.34,
                [0.30, 0.95, 0.72, 1.0],
            );
        }
        if let Some(cur) = opts.preview_point {
            pts.push(Vector3::from(to_f32(cur)));
        }
        let line_col = [0.28, 0.96, 0.78, 0.95];
        for i in 0..pts.len().saturating_sub(1) {
            push_line(&mut out.lines, pts[i].into(), pts[i + 1].into(), line_col);
        }
        if pts.len() >= 3 {
            push_line(
                &mut out.lines,
                pts[pts.len() - 1].into(),
                pts[0].into(),
                [0.28, 0.96, 0.78, 0.55],
            );
            let n = (pts[1] - pts[0]).cross(&(pts[2] - pts[0])).normalize();
            let fill = [0.22, 0.84, 0.74, 0.32];
            for k in 1..(pts.len() - 1) {
                out.transparent_tris.extend_from_slice(&[
                    MeshVertex {
                        pos: pts[0].into(),
                        normal: n.into(),
                        color: fill,
                    },
                    MeshVertex {
                        pos: pts[k].into(),
                        normal: n.into(),
                        color: fill,
                    },
                    MeshVertex {
                        pos: pts[k + 1].into(),
                        normal: n.into(),
                        color: fill,
                    },
                ]);
            }
        }
    }

    legend
}

fn compute_legend_range(field: ColorField, solution: Option<&Solution>) -> Option<LegendRange> {
    let sol = solution?;
    let s = &sol.summary;
    match field {
        ColorField::SectionStyle => None,
        ColorField::DisplacementMag => Some(LegendRange {
            field,
            min_display: 0.0,
            max_display: (s.max_displacement_m * 1e3).max(1e-6),
            diverging: false,
        }),
        ColorField::AxialForce => {
            let peak = s.max_axial_n.abs().max(s.min_axial_n.abs()).max(1.0) * 1e-3;
            Some(LegendRange {
                field,
                min_display: -peak,
                max_display: peak,
                diverging: true,
            })
        }
        ColorField::ShearVz | ColorField::ShearVy => Some(LegendRange {
            field,
            min_display: 0.0,
            max_display: (s.max_shear_n * 1e-3).max(1e-3),
            diverging: false,
        }),
        ColorField::MomentMy | ColorField::MomentMz => {
            let shell_m = sol
                .shell_results
                .values()
                .map(|sr| sr.bending_moments[0].hypot(sr.bending_moments[1]))
                .fold(0.0_f64, f64::max);
            Some(LegendRange {
                field,
                min_display: 0.0,
                max_display: (s.max_moment_nm.max(shell_m) * 1e-3).max(1e-3),
                diverging: false,
            })
        }
        ColorField::TorqueT => Some(LegendRange {
            field,
            min_display: 0.0,
            max_display: (s.max_torque_nm * 1e-3).max(1e-3),
            diverging: false,
        }),
        ColorField::NormalStress | ColorField::VonMisesStress => {
            let mut max_pa = s.max_stress_pa;
            if field == ColorField::VonMisesStress {
                let beam_vm = sol
                    .element_results
                    .values()
                    .flat_map(|er| er.stations.iter().map(|st| st.von_mises_stress))
                    .fold(0.0_f64, f64::max);
                let shell_vm = sol
                    .shell_results
                    .values()
                    .map(|sr| sr.max_von_mises_pa)
                    .fold(0.0_f64, f64::max);
                max_pa = beam_vm.max(shell_vm);
            }
            Some(LegendRange {
                field,
                min_display: 0.0,
                max_display: (max_pa * 1e-6).max(1e-3),
                diverging: false,
            })
        }
        ColorField::Utilization => Some(LegendRange {
            field,
            min_display: 0.0,
            max_display: (s.max_utilization * 100.0).max(100.0),
            diverging: false,
        }),
    }
}

fn sample_color(
    field: ColorField,
    st: &StationSample,
    fy: f64,
    section_id: usize,
    legend: Option<LegendRange>,
) -> [f32; 4] {
    if field == ColorField::SectionStyle {
        return section_palette(section_id);
    }
    let Some(leg) = legend else {
        return section_palette(section_id);
    };
    match field {
        ColorField::SectionStyle => section_palette(section_id),
        ColorField::DisplacementMag => {
            let u_mm = (st.disp_global[0].powi(2)
                + st.disp_global[1].powi(2)
                + st.disp_global[2].powi(2))
            .sqrt()
                * 1e3;
            colormap_scientific((u_mm / leg.max_display) as f32)
        }
        ColorField::AxialForce => {
            let n_kn = st.axial_n * 1e-3;
            colormap_diverging((n_kn / leg.max_display) as f32)
        }
        ColorField::ShearVz => colormap_scientific(((st.shear_vz.abs() * 1e-3) / leg.max_display) as f32),
        ColorField::ShearVy => colormap_scientific(((st.shear_vy.abs() * 1e-3) / leg.max_display) as f32),
        ColorField::MomentMy => {
            colormap_scientific(((st.moment_my.abs() * 1e-3) / leg.max_display) as f32)
        }
        ColorField::MomentMz => {
            colormap_scientific(((st.moment_mz.abs() * 1e-3) / leg.max_display) as f32)
        }
        ColorField::TorqueT => {
            colormap_scientific(((st.torque_t.abs() * 1e-3) / leg.max_display) as f32)
        }
        ColorField::NormalStress => {
            colormap_scientific(((st.max_normal_stress * 1e-6) / leg.max_display) as f32)
        }
        ColorField::VonMisesStress => {
            colormap_scientific(((st.von_mises_stress * 1e-6) / leg.max_display) as f32)
        }
        ColorField::Utilization => {
            let util_pct = (st.max_normal_stress / fy) * 100.0;
            colormap_scientific((util_pct / 100.0) as f32)
        }
    }
}

fn rotate_section_axes(
    r_mat: &Matrix3<f64>,
    base_ey: Vector3<f32>,
    base_ez: Vector3<f32>,
    rot_local: [f64; 3],
    scale: f64,
) -> (Vector3<f32>, Vector3<f32>) {
    let rot_vec_local = nalgebra::Vector3::new(
        rot_local[0] * scale,
        rot_local[1] * scale,
        rot_local[2] * scale,
    );
    let rot_vec_global = r_mat.transpose() * rot_vec_local;
    let angle = rot_vec_global.norm() as f32;
    if angle < 1e-6 {
        return (base_ey, base_ez);
    }
    let axis = Unit::new_normalize(Vector3::new(
        rot_vec_global[0] as f32,
        rot_vec_global[1] as f32,
        rot_vec_global[2] as f32,
    ));
    let rot = Rotation3::from_axis_angle(&axis, angle.clamp(-1.4, 1.4));
    (rot * base_ey, rot * base_ez)
}

#[allow(clippy::too_many_arguments)]
fn build_element_diagram_ribbon(
    ribbon: DiagramRibbon,
    diagram_scale: f32,
    char_len: f32,
    summary: &beam3d_core::AnalysisSummary,
    stations: &[StationSample],
    centers: &[Vector3<f32>],
    base_ey: Vector3<f32>,
    base_ez: Vector3<f32>,
    out: &mut SceneGeometry,
) {
    if stations.len() < 2 || stations.len() != centers.len() {
        return;
    }
    let max_world_height = char_len * 0.11 * diagram_scale;
    let (dir_vec, peak_val): (Vector3<f32>, f64) = match ribbon {
        DiagramRibbon::None => return,
        // Plot sagging positive My downward (-ez) per structural engineering convention, or +ez;
        // Using -base_ez draws sagging moment on tension side (bottom of girder)!
        DiagramRibbon::MomentMy => (-base_ez, summary.max_moment_nm.max(1.0)),
        DiagramRibbon::MomentMz => (base_ey, summary.max_moment_nm.max(1.0)),
        DiagramRibbon::ShearVz => (base_ez, summary.max_shear_n.max(1.0)),
        DiagramRibbon::ShearVy => (base_ey, summary.max_shear_n.max(1.0)),
        DiagramRibbon::AxialN => (
            base_ez,
            summary.max_axial_n.abs().max(summary.min_axial_n.abs()).max(1.0),
        ),
        DiagramRibbon::TorqueT => (base_ez, summary.max_torque_nm.max(1.0)),
    };

    let val_of = |st: &StationSample| -> f64 {
        match ribbon {
            DiagramRibbon::None => 0.0,
            DiagramRibbon::MomentMy => st.moment_my,
            DiagramRibbon::MomentMz => st.moment_mz,
            DiagramRibbon::ShearVz => st.shear_vz,
            DiagramRibbon::ShearVy => st.shear_vy,
            DiagramRibbon::AxialN => st.axial_n,
            DiagramRibbon::TorqueT => st.torque_t,
        }
    };

    let ex_approx = (centers[centers.len() - 1] - centers[0]).normalize();
    let ribbon_normal = ex_approx.cross(&dir_vec).normalize();

    let mut tip_pts = Vec::with_capacity(stations.len());
    let mut tip_cols = Vec::with_capacity(stations.len());

    for (k, st) in stations.iter().enumerate() {
        let v = val_of(st);
        let norm_signed = (v / peak_val).clamp(-1.0, 1.0) as f32;
        let tip = centers[k] + dir_vec * (norm_signed * max_world_height);
        let mut c = colormap_diverging(norm_signed);
        c[3] = 0.56;
        tip_pts.push(tip);
        tip_cols.push(c);
    }

    for k in 0..stations.len() - 1 {
        let b0 = centers[k];
        let b1 = centers[k + 1];
        let t0 = tip_pts[k];
        let t1 = tip_pts[k + 1];
        let c0 = tip_cols[k];
        let c1 = tip_cols[k + 1];

        out.transparent_tris.extend_from_slice(&[
            MeshVertex {
                pos: b0.into(),
                normal: ribbon_normal.into(),
                color: c0,
            },
            MeshVertex {
                pos: b1.into(),
                normal: ribbon_normal.into(),
                color: c1,
            },
            MeshVertex {
                pos: t1.into(),
                normal: ribbon_normal.into(),
                color: c1,
            },
            MeshVertex {
                pos: b0.into(),
                normal: ribbon_normal.into(),
                color: c0,
            },
            MeshVertex {
                pos: t1.into(),
                normal: ribbon_normal.into(),
                color: c1,
            },
            MeshVertex {
                pos: t0.into(),
                normal: ribbon_normal.into(),
                color: c0,
            },
        ]);

        let mut line_c = tip_cols[k];
        line_c[3] = 0.95;
        push_line(&mut out.lines, t0.into(), t1.into(), line_c);
        if k % 4 == 0 || k + 2 == stations.len() {
            let mut tick_c = line_c;
            tick_c[3] = 0.55;
            push_line(&mut out.lines, b0.into(), t0.into(), tick_c);
        }
    }
    let last = stations.len() - 1;
    let mut end_c = tip_cols[last];
    end_c[3] = 0.75;
    push_line(
        &mut out.lines,
        centers[0].into(),
        tip_pts[0].into(),
        tip_cols[0],
    );
    push_line(&mut out.lines, centers[last].into(), tip_pts[last].into(), end_c);
}

fn build_workplane_grid(
    model: &StructuralModel,
    kind: WorkplaneKind,
    plane_offset: f32,
    out: &mut SceneGeometry,
) {
    let (min_p, max_p) = model.bounding_box();
    let cx = if min_p[0].is_finite() {
        (0.5 * (min_p[0] + max_p[0]) as f32).round()
    } else {
        0.0
    };
    let cy = if min_p[1].is_finite() {
        (0.5 * (min_p[1] + max_p[1]) as f32).round()
    } else {
        0.0
    };
    let max_z = if max_p[2].is_finite() {
        (max_p[2] as f32).max(8.0) + 4.0
    } else {
        10.0
    };
    let span = ((max_p[0] - min_p[0])
        .max(max_p[1] - min_p[1])
        .max(max_p[2] - min_p[2]) as f32
        * 0.85)
        .clamp(8.0, 60.0)
        .ceil();
    let step = if span > 30.0 { 2.0 } else { 1.0 };
    let half_steps = (span / step).ceil() as i32;
    let ext = half_steps as f32 * step;

    // When a vertical plane (XZ or YZ) is active, keep a subtle Z=0 ground reference grid
    // so floor orientation in 3D space stays crystal-clear.
    if kind != WorkplaneKind::XY {
        for i in (-half_steps..=half_steps).step_by(2) {
            let off = i as f32 * step;
            let faint = [0.16, 0.19, 0.24, 0.24];
            push_line(&mut out.lines, [cx - ext, cy + off, 0.0], [cx + ext, cy + off, 0.0], faint);
            push_line(&mut out.lines, [cx + off, cy - ext, 0.0], [cx + off, cy + ext, 0.0], faint);
        }
    }

    match kind {
        WorkplaneKind::XY => {
            for i in -half_steps..=half_steps {
                let off = i as f32 * step;
                let is_major = i % 5 == 0;
                let col = if is_major {
                    [0.24, 0.30, 0.39, 0.68]
                } else {
                    [0.16, 0.20, 0.27, 0.44]
                };
                let y = cy + off;
                push_line(
                    &mut out.lines,
                    [cx - ext, y, plane_offset],
                    [cx + ext, y, plane_offset],
                    col,
                );
                let x = cx + off;
                push_line(
                    &mut out.lines,
                    [x, cy - ext, plane_offset],
                    [x, cy + ext, plane_offset],
                    col,
                );
            }
        }
        WorkplaneKind::XZ => {
            let z_steps_min = -2i32;
            let z_steps_max = (max_z / step).ceil() as i32;
            let z_min = z_steps_min as f32 * step;
            let z_max = z_steps_max as f32 * step;

            // Vertical lines along Z at each X
            for i in -half_steps..=half_steps {
                let x = cx + i as f32 * step;
                let col = if i % 5 == 0 {
                    [0.22, 0.44, 0.72, 0.72]
                } else {
                    [0.16, 0.30, 0.48, 0.44]
                };
                push_line(
                    &mut out.lines,
                    [x, plane_offset, z_min],
                    [x, plane_offset, z_max],
                    col,
                );
            }
            // Horizontal elevation lines at each Z
            for iz in z_steps_min..=z_steps_max {
                let z = iz as f32 * step;
                let col = if iz == 0 {
                    [0.38, 0.72, 1.0, 0.92]
                } else if iz % 5 == 0 {
                    [0.22, 0.44, 0.72, 0.72]
                } else {
                    [0.16, 0.30, 0.48, 0.44]
                };
                push_line(
                    &mut out.lines,
                    [cx - ext, plane_offset, z],
                    [cx + ext, plane_offset, z],
                    col,
                );
            }
        }
        WorkplaneKind::YZ => {
            let z_steps_min = -2i32;
            let z_steps_max = (max_z / step).ceil() as i32;
            let z_min = z_steps_min as f32 * step;
            let z_max = z_steps_max as f32 * step;

            // Vertical lines along Z at each Y
            for i in -half_steps..=half_steps {
                let y = cy + i as f32 * step;
                let col = if i % 5 == 0 {
                    [0.20, 0.62, 0.48, 0.72]
                } else {
                    [0.15, 0.40, 0.32, 0.44]
                };
                push_line(
                    &mut out.lines,
                    [plane_offset, y, z_min],
                    [plane_offset, y, z_max],
                    col,
                );
            }
            // Horizontal elevation lines at each Z
            for iz in z_steps_min..=z_steps_max {
                let z = iz as f32 * step;
                let col = if iz == 0 {
                    [0.34, 0.92, 0.68, 0.92]
                } else if iz % 5 == 0 {
                    [0.20, 0.62, 0.48, 0.72]
                } else {
                    [0.15, 0.40, 0.32, 0.44]
                };
                push_line(
                    &mut out.lines,
                    [plane_offset, cy - ext, z],
                    [plane_offset, cy + ext, z],
                    col,
                );
            }
        }
    }

    // World origin coordinate axes (+X Red, +Y Green, +Z Blue)
    let axis_len = (span * 0.18).clamp(1.2, 4.0);
    push_line(&mut out.lines, [0.0, 0.0, 0.0], [axis_len, 0.0, 0.0], [0.95, 0.28, 0.28, 0.95]);
    push_line(&mut out.lines, [0.0, 0.0, 0.0], [0.0, axis_len, 0.0], [0.28, 0.88, 0.40, 0.95]);
    push_line(&mut out.lines, [0.0, 0.0, 0.0], [0.0, 0.0, axis_len], [0.32, 0.60, 0.98, 0.95]);
}

fn build_support_glyph(
    pos: Vector3<f32>,
    restraints: &[bool; 6],
    s: f32,
    out: &mut SceneGeometry,
) {
    let preset = SupportPreset::from_restraints(restraints);
    match preset {
        SupportPreset::Free => {}
        SupportPreset::Fixed => {
            let h = s * 0.55;
            let w = s * 1.15;
            push_box(
                &mut out.opaque_tris,
                pos - Vector3::new(0.0, 0.0, h * 0.5),
                Vector3::new(w, w, h),
                [0.26, 0.78, 0.68, 1.0],
            );
        }
        SupportPreset::Pinned => {
            push_pyramid(
                &mut out.opaque_tris,
                pos,
                s * 0.95,
                s * 0.95,
                [0.26, 0.82, 0.52, 1.0],
            );
        }
        SupportPreset::RollerZ => {
            push_pyramid(
                &mut out.opaque_tris,
                pos,
                s * 0.85,
                s * 0.78,
                [0.34, 0.76, 0.94, 1.0],
            );
            push_box(
                &mut out.opaque_tris,
                pos - Vector3::new(0.0, 0.0, s * 0.92),
                Vector3::new(s * 1.1, s * 1.1, s * 0.16),
                [0.34, 0.76, 0.94, 1.0],
            );
        }
        SupportPreset::Custom => {
            push_pyramid(
                &mut out.opaque_tris,
                pos,
                s * 0.85,
                s * 0.85,
                [0.86, 0.66, 0.28, 1.0],
            );
        }
    }
}

fn build_load_glyphs(
    model: &StructuralModel,
    solution: Option<&Solution>,
    disp_scale: f64,
    glyph_size: f32,
    out: &mut SceneGeometry,
) {
    let max_nodal_f = model
        .nodal_loads
        .iter()
        .map(|nl| (nl.forces[0].powi(2) + nl.forces[1].powi(2) + nl.forces[2].powi(2)).sqrt())
        .fold(0.0_f64, f64::max)
        .max(1.0);

    let max_nodal_m = model
        .nodal_loads
        .iter()
        .map(|nl| (nl.moments[0].powi(2) + nl.moments[1].powi(2) + nl.moments[2].powi(2)).sqrt())
        .fold(0.0_f64, f64::max)
        .max(1.0);

    for nl in &model.nodal_loads {
        let Some(node) = model.get_node(nl.node_id) else {
            continue;
        };
        let mut p = Vector3::from(to_f32(node.position));
        if disp_scale.abs() > 1e-9 {
            if let Some(nr) = solution.and_then(|s| s.node_results.get(&node.id)) {
                p.x += (disp_scale * nr.displacements[0]) as f32;
                p.y += (disp_scale * nr.displacements[1]) as f32;
                p.z += (disp_scale * nr.displacements[2]) as f32;
            }
        }

        let f_vec = Vector3::new(nl.forces[0] as f32, nl.forces[1] as f32, nl.forces[2] as f32);
        let f_mag = f_vec.norm();
        if f_mag > 1e-4 {
            let dir = f_vec / f_mag;
            let len = glyph_size * (1.8 + 2.4 * ((f_mag as f64 / max_nodal_f) as f32).sqrt());
            let tail = p - dir * len;
            push_arrow_3d(
                &mut out.opaque_tris,
                tail,
                p - dir * (glyph_size * 0.22),
                glyph_size * 0.12,
                [0.98, 0.36, 0.22, 1.0],
            );
        }

        let m_vec = Vector3::new(nl.moments[0] as f32, nl.moments[1] as f32, nl.moments[2] as f32);
        let m_mag = m_vec.norm();
        if m_mag > 1e-4 {
            let dir = m_vec / m_mag;
            let len = glyph_size * (1.6 + 1.8 * ((m_mag as f64 / max_nodal_m) as f32).sqrt());
            push_arrow_3d(
                &mut out.opaque_tris,
                p,
                p + dir * len,
                glyph_size * 0.13,
                [0.88, 0.34, 0.92, 1.0],
            );
        }
    }

    // Distributed loads comb
    let max_q = model
        .distributed_loads
        .iter()
        .map(|dl| {
            let ms = (dl.q_start[0].powi(2) + dl.q_start[1].powi(2) + dl.q_start[2].powi(2)).sqrt();
            let me = (dl.q_end[0].powi(2) + dl.q_end[1].powi(2) + dl.q_end[2].powi(2)).sqrt();
            ms.max(me)
        })
        .fold(0.0_f64, f64::max)
        .max(1.0);

    for dl in &model.distributed_loads {
        let Some(elem) = model.get_element(dl.element_id) else {
            continue;
        };
        let (Some(ni), Some(nj)) = (model.get_node(elem.node_i), model.get_node(elem.node_j))
        else {
            continue;
        };
        let Ok(frame) = ElementFrame::compute(ni.position, nj.position, elem.roll_angle_deg) else {
            continue;
        };

        let r_t = frame.r_mat.transpose();
        let q0_glob = match dl.coord_system {
            LoadCoordSystem::Global => nalgebra::Vector3::from(dl.q_start),
            LoadCoordSystem::Local => r_t * nalgebra::Vector3::from(dl.q_start),
        };
        let q1_glob = match dl.coord_system {
            LoadCoordSystem::Global => nalgebra::Vector3::from(dl.q_end),
            LoadCoordSystem::Local => r_t * nalgebra::Vector3::from(dl.q_end),
        };

        let n_arrows = ((frame.length / 1.1).round() as usize).clamp(4, 10);
        let p0 = Vector3::from(to_f32(ni.position));
        let p1 = Vector3::from(to_f32(nj.position));
        let comb_col = [0.98, 0.68, 0.18, 0.92];
        let mut prev_tail: Option<Vector3<f32>> = None;

        for k in 0..=n_arrows {
            let t = k as f64 / n_arrows as f64;
            let q = q0_glob * (1.0 - t) + q1_glob * t;
            let q_mag = q.norm();
            let base = p0 + (p1 - p0) * (t as f32);
            if q_mag > 1e-3 {
                let dir = Vector3::new(
                    (q.x / q_mag) as f32,
                    (q.y / q_mag) as f32,
                    (q.z / q_mag) as f32,
                );
                let h = glyph_size * (0.8 + 1.9 * (q_mag / max_q) as f32);
                let tail = base - dir * h;
                push_arrow_3d(
                    &mut out.opaque_tris,
                    tail,
                    base - dir * (glyph_size * 0.12),
                    glyph_size * 0.075,
                    comb_col,
                );
                if let Some(pt) = prev_tail {
                    push_line(&mut out.lines, pt.into(), tail.into(), comb_col);
                }
                prev_tail = Some(tail);
            } else {
                prev_tail = None;
            }
        }
    }

    // 2D Surface pressure load arrows at shell centroids
    if !model.surface_loads.is_empty() {
        let max_p = model
            .surface_loads
            .iter()
            .map(|sl| {
                (sl.pressure[0].powi(2) + sl.pressure[1].powi(2) + sl.pressure[2].powi(2)).sqrt()
            })
            .fold(0.0_f64, f64::max)
            .max(1.0);

        for sl in &model.surface_loads {
            let Some(shell) = model.get_shell(sl.shell_id) else {
                continue;
            };
            let ids = shell.nodes.as_slice();
            let mut raw_pos = Vec::with_capacity(ids.len());
            let mut def_center = Vector3::<f32>::zeros();
            let mut ok = true;
            for &nid in ids {
                if let Some(n) = model.get_node(nid) {
                    raw_pos.push(n.position);
                    let mut p = Vector3::from(to_f32(n.position));
                    if disp_scale.abs() > 1e-9 {
                        if let Some(nr) = solution.and_then(|s| s.node_results.get(&nid)) {
                            p.x += (disp_scale * nr.displacements[0]) as f32;
                            p.y += (disp_scale * nr.displacements[1]) as f32;
                            p.z += (disp_scale * nr.displacements[2]) as f32;
                        }
                    }
                    def_center += p;
                } else {
                    ok = false;
                    break;
                }
            }
            if !ok {
                continue;
            }
            def_center /= ids.len() as f32;
            let Ok(frame) = beam3d_core::ShellFrame::compute(&raw_pos) else {
                continue;
            };
            let p_glob = match sl.coord_system {
                LoadCoordSystem::Global => nalgebra::Vector3::from(sl.pressure),
                LoadCoordSystem::Local => frame.r_mat.transpose() * nalgebra::Vector3::from(sl.pressure),
            };
            let p_mag = p_glob.norm();
            if p_mag > 1e-3 {
                let dir = Vector3::new(
                    (p_glob.x / p_mag) as f32,
                    (p_glob.y / p_mag) as f32,
                    (p_glob.z / p_mag) as f32,
                );
                let h = glyph_size * (0.95 + 1.55 * ((p_mag / max_p) as f32).sqrt());
                let tail = def_center - dir * h;
                push_arrow_3d(
                    &mut out.opaque_tris,
                    tail,
                    def_center - dir * (glyph_size * 0.10),
                    glyph_size * 0.085,
                    [0.98, 0.56, 0.24, 1.0],
                );
            }
        }
    }
}

fn build_shell_surfaces(
    model: &StructuralModel,
    solution: Option<&Solution>,
    opts: &ViewportDisplayOptions,
    eff_disp_scale: f64,
    legend: Option<LegendRange>,
    glyph_size: f32,
    out: &mut SceneGeometry,
) {
    for shell in &model.shells {
        let ids = shell.nodes.as_slice();
        let mut pts = Vec::with_capacity(ids.len());
        let mut undef_pts = Vec::with_capacity(ids.len());
        let mut corner_umag = Vec::with_capacity(ids.len());
        let mut ok = true;

        for &nid in ids {
            let Some(n) = model.get_node(nid) else {
                ok = false;
                break;
            };
            undef_pts.push(Vector3::from(to_f32(n.position)));
            let mut p = Vector3::from(to_f32(n.position));
            let mut umag_mm = 0.0_f64;
            if let Some(nr) = solution.and_then(|s| s.node_results.get(&nid)) {
                let d = nr.displacements;
                umag_mm = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() * 1e3;
                if eff_disp_scale.abs() > 1e-9 {
                    p.x += (eff_disp_scale * d[0]) as f32;
                    p.y += (eff_disp_scale * d[1]) as f32;
                    p.z += (eff_disp_scale * d[2]) as f32;
                }
            }
            pts.push(p);
            corner_umag.push(umag_mm);
        }
        if !ok || pts.len() < 3 {
            continue;
        }

        // Optional undeformed ghost wireframe outline for shells
        if eff_disp_scale.abs() > 1e-9 && opts.show_undeformed_ghost {
            for k in 0..undef_pts.len() {
                push_line(
                    &mut out.lines,
                    undef_pts[k].into(),
                    undef_pts[(k + 1) % undef_pts.len()].into(),
                    [0.40, 0.45, 0.54, 0.30],
                );
            }
        }

        let is_selected = opts.selected_shell == Some(shell.id);
        let is_hovered = opts.hovered_shell == Some(shell.id);
        let sr = solution.and_then(|s| s.shell_results.get(&shell.id));

        let base_col = [0.56_f32, 0.62, 0.70, 0.96];
        let uniform_shell_col = match (opts.color_field, sr, legend) {
            (ColorField::VonMisesStress, Some(r), Some(leg)) => {
                colormap_scientific(((r.max_von_mises_pa * 1e-6) / leg.max_display) as f32)
            }
            (ColorField::NormalStress, Some(r), Some(leg)) => {
                colormap_scientific(((r.max_normal_stress_pa * 1e-6) / leg.max_display) as f32)
            }
            (ColorField::Utilization, Some(r), _) => {
                colormap_scientific(r.utilization as f32)
            }
            (ColorField::MomentMy | ColorField::MomentMz, Some(r), Some(leg)) => {
                let m_knm = r.bending_moments[0].hypot(r.bending_moments[1]) * 1e-3;
                colormap_scientific((m_knm / leg.max_display) as f32)
            }
            _ => base_col,
        };

        let corner_color = |idx: usize| -> [f32; 4] {
            let mut c = if opts.color_field == ColorField::DisplacementMag {
                if let Some(leg) = legend {
                    colormap_scientific((corner_umag[idx] / leg.max_display) as f32)
                } else {
                    base_col
                }
            } else {
                uniform_shell_col
            };
            if is_selected {
                c = mix_color(c, [1.0, 0.80, 0.22, 1.0], 0.45);
            } else if is_hovered {
                c = mix_color(c, [0.45, 0.90, 1.0, 1.0], 0.30);
            }
            c
        };

        // Triangulate Tri3 (1 tri) or Quad4 (2 tris)
        let tri_indices: &[[usize; 3]] = if pts.len() == 3 {
            &[[0, 1, 2]]
        } else {
            &[[0, 1, 2], [0, 2, 3]]
        };

        for &[a, b, c_idx] in tri_indices {
            let normal = (pts[b] - pts[a]).cross(&(pts[c_idx] - pts[a]));
            let n = if normal.norm() > 1e-6 {
                normal.normalize()
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };
            out.opaque_tris.extend_from_slice(&[
                MeshVertex {
                    pos: pts[a].into(),
                    normal: n.into(),
                    color: corner_color(a),
                },
                MeshVertex {
                    pos: pts[b].into(),
                    normal: n.into(),
                    color: corner_color(b),
                },
                MeshVertex {
                    pos: pts[c_idx].into(),
                    normal: n.into(),
                    color: corner_color(c_idx),
                },
            ]);
        }

        // Crisp mesh edge lines
        let edge_col = if is_selected {
            [1.0, 0.86, 0.25, 1.0]
        } else if is_hovered {
            [0.55, 0.92, 1.0, 0.95]
        } else {
            [0.14, 0.18, 0.24, 0.78]
        };
        for k in 0..pts.len() {
            push_line(
                &mut out.lines,
                pts[k].into(),
                pts[(k + 1) % pts.len()].into(),
                edge_col,
            );
        }

        // Optional local outward normal indicator at shell centroid
        if opts.show_local_axes {
            let mut c_pos = Vector3::<f32>::zeros();
            for p in &pts {
                c_pos += *p;
            }
            c_pos /= pts.len() as f32;
            let n = (pts[1] - pts[0]).cross(&(pts[2] - pts[0]));
            if n.norm() > 1e-6 {
                let n = n.normalize();
                push_line(
                    &mut out.lines,
                    c_pos.into(),
                    (c_pos + n * (glyph_size * 1.1)).into(),
                    [0.32, 0.65, 1.0, 0.95],
                );
            }
        }
    }
}

fn build_reaction_glyphs(
    model: &StructuralModel,
    sol: &Solution,
    glyph_size: f32,
    out: &mut SceneGeometry,
) {
    let max_r = sol
        .node_results
        .values()
        .map(|nr| (nr.reactions[0].powi(2) + nr.reactions[1].powi(2) + nr.reactions[2].powi(2)).sqrt())
        .fold(0.0_f64, f64::max)
        .max(1.0);

    for node in &model.nodes {
        if !node.is_supported() {
            continue;
        }
        let Some(nr) = sol.node_results.get(&node.id) else {
            continue;
        };
        let r_vec = Vector3::new(
            nr.reactions[0] as f32,
            nr.reactions[1] as f32,
            nr.reactions[2] as f32,
        );
        let r_mag = r_vec.norm();
        if r_mag > 1e-2 {
            let dir = r_vec / r_mag;
            let p = Vector3::from(to_f32(node.position));
            let len = glyph_size * (1.5 + 2.2 * ((r_mag as f64 / max_r) as f32).sqrt());
            let tail = p - dir * (len + glyph_size * 0.45);
            let tip = p - dir * (glyph_size * 0.45);
            push_arrow_3d(
                &mut out.opaque_tris,
                tail,
                tip,
                glyph_size * 0.12,
                [0.22, 0.92, 0.52, 1.0],
            );
        }
    }
}

fn push_line(lines: &mut Vec<LineVertex>, a: [f32; 3], b: [f32; 3], color: [f32; 4]) {
    lines.push(LineVertex { pos: a, color });
    lines.push(LineVertex { pos: b, color });
}

fn push_octahedron(tris: &mut Vec<MeshVertex>, center: Vector3<f32>, r: f32, color: [f32; 4]) {
    let px = center + Vector3::new(r, 0.0, 0.0);
    let nx = center - Vector3::new(r, 0.0, 0.0);
    let py = center + Vector3::new(0.0, r, 0.0);
    let ny = center - Vector3::new(0.0, r, 0.0);
    let pz = center + Vector3::new(0.0, 0.0, r);
    let nz = center - Vector3::new(0.0, 0.0, r);

    let faces = [
        [px, py, pz],
        [py, nx, pz],
        [nx, ny, pz],
        [ny, px, pz],
        [py, px, nz],
        [nx, py, nz],
        [ny, nx, nz],
        [px, ny, nz],
    ];
    for f in &faces {
        let n = (f[1] - f[0]).cross(&(f[2] - f[0])).normalize();
        for v in f {
            tris.push(MeshVertex {
                pos: (*v).into(),
                normal: n.into(),
                color,
            });
        }
    }
}

fn push_pyramid(
    tris: &mut Vec<MeshVertex>,
    apex: Vector3<f32>,
    base_width: f32,
    height: f32,
    color: [f32; 4],
) {
    let hw = 0.5 * base_width;
    let bz = apex.z - height;
    let b0 = Vector3::new(apex.x - hw, apex.y - hw, bz);
    let b1 = Vector3::new(apex.x + hw, apex.y - hw, bz);
    let b2 = Vector3::new(apex.x + hw, apex.y + hw, bz);
    let b3 = Vector3::new(apex.x - hw, apex.y + hw, bz);

    let sides = [[apex, b0, b1], [apex, b1, b2], [apex, b2, b3], [apex, b3, b0]];
    for f in &sides {
        let n = (f[1] - f[0]).cross(&(f[2] - f[0])).normalize();
        for v in f {
            tris.push(MeshVertex {
                pos: (*v).into(),
                normal: n.into(),
                color,
            });
        }
    }
    let bn = [0.0, 0.0, -1.0];
    for f in &[[b0, b2, b1], [b0, b3, b2]] {
        for v in f {
            tris.push(MeshVertex {
                pos: (*v).into(),
                normal: bn,
                color,
            });
        }
    }
}

fn push_box(
    tris: &mut Vec<MeshVertex>,
    center: Vector3<f32>,
    size: Vector3<f32>,
    color: [f32; 4],
) {
    let h = size * 0.5;
    let quads: [([Vector3<f32>; 4], [f32; 3]); 6] = [
        (
            [
                center + Vector3::new(h.x, -h.y, -h.z),
                center + Vector3::new(h.x, h.y, -h.z),
                center + Vector3::new(h.x, h.y, h.z),
                center + Vector3::new(h.x, -h.y, h.z),
            ],
            [1.0, 0.0, 0.0],
        ),
        (
            [
                center + Vector3::new(-h.x, h.y, -h.z),
                center + Vector3::new(-h.x, -h.y, -h.z),
                center + Vector3::new(-h.x, -h.y, h.z),
                center + Vector3::new(-h.x, h.y, h.z),
            ],
            [-1.0, 0.0, 0.0],
        ),
        (
            [
                center + Vector3::new(h.x, h.y, -h.z),
                center + Vector3::new(-h.x, h.y, -h.z),
                center + Vector3::new(-h.x, h.y, h.z),
                center + Vector3::new(h.x, h.y, h.z),
            ],
            [0.0, 1.0, 0.0],
        ),
        (
            [
                center + Vector3::new(-h.x, -h.y, -h.z),
                center + Vector3::new(h.x, -h.y, -h.z),
                center + Vector3::new(h.x, -h.y, h.z),
                center + Vector3::new(-h.x, -h.y, h.z),
            ],
            [0.0, -1.0, 0.0],
        ),
        (
            [
                center + Vector3::new(-h.x, -h.y, h.z),
                center + Vector3::new(h.x, -h.y, h.z),
                center + Vector3::new(h.x, h.y, h.z),
                center + Vector3::new(-h.x, h.y, h.z),
            ],
            [0.0, 0.0, 1.0],
        ),
        (
            [
                center + Vector3::new(-h.x, h.y, -h.z),
                center + Vector3::new(h.x, h.y, -h.z),
                center + Vector3::new(h.x, -h.y, -h.z),
                center + Vector3::new(-h.x, -h.y, -h.z),
            ],
            [0.0, 0.0, -1.0],
        ),
    ];

    for (q, n) in &quads {
        for idx in [0, 1, 2, 0, 2, 3] {
            tris.push(MeshVertex {
                pos: q[idx].into(),
                normal: *n,
                color,
            });
        }
    }
}

fn push_arrow_3d(
    tris: &mut Vec<MeshVertex>,
    tail: Vector3<f32>,
    tip: Vector3<f32>,
    shaft_radius: f32,
    color: [f32; 4],
) {
    let delta = tip - tail;
    let total_len = delta.norm();
    if total_len < 1e-4 {
        return;
    }
    let dir = delta / total_len;
    let ref_up = if dir.z.abs() < 0.95 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let u = dir.cross(&ref_up).normalize();
    let v = dir.cross(&u).normalize();

    let head_len = (shaft_radius * 4.5).min(total_len * 0.45);
    let head_rad = shaft_radius * 2.5;
    let neck = tip - dir * head_len;

    let n_seg = 8usize;
    for k in 0..n_seg {
        let a0 = (k as f32 / n_seg as f32) * std::f32::consts::TAU;
        let a1 = ((k + 1) as f32 / n_seg as f32) * std::f32::consts::TAU;
        let n0 = u * a0.cos() + v * a0.sin();
        let n1 = u * a1.cos() + v * a1.sin();

        // Shaft quad
        let s00 = tail + n0 * shaft_radius;
        let s01 = tail + n1 * shaft_radius;
        let s10 = neck + n0 * shaft_radius;
        let s11 = neck + n1 * shaft_radius;
        tris.extend_from_slice(&[
            MeshVertex {
                pos: s00.into(),
                normal: n0.into(),
                color,
            },
            MeshVertex {
                pos: s10.into(),
                normal: n0.into(),
                color,
            },
            MeshVertex {
                pos: s11.into(),
                normal: n1.into(),
                color,
            },
            MeshVertex {
                pos: s00.into(),
                normal: n0.into(),
                color,
            },
            MeshVertex {
                pos: s11.into(),
                normal: n1.into(),
                color,
            },
            MeshVertex {
                pos: s01.into(),
                normal: n1.into(),
                color,
            },
        ]);

        // Cone head triangle
        let c0 = neck + n0 * head_rad;
        let c1 = neck + n1 * head_rad;
        let cone_n = (n0 + n1 + dir * 0.5).normalize();
        tris.extend_from_slice(&[
            MeshVertex {
                pos: c0.into(),
                normal: cone_n.into(),
                color,
            },
            MeshVertex {
                pos: tip.into(),
                normal: cone_n.into(),
                color,
            },
            MeshVertex {
                pos: c1.into(),
                normal: cone_n.into(),
                color,
            },
        ]);
    }
}

#[inline]
fn to_f32(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

#[inline]
fn mix_color(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
        a[3] + t * (b[3] - a[3]),
    ]
}
