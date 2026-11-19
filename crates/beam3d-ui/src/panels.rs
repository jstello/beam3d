use crate::app::{Beam3dApp, BottomTab, LeftTab};
use crate::mesh_gen::{ColorField, DiagramRibbon};
use beam3d_core::{
    BeamFormulation, CrossSection, DistributedLoad, LoadCoordSystem, Material, NodalLoad,
    SectionProfile, SupportPreset,
};
use eframe::egui::{self, Color32, RichText, Stroke};
use egui_extras::{Column, TableBuilder};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};

pub fn draw_left_panel(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    ui.add_space(4.0);
    ui.horizontal(|ui| {
        ui.selectable_value(&mut app.left_tab, LeftTab::Structure, "🏗 Structure");
        ui.selectable_value(&mut app.left_tab, LeftTab::Sections, "📐 Sections & Mat");
        ui.selectable_value(&mut app.left_tab, LeftTab::Loads, "⬇ Loads");
    });
    ui.separator();

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match app.left_tab {
            LeftTab::Structure => draw_structure_tab(app, ui),
            LeftTab::Sections => draw_sections_tab(app, ui),
            LeftTab::Loads => draw_loads_tab(app, ui),
        });
}

fn draw_structure_tab(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    ui.label(RichText::new("Project").strong().color(Color32::from_rgb(170, 205, 250)));
    ui.horizontal(|ui| {
        ui.label("Name:");
        ui.text_edit_singleline(&mut app.model.name);
    });
    if !app.model.description.is_empty() {
        ui.label(
            RichText::new(&app.model.description)
                .small()
                .color(Color32::from_gray(165)),
        );
    }

    ui.add_space(8.0);
    egui::CollapsingHeader::new(
        RichText::new(format!("Nodes ({})", app.model.nodes.len())).strong(),
    )
    .default_open(true)
    .show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label("X:");
            ui.add(egui::DragValue::new(&mut app.new_node_pos[0]).speed(0.25).suffix(" m"));
            ui.label("Y:");
            ui.add(egui::DragValue::new(&mut app.new_node_pos[1]).speed(0.25).suffix(" m"));
            ui.label("Z:");
            ui.add(egui::DragValue::new(&mut app.new_node_pos[2]).speed(0.25).suffix(" m"));
        });
        if ui.button("+ Add Node at Coordinates").clicked() {
            app.push_undo();
            let nid = app.model.add_node(app.new_node_pos);
            app.selected_node = Some(nid);
            app.selected_element = None;
            app.mark_model_changed();
        }

        ui.add_space(4.0);
        let mut delete_node = None;
        egui::ScrollArea::vertical()
            .id_salt("nodes_list_scroll")
            .max_height(190.0)
            .show(ui, |ui| {
                for n in &app.model.nodes {
                    let is_sel = app.selected_node == Some(n.id);
                    let sup_tag = match SupportPreset::from_restraints(&n.restraints) {
                        SupportPreset::Free => "",
                        SupportPreset::Fixed => " [Fixed]",
                        SupportPreset::Pinned => " [Pin]",
                        SupportPreset::RollerZ => " [Roller]",
                        SupportPreset::Custom => " [BC]",
                    };
                    ui.horizontal(|ui| {
                        let txt = format!(
                            "N{:<2} ({:>5.2}, {:>5.2}, {:>5.2}){}",
                            n.id, n.position[0], n.position[1], n.position[2], sup_tag
                        );
                        if ui
                            .selectable_label(is_sel, RichText::new(txt).monospace().size(11.5))
                            .clicked()
                        {
                            app.selected_node = Some(n.id);
                            app.selected_element = None;
                        }
                        if ui.small_button("✕").on_hover_text("Delete Node").clicked() {
                            delete_node = Some(n.id);
                        }
                    });
                }
            });

        if let Some(nid) = delete_node {
            app.push_undo();
            app.model.remove_node(nid);
            if app.selected_node == Some(nid) {
                app.selected_node = None;
            }
            app.mark_model_changed();
        }
    });

    ui.add_space(6.0);
    egui::CollapsingHeader::new(
        RichText::new(format!("Beam Elements ({})", app.model.elements.len())).strong(),
    )
    .default_open(true)
    .show(ui, |ui| {
        if app.model.nodes.len() >= 2 {
            if app.model.get_node(app.new_beam_ni).is_none() {
                app.new_beam_ni = app.model.nodes[0].id;
            }
            if app.model.get_node(app.new_beam_nj).is_none() {
                app.new_beam_nj = app.model.nodes[1.min(app.model.nodes.len() - 1)].id;
            }
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt("new_beam_ni")
                    .selected_text(format!("i: N{}", app.new_beam_ni))
                    .width(72.0)
                    .show_ui(ui, |ui| {
                        for n in &app.model.nodes {
                            ui.selectable_value(&mut app.new_beam_ni, n.id, format!("N{}", n.id));
                        }
                    });
                egui::ComboBox::from_id_salt("new_beam_nj")
                    .selected_text(format!("j: N{}", app.new_beam_nj))
                    .width(72.0)
                    .show_ui(ui, |ui| {
                        for n in &app.model.nodes {
                            ui.selectable_value(&mut app.new_beam_nj, n.id, format!("N{}", n.id));
                        }
                    });
                if ui.button("+ Connect").clicked() && app.new_beam_ni != app.new_beam_nj {
                    let sec_id = app.model.sections.first().map(|s| s.id).unwrap_or(1);
                    let mat_id = app.model.materials.first().map(|m| m.id).unwrap_or(1);
                    app.push_undo();
                    if let Some(eid) =
                        app.model
                            .add_beam(app.new_beam_ni, app.new_beam_nj, mat_id, sec_id)
                    {
                        app.selected_element = Some(eid);
                        app.selected_node = None;
                        app.mark_model_changed();
                    }
                }
            });
        }

        ui.add_space(4.0);
        let mut delete_elem = None;
        egui::ScrollArea::vertical()
            .id_salt("beams_list_scroll")
            .max_height(240.0)
            .show(ui, |ui| {
                for el in &app.model.elements {
                    let is_sel = app.selected_element == Some(el.id);
                    let sec_name = app
                        .model
                        .get_section(el.section_id)
                        .map(|s| s.name.as_str())
                        .unwrap_or("?");
                    let hinge_tag = if el.has_releases_i() || el.has_releases_j() {
                        " [H]"
                    } else {
                        ""
                    };
                    ui.horizontal(|ui| {
                        let txt = format!(
                            "B{:<2} N{}→N{} ({}){}",
                            el.id, el.node_i, el.node_j, sec_name, hinge_tag
                        );
                        if ui
                            .selectable_label(is_sel, RichText::new(txt).monospace().size(11.5))
                            .clicked()
                        {
                            app.selected_element = Some(el.id);
                            app.selected_node = None;
                        }
                        if ui.small_button("✕").on_hover_text("Delete Beam").clicked() {
                            delete_elem = Some(el.id);
                        }
                    });
                }
            });

        if let Some(eid) = delete_elem {
            app.push_undo();
            app.model.remove_element(eid);
            if app.selected_element == Some(eid) {
                app.selected_element = None;
            }
            app.mark_model_changed();
        }
    });
}

fn draw_sections_tab(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.label(
            RichText::new("Cross-Sections")
                .strong()
                .color(Color32::from_rgb(170, 205, 250)),
        );
        if ui.small_button("+ New").clicked() {
            app.push_undo();
            let id = app.model.next_section_id();
            app.model.sections.push(CrossSection {
                id,
                name: format!("Section {id}"),
                profile: SectionProfile::IBeam {
                    depth: 0.300,
                    width: 0.150,
                    web_thickness: 0.008,
                    flange_thickness: 0.012,
                },
            });
            app.selected_section_idx = app.model.sections.len() - 1;
        }
    });

    if !app.model.sections.is_empty() {
        app.selected_section_idx = app.selected_section_idx.min(app.model.sections.len() - 1);
        egui::ComboBox::from_id_salt("section_selector")
            .selected_text(&app.model.sections[app.selected_section_idx].name)
            .width(230.0)
            .show_ui(ui, |ui| {
                for (i, s) in app.model.sections.iter().enumerate() {
                    ui.selectable_value(&mut app.selected_section_idx, i, &s.name);
                }
            });

        let mut changed = false;
        let sec_before = app.model.sections[app.selected_section_idx].clone();
        {
            let sec = &mut app.model.sections[app.selected_section_idx];
            ui.horizontal(|ui| {
                ui.label("Name:");
                if ui.text_edit_singleline(&mut sec.name).changed() {
                    changed = true;
                }
            });

            // Profile family selector
            let cur_kind = match sec.profile {
                SectionProfile::IBeam { .. } => 0,
                SectionProfile::HollowBox { .. } => 1,
                SectionProfile::Pipe { .. } => 2,
                SectionProfile::SolidRect { .. } => 3,
                SectionProfile::SolidCircle { .. } => 4,
                SectionProfile::Custom { .. } => 5,
            };
            let mut next_kind = cur_kind;
            egui::ComboBox::from_label("Shape")
                .selected_text(match cur_kind {
                    0 => "I / H Wide-Flange",
                    1 => "Hollow Rectangular (HSS)",
                    2 => "Circular Pipe / Tube",
                    3 => "Solid Rectangular",
                    4 => "Solid Circular Rod",
                    _ => "Custom Properties",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut next_kind, 0, "I / H Wide-Flange");
                    ui.selectable_value(&mut next_kind, 1, "Hollow Rectangular (HSS)");
                    ui.selectable_value(&mut next_kind, 2, "Circular Pipe / Tube");
                    ui.selectable_value(&mut next_kind, 3, "Solid Rectangular");
                    ui.selectable_value(&mut next_kind, 4, "Solid Circular Rod");
                    ui.selectable_value(&mut next_kind, 5, "Custom Properties");
                });

            if next_kind != cur_kind {
                changed = true;
                let p = sec.profile.properties();
                sec.profile = match next_kind {
                    0 => SectionProfile::IBeam {
                        depth: 0.300,
                        width: 0.160,
                        web_thickness: 0.008,
                        flange_thickness: 0.013,
                    },
                    1 => SectionProfile::HollowBox {
                        depth: 0.200,
                        width: 0.150,
                        thickness: 0.010,
                    },
                    2 => SectionProfile::Pipe {
                        outer_diameter: 0.219,
                        thickness: 0.010,
                    },
                    3 => SectionProfile::SolidRect {
                        depth: 0.250,
                        width: 0.150,
                    },
                    4 => SectionProfile::SolidCircle { diameter: 0.120 },
                    _ => SectionProfile::Custom {
                        area: p.area,
                        iy: p.iy,
                        iz: p.iz,
                        j: p.j,
                        asy: p.asy,
                        asz: p.asz,
                        depth: p.half_depth_z * 2.0,
                        width: p.half_width_y * 2.0,
                    },
                };
            }

            // Dimension editors (in mm for intuitive engineering UX)
            match &mut sec.profile {
                SectionProfile::IBeam {
                    depth,
                    width,
                    web_thickness,
                    flange_thickness,
                } => {
                    changed |= mm_drag(ui, "Depth h:", depth, 20.0..=2500.0);
                    changed |= mm_drag(ui, "Flange Width b:", width, 20.0..=1500.0);
                    changed |= mm_drag(ui, "Web tw:", web_thickness, 2.0..=150.0);
                    changed |= mm_drag(ui, "Flange tf:", flange_thickness, 2.0..=180.0);
                }
                SectionProfile::HollowBox {
                    depth,
                    width,
                    thickness,
                } => {
                    changed |= mm_drag(ui, "Depth h:", depth, 20.0..=2000.0);
                    changed |= mm_drag(ui, "Width b:", width, 20.0..=2000.0);
                    changed |= mm_drag(ui, "Wall t:", thickness, 1.5..=150.0);
                }
                SectionProfile::Pipe {
                    outer_diameter,
                    thickness,
                } => {
                    changed |= mm_drag(ui, "Outer Ø D:", outer_diameter, 15.0..=2500.0);
                    changed |= mm_drag(ui, "Wall t:", thickness, 1.5..=200.0);
                }
                SectionProfile::SolidRect { depth, width } => {
                    changed |= mm_drag(ui, "Depth h:", depth, 5.0..=2000.0);
                    changed |= mm_drag(ui, "Width b:", width, 5.0..=2000.0);
                }
                SectionProfile::SolidCircle { diameter } => {
                    changed |= mm_drag(ui, "Diameter Ø:", diameter, 5.0..=2000.0);
                }
                SectionProfile::Custom {
                    area,
                    iy,
                    iz,
                    j,
                    asy,
                    asz,
                    ..
                } => {
                    ui.horizontal(|ui| {
                        ui.label("A (m²):");
                        changed |= ui
                            .add(egui::DragValue::new(area).speed(1e-4).range(1e-6..=10.0))
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Iy (m⁴):");
                        changed |= ui
                            .add(egui::DragValue::new(iy).speed(1e-5).range(1e-10..=10.0))
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Iz (m⁴):");
                        changed |= ui
                            .add(egui::DragValue::new(iz).speed(1e-5).range(1e-10..=10.0))
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("J (m⁴):");
                        changed |= ui
                            .add(egui::DragValue::new(j).speed(1e-5).range(1e-10..=10.0))
                            .changed();
                    });
                    let _ = (asy, asz);
                }
            }

            // 2D Cross-Section Blueprint Preview + Computed Properties
            draw_section_2d_preview(ui, &sec.profile);
            let props = sec.properties();
            egui::Grid::new("sec_props_grid")
                .num_columns(2)
                .spacing([12.0, 2.0])
                .show(ui, |ui| {
                    ui.label(RichText::new(format!("A  = {:.2} cm²", props.area * 1e4)).monospace().small());
                    ui.label(RichText::new(format!("J   = {:.1} cm⁴", props.j * 1e8)).monospace().small());
                    ui.end_row();
                    ui.label(RichText::new(format!("Iy = {:.1} cm⁴", props.iy * 1e8)).monospace().small());
                    ui.label(RichText::new(format!("Asz = {:.2} cm²", props.asz * 1e4)).monospace().small());
                    ui.end_row();
                    ui.label(RichText::new(format!("Iz = {:.1} cm⁴", props.iz * 1e8)).monospace().small());
                    ui.label(RichText::new(format!("Asy = {:.2} cm²", props.asy * 1e4)).monospace().small());
                    ui.end_row();
                });
        }

        if changed && app.model.sections[app.selected_section_idx] != sec_before {
            app.mark_model_changed();
        }
    }

    ui.separator();
    ui.horizontal(|ui| {
        ui.label(
            RichText::new("Materials")
                .strong()
                .color(Color32::from_rgb(170, 205, 250)),
        );
        if ui.small_button("+ Steel").clicked() {
            let id = app.model.next_material_id();
            app.model.materials.push(Material::structural_steel_s355(id));
            app.selected_material_idx = app.model.materials.len() - 1;
        }
        if ui.small_button("+ Alu").clicked() {
            let id = app.model.next_material_id();
            app.model.materials.push(Material::aluminum_6061_t6(id));
            app.selected_material_idx = app.model.materials.len() - 1;
        }
        if ui.small_button("+ Conc").clicked() {
            let id = app.model.next_material_id();
            app.model.materials.push(Material::concrete_c30(id));
            app.selected_material_idx = app.model.materials.len() - 1;
        }
    });

    if !app.model.materials.is_empty() {
        app.selected_material_idx = app.selected_material_idx.min(app.model.materials.len() - 1);
        egui::ComboBox::from_id_salt("material_selector")
            .selected_text(&app.model.materials[app.selected_material_idx].name)
            .width(230.0)
            .show_ui(ui, |ui| {
                for (i, m) in app.model.materials.iter().enumerate() {
                    ui.selectable_value(&mut app.selected_material_idx, i, &m.name);
                }
            });

        let mut mat_changed = false;
        {
            let mat = &mut app.model.materials[app.selected_material_idx];
            ui.horizontal(|ui| {
                ui.label("Name:");
                mat_changed |= ui.text_edit_singleline(&mut mat.name).changed();
            });
            let mut e_gpa = mat.e * 1e-9;
            let mut g_gpa = mat.g * 1e-9;
            let mut fy_mpa = mat.yield_strength * 1e-6;

            ui.horizontal(|ui| {
                ui.label("E (Young):");
                if ui
                    .add(egui::DragValue::new(&mut e_gpa).speed(1.0).range(0.1..=1000.0).suffix(" GPa"))
                    .changed()
                {
                    mat.e = e_gpa * 1e9;
                    mat_changed = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("G (Shear):");
                if ui
                    .add(egui::DragValue::new(&mut g_gpa).speed(0.5).range(0.05..=500.0).suffix(" GPa"))
                    .changed()
                {
                    mat.g = g_gpa * 1e9;
                    mat_changed = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Density ρ:");
                mat_changed |= ui
                    .add(
                        egui::DragValue::new(&mut mat.density)
                            .speed(10.0)
                            .range(10.0..=25_000.0)
                            .suffix(" kg/m³"),
                    )
                    .changed();
            });
            ui.horizontal(|ui| {
                ui.label("Yield fy:");
                if ui
                    .add(egui::DragValue::new(&mut fy_mpa).speed(2.0).range(1.0..=3000.0).suffix(" MPa"))
                    .changed()
                {
                    mat.yield_strength = fy_mpa * 1e6;
                    mat_changed = true;
                }
            });
        }
        if mat_changed {
            app.mark_model_changed();
        }
    }
}

fn draw_section_2d_preview(ui: &mut egui::Ui, profile: &SectionProfile) {
    let desired = egui::vec2(ui.available_width().min(250.0), 115.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 6.0, Color32::from_rgb(18, 23, 31));
    painter.rect_stroke(
        rect,
        6.0,
        Stroke::new(1.0, Color32::from_rgb(48, 58, 74)),
        egui::StrokeKind::Inside,
    );

    let props = profile.properties();
    let max_dim = (props.half_depth_z.max(props.half_width_y) * 2.0).max(1e-3) as f32;
    let scale = (rect.height().min(rect.width()) * 0.72) / max_dim;
    let center = rect.center();

    // Centroid axes (y horizontal right, z vertical up)
    painter.line_segment(
        [
            egui::pos2(rect.left() + 8.0, center.y),
            egui::pos2(rect.right() - 8.0, center.y),
        ],
        Stroke::new(1.0, Color32::from_rgba_unmultiplied(80, 190, 110, 95)),
    );
    painter.line_segment(
        [
            egui::pos2(center.x, rect.bottom() - 8.0),
            egui::pos2(center.x, rect.top() + 8.0),
        ],
        Stroke::new(1.0, Color32::from_rgba_unmultiplied(90, 145, 250, 95)),
    );

    let (edges, caps) = profile.extrusion_geometry();
    let to_pt = |yz: [f32; 2]| egui::pos2(center.x + yz[0] * scale, center.y - yz[1] * scale);

    for tri in &caps {
        painter.add(egui::Shape::convex_polygon(
            vec![to_pt(tri[0]), to_pt(tri[1]), to_pt(tri[2])],
            Color32::from_rgba_unmultiplied(92, 152, 218, 175),
            Stroke::NONE,
        ));
    }
    for e in &edges {
        painter.line_segment(
            [to_pt(e.p0), to_pt(e.p1)],
            Stroke::new(1.3, Color32::from_rgb(185, 220, 255)),
        );
    }
}

fn draw_loads_tab(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    let mut changed = false;

    ui.label(
        RichText::new("Self-Weight / Gravity")
            .strong()
            .color(Color32::from_rgb(170, 205, 250)),
    );
    changed |= ui
        .checkbox(&mut app.model.gravity.enabled, "Include Member Self-Weight")
        .changed();
    if app.model.gravity.enabled {
        ui.horizontal(|ui| {
            ui.label("gz:");
            changed |= ui
                .add(
                    egui::DragValue::new(&mut app.model.gravity.acceleration[2])
                        .speed(0.1)
                        .suffix(" m/s²"),
                )
                .changed();
        });
    }

    ui.separator();
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(format!("Nodal Loads ({})", app.model.nodal_loads.len()))
                .strong()
                .color(Color32::from_rgb(250, 175, 130)),
        );
        if ui.small_button("+ Add").clicked() && !app.model.nodes.is_empty() {
            app.push_undo();
            let nid = app
                .selected_node
                .unwrap_or_else(|| app.model.nodes.last().unwrap().id);
            let id = app.model.next_nodal_load_id();
            app.model.nodal_loads.push(NodalLoad {
                id,
                node_id: nid,
                forces: [0.0, 0.0, -25_000.0],
                moments: [0.0; 3],
            });
            changed = true;
        }
    });

    let mut del_nl = None;
    for nl in &mut app.model.nodal_loads {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("Load #{}", nl.id)).strong().small());
                egui::ComboBox::from_id_salt(("nl_node", nl.id))
                    .selected_text(format!("Node N{}", nl.node_id))
                    .width(85.0)
                    .show_ui(ui, |ui| {
                        for n in &app.model.nodes {
                            changed |= ui
                                .selectable_value(&mut nl.node_id, n.id, format!("N{}", n.id))
                                .changed();
                        }
                    });
                if ui.small_button("✕").clicked() {
                    del_nl = Some(nl.id);
                }
            });
            let mut fx = nl.forces[0] * 1e-3;
            let mut fy = nl.forces[1] * 1e-3;
            let mut fz = nl.forces[2] * 1e-3;
            ui.horizontal(|ui| {
                ui.label("F (kN):");
                if ui.add(egui::DragValue::new(&mut fx).speed(1.0).prefix("x:")).changed() {
                    nl.forces[0] = fx * 1e3;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut fy).speed(1.0).prefix("y:")).changed() {
                    nl.forces[1] = fy * 1e3;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut fz).speed(1.0).prefix("z:")).changed() {
                    nl.forces[2] = fz * 1e3;
                    changed = true;
                }
            });
            let mut mx = nl.moments[0] * 1e-3;
            let mut my = nl.moments[1] * 1e-3;
            let mut mz = nl.moments[2] * 1e-3;
            ui.horizontal(|ui| {
                ui.label("M (kN·m):");
                if ui.add(egui::DragValue::new(&mut mx).speed(0.5).prefix("x:")).changed() {
                    nl.moments[0] = mx * 1e3;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut my).speed(0.5).prefix("y:")).changed() {
                    nl.moments[1] = my * 1e3;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut mz).speed(0.5).prefix("z:")).changed() {
                    nl.moments[2] = mz * 1e3;
                    changed = true;
                }
            });
        });
    }
    if let Some(id) = del_nl {
        app.push_undo();
        app.model.nodal_loads.retain(|l| l.id != id);
        changed = true;
    }

    ui.separator();
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(format!(
                "Distributed Loads ({})",
                app.model.distributed_loads.len()
            ))
            .strong()
            .color(Color32::from_rgb(250, 200, 120)),
        );
        if ui.small_button("+ Add").clicked() && !app.model.elements.is_empty() {
            app.push_undo();
            let eid = app
                .selected_element
                .unwrap_or_else(|| app.model.elements[0].id);
            let id = app.model.next_distributed_load_id();
            app.model.distributed_loads.push(DistributedLoad {
                id,
                element_id: eid,
                coord_system: LoadCoordSystem::Global,
                q_start: [0.0, 0.0, -15_000.0],
                q_end: [0.0, 0.0, -15_000.0],
            });
            changed = true;
        }
    });

    let elem_ids: Vec<usize> = app.model.elements.iter().map(|e| e.id).collect();
    let mut del_dl = None;
    egui::ScrollArea::vertical()
        .id_salt("dloads_scroll")
        .max_height(280.0)
        .show(ui, |ui| {
            for dl in &mut app.model.distributed_loads {
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        egui::ComboBox::from_id_salt(("dl_elem", dl.id))
                            .selected_text(format!("Beam B{}", dl.element_id))
                            .width(82.0)
                            .show_ui(ui, |ui| {
                                for &eid in &elem_ids {
                                    changed |= ui
                                        .selectable_value(&mut dl.element_id, eid, format!("B{eid}"))
                                        .changed();
                                }
                            });
                        egui::ComboBox::from_id_salt(("dl_cs", dl.id))
                            .selected_text(match dl.coord_system {
                                LoadCoordSystem::Global => "Global",
                                LoadCoordSystem::Local => "Local",
                            })
                            .width(68.0)
                            .show_ui(ui, |ui| {
                                changed |= ui
                                    .selectable_value(
                                        &mut dl.coord_system,
                                        LoadCoordSystem::Global,
                                        "Global",
                                    )
                                    .changed();
                                changed |= ui
                                    .selectable_value(
                                        &mut dl.coord_system,
                                        LoadCoordSystem::Local,
                                        "Local",
                                    )
                                    .changed();
                            });
                        if ui.small_button("=Uni").on_hover_text("Set q_end = q_start").clicked() {
                            dl.q_end = dl.q_start;
                            changed = true;
                        }
                        if ui.small_button("✕").clicked() {
                            del_dl = Some(dl.id);
                        }
                    });

                    let mut qs = [
                        dl.q_start[0] * 1e-3,
                        dl.q_start[1] * 1e-3,
                        dl.q_start[2] * 1e-3,
                    ];
                    ui.horizontal(|ui| {
                        ui.label("q₁(kN/m):");
                        for (idx, pfx) in [(0, "x:"), (1, "y:"), (2, "z:")] {
                            if ui
                                .add(egui::DragValue::new(&mut qs[idx]).speed(0.5).prefix(pfx))
                                .changed()
                            {
                                dl.q_start[idx] = qs[idx] * 1e3;
                                changed = true;
                            }
                        }
                    });
                    let mut qe = [dl.q_end[0] * 1e-3, dl.q_end[1] * 1e-3, dl.q_end[2] * 1e-3];
                    ui.horizontal(|ui| {
                        ui.label("q₂(kN/m):");
                        for (idx, pfx) in [(0, "x:"), (1, "y:"), (2, "z:")] {
                            if ui
                                .add(egui::DragValue::new(&mut qe[idx]).speed(0.5).prefix(pfx))
                                .changed()
                            {
                                dl.q_end[idx] = qe[idx] * 1e3;
                                changed = true;
                            }
                        }
                    });
                });
            }
        });

    if let Some(id) = del_dl {
        app.push_undo();
        app.model.distributed_loads.retain(|l| l.id != id);
        changed = true;
    }

    if changed {
        app.mark_model_changed();
    }
}

pub fn draw_right_panel(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            draw_summary_card(app, ui);
            ui.add_space(6.0);
            draw_display_settings_card(app, ui);
            ui.add_space(6.0);
            draw_selection_inspector(app, ui);
        });
}

fn draw_summary_card(app: &Beam3dApp, ui: &mut egui::Ui) {
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("Analysis Summary")
                    .strong()
                    .color(Color32::from_rgb(170, 205, 250)),
            );
            if let Some(sol) = &app.solution {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(format!("✓ {:.2} ms", sol.summary.solve_time_ms))
                            .small()
                            .color(Color32::from_rgb(95, 225, 135)),
                    );
                });
            }
        });

        if let Some(err) = &app.solver_error {
            ui.colored_label(Color32::from_rgb(255, 110, 100), format!("⚠ {err}"));
            return;
        }

        let Some(sol) = &app.solution else {
            ui.label(RichText::new("Press Solve to run 3D FEA.").italics());
            return;
        };

        let s = &sol.summary;
        egui::Grid::new("summary_metrics_grid")
            .num_columns(2)
            .spacing([14.0, 3.0])
            .show(ui, |ui| {
                ui.label("Active / Total DOFs:");
                ui.label(RichText::new(format!("{} / {}", s.free_dofs, s.total_dofs)).monospace());
                ui.end_row();

                ui.label("Peak Deflection |u|:");
                ui.label(
                    RichText::new(format!("{:.3} mm", s.max_displacement_m * 1e3))
                        .monospace()
                        .strong(),
                );
                ui.end_row();

                ui.label("Peak Bending |M|:");
                ui.label(RichText::new(format!("{:.2} kN·m", s.max_moment_nm * 1e-3)).monospace());
                ui.end_row();

                ui.label("Peak Shear |V|:");
                ui.label(RichText::new(format!("{:.2} kN", s.max_shear_n * 1e-3)).monospace());
                ui.end_row();

                ui.label("Axial Range N:");
                ui.label(
                    RichText::new(format!(
                        "{:.1} .. +{:.1} kN",
                        s.min_axial_n * 1e-3,
                        s.max_axial_n * 1e-3
                    ))
                    .monospace(),
                );
                ui.end_row();

                ui.label("Peak Normal Stress:");
                ui.label(RichText::new(format!("{:.1} MPa", s.max_stress_pa * 1e-6)).monospace());
                ui.end_row();

                ui.label("Equilibrium Residual:");
                ui.label(
                    RichText::new(format!("{:.2e} N", s.force_equilibrium_residual_n))
                        .monospace()
                        .small()
                        .color(Color32::from_gray(165)),
                );
                ui.end_row();
            });

        let util = s.max_utilization as f32;
        let util_col = if util <= 0.75 {
            Color32::from_rgb(55, 195, 110)
        } else if util <= 1.0 {
            Color32::from_rgb(240, 180, 50)
        } else {
            Color32::from_rgb(235, 75, 75)
        };
        ui.add_space(3.0);
        ui.add(
            egui::ProgressBar::new(util.clamp(0.0, 1.0))
                .fill(util_col)
                .text(format!(
                    "Max Yield Utilization: {:.1}%{}",
                    util * 100.0,
                    s.max_utilization_element
                        .map(|id| format!(" (B{id})"))
                        .unwrap_or_default()
                )),
        );
    });
}

fn draw_display_settings_card(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new(RichText::new("🎨 3D Post-Processor & View").strong())
        .default_open(true)
        .show(ui, |ui| {
            egui::ComboBox::from_label("Contour")
                .selected_text(app.color_field.label())
                .width(175.0)
                .show_ui(ui, |ui| {
                    for cf in ColorField::ALL {
                        ui.selectable_value(&mut app.color_field, cf, cf.label());
                    }
                });

            egui::ComboBox::from_label("3D Diagram")
                .selected_text(app.diagram_ribbon.label())
                .width(175.0)
                .show_ui(ui, |ui| {
                    for dr in DiagramRibbon::ALL {
                        ui.selectable_value(&mut app.diagram_ribbon, dr, dr.label());
                    }
                });

            if app.diagram_ribbon != DiagramRibbon::None {
                ui.add(
                    egui::Slider::new(&mut app.diagram_scale, 0.2..=3.5)
                        .text("Diagram Height"),
                );
            }

            ui.horizontal(|ui| {
                ui.checkbox(&mut app.show_deformed, "Deformed Shape");
                ui.checkbox(&mut app.animate_deformed, "Animate");
                if ui.small_button("Auto Scale").clicked() {
                    app.recompute_auto_disp_scale();
                }
            });

            if app.show_deformed {
                ui.add(
                    egui::Slider::new(&mut app.disp_scale, 0.1..=2000.0)
                        .logarithmic(true)
                        .text("Deflection ×"),
                );
                ui.checkbox(&mut app.show_undeformed_ghost, "Ghost Undeformed Wireframe");
            }

            ui.horizontal(|ui| {
                ui.checkbox(&mut app.show_extruded, "3D Solid Profiles");
                if app.show_extruded {
                    ui.add(
                        egui::DragValue::new(&mut app.profile_scale)
                            .speed(0.05)
                            .range(0.2..=4.0)
                            .prefix("×"),
                    );
                }
            });

            ui.horizontal_wrapped(|ui| {
                ui.checkbox(&mut app.show_supports, "Supports");
                ui.checkbox(&mut app.show_loads, "Loads");
                ui.checkbox(&mut app.show_reactions, "Reactions");
                ui.checkbox(&mut app.show_local_axes, "Local Axes");
                ui.checkbox(&mut app.show_node_labels, "Labels");
                ui.checkbox(&mut app.show_grid, "Grid");
            });
        });
}

fn draw_selection_inspector(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    ui.label(
        RichText::new("Selection Inspector")
            .strong()
            .color(Color32::from_rgb(250, 210, 120)),
    );

    if let Some(nid) = app.selected_node {
        draw_node_inspector(app, ui, nid);
    } else if let Some(eid) = app.selected_element {
        draw_element_inspector(app, ui, eid);
    } else {
        ui.label(
            RichText::new("Click any node or beam in the 3D viewport (or left tree) to inspect and edit its properties, hinges, cross-section, and internal forces.")
                .small()
                .color(Color32::from_gray(155)),
        );
    }
}

fn draw_node_inspector(app: &mut Beam3dApp, ui: &mut egui::Ui, nid: usize) {
    let Some(node_clone) = app.model.get_node(nid).cloned() else {
        app.selected_node = None;
        return;
    };

    let mut changed = false;
    let mut extrude_from_node: Option<([f64; 3], f64)> = None;
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("Node N{nid}")).strong());
            if ui.small_button("+ Nodal Load").clicked() {
                app.push_undo();
                let lid = app.model.next_nodal_load_id();
                app.model.nodal_loads.push(NodalLoad {
                    id: lid,
                    node_id: nid,
                    forces: [0.0, 0.0, -20_000.0],
                    moments: [0.0; 3],
                });
                app.left_tab = LeftTab::Loads;
                changed = true;
            }
        });

        ui.horizontal(|ui| {
            ui.label("ΔZ:");
            ui.add(
                egui::DragValue::new(&mut app.extrude_dz)
                    .speed(0.25)
                    .range(-50.0..=50.0)
                    .suffix(" m"),
            );
            if ui
                .small_button("⬆ Extrude Column (+ΔZ)")
                .on_hover_text("Create a node offset by ΔZ and connect a vertical beam")
                .clicked()
                && app.extrude_dz.abs() > 1e-3
            {
                extrude_from_node = Some((node_clone.position, app.extrude_dz));
            }
        });

        if let Some(node) = app.model.get_node_mut(nid) {
            ui.horizontal(|ui| {
                ui.label("Coords (m):");
                for (i, lbl) in [(0, "X:"), (1, "Y:"), (2, "Z:")] {
                    changed |= ui
                        .add(egui::DragValue::new(&mut node.position[i]).speed(0.1).prefix(lbl))
                        .changed();
                }
            });

            let cur_preset = SupportPreset::from_restraints(&node.restraints);
            let mut sel_preset = cur_preset;
            egui::ComboBox::from_label("Support")
                .selected_text(cur_preset.label())
                .show_ui(ui, |ui| {
                    for p in [
                        SupportPreset::Free,
                        SupportPreset::Fixed,
                        SupportPreset::Pinned,
                        SupportPreset::RollerZ,
                        SupportPreset::Custom,
                    ] {
                        ui.selectable_value(&mut sel_preset, p, p.label());
                    }
                });
            if sel_preset != cur_preset {
                if let Some(r) = sel_preset.to_restraints() {
                    node.restraints = r;
                    changed = true;
                }
            }

            ui.horizontal_wrapped(|ui| {
                for (i, name) in ["ux", "uy", "uz", "rx", "ry", "rz"].iter().enumerate() {
                    changed |= ui.checkbox(&mut node.restraints[i], *name).changed();
                }
            });

            if node.is_supported() {
                egui::CollapsingHeader::new("Prescribed Settlement / Rotation")
                    .default_open(false)
                    .show(ui, |ui| {
                        for (i, name) in ["ux (m)", "uy (m)", "uz (m)", "rx (rad)", "ry (rad)", "rz (rad)"]
                            .iter()
                            .enumerate()
                        {
                            if node.restraints[i] {
                                ui.horizontal(|ui| {
                                    ui.label(*name);
                                    changed |= ui
                                        .add(egui::DragValue::new(&mut node.prescribed[i]).speed(0.001))
                                        .changed();
                                });
                            }
                        }
                    });
            }
        }

        if let Some(nr) = app.solution.as_ref().and_then(|s| s.node_results.get(&nid)) {
            ui.separator();
            ui.label(RichText::new("Solved Displacements").strong().small());
            ui.label(
                RichText::new(format!(
                    "u = ({:+.3}, {:+.3}, {:+.3}) mm\nθ = ({:+.3}, {:+.3}, {:+.3}) mrad",
                    nr.displacements[0] * 1e3,
                    nr.displacements[1] * 1e3,
                    nr.displacements[2] * 1e3,
                    nr.displacements[3] * 1e3,
                    nr.displacements[4] * 1e3,
                    nr.displacements[5] * 1e3,
                ))
                .monospace()
                .small(),
            );
            if node_clone.is_supported() {
                ui.label(RichText::new("Support Reactions").strong().small());
                ui.label(
                    RichText::new(format!(
                        "R = ({:+.2}, {:+.2}, {:+.2}) kN\nM = ({:+.2}, {:+.2}, {:+.2}) kN·m",
                        nr.reactions[0] * 1e-3,
                        nr.reactions[1] * 1e-3,
                        nr.reactions[2] * 1e-3,
                        nr.reactions[3] * 1e-3,
                        nr.reactions[4] * 1e-3,
                        nr.reactions[5] * 1e-3,
                    ))
                    .monospace()
                    .small()
                    .color(Color32::from_rgb(120, 235, 160)),
                );
            }
        }
    });

    if let Some((pos, dz)) = extrude_from_node {
        let top_pos = [pos[0], pos[1], pos[2] + dz];
        let sec_id = app.model.sections.first().map(|s| s.id).unwrap_or(1);
        let mat_id = app.model.materials.first().map(|m| m.id).unwrap_or(1);
        app.push_undo();
        let top_nid = app.model.find_or_add_node(top_pos, 1e-3);
        if let Some(eid) = app.model.add_beam(nid, top_nid, mat_id, sec_id) {
            app.selected_node = Some(top_nid);
            app.status_msg = format!(
                "Extruded Column B{eid} from N{nid} to N{top_nid} at Z = {:.2} m.",
                top_pos[2]
            );
        }
        changed = true;
    }

    if changed {
        app.mark_model_changed();
    }
}

fn draw_element_inspector(app: &mut Beam3dApp, ui: &mut egui::Ui, eid: usize) {
    if app.model.get_element(eid).is_none() {
        app.selected_element = None;
        return;
    }

    let sections: Vec<(usize, String)> = app
        .model
        .sections
        .iter()
        .map(|s| (s.id, s.name.clone()))
        .collect();
    let materials: Vec<(usize, String)> = app
        .model
        .materials
        .iter()
        .map(|m| (m.id, m.name.clone()))
        .collect();

    let mut changed = false;
    let mut apply_formulation_all: Option<BeamFormulation> = None;
    let mut trigger_subdivide = false;

    egui::Frame::group(ui.style()).show(ui, |ui| {
        if let Some((ni, nj)) = app.model.get_element(eid).map(|e| (e.node_i, e.node_j)) {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(format!("Beam B{} (N{} → N{})", eid, ni, nj))
                        .strong(),
                );
                if ui.small_button("+ Dist Load").clicked() {
                    let lid = app.model.next_distributed_load_id();
                    app.model.distributed_loads.push(DistributedLoad {
                        id: lid,
                        element_id: eid,
                        coord_system: LoadCoordSystem::Global,
                        q_start: [0.0, 0.0, -15_000.0],
                        q_end: [0.0, 0.0, -15_000.0],
                    });
                    app.left_tab = LeftTab::Loads;
                    changed = true;
                }
            });

            if let Some(el) = app.model.get_element_mut(eid) {
                egui::ComboBox::from_label("Section")
                    .selected_text(
                        sections
                            .iter()
                            .find(|(id, _)| *id == el.section_id)
                            .map(|(_, n)| n.as_str())
                            .unwrap_or("?"),
                    )
                    .show_ui(ui, |ui| {
                        for (id, name) in &sections {
                            changed |= ui.selectable_value(&mut el.section_id, *id, name).changed();
                        }
                    });

                egui::ComboBox::from_label("Material")
                    .selected_text(
                        materials
                            .iter()
                            .find(|(id, _)| *id == el.material_id)
                            .map(|(_, n)| n.as_str())
                            .unwrap_or("?"),
                    )
                    .show_ui(ui, |ui| {
                        for (id, name) in &materials {
                            changed |=
                                ui.selectable_value(&mut el.material_id, *id, name).changed();
                        }
                    });

                egui::ComboBox::from_label("Theory")
                    .selected_text(el.formulation.label())
                    .show_ui(ui, |ui| {
                        changed |= ui
                            .selectable_value(
                                &mut el.formulation,
                                BeamFormulation::Timoshenko,
                                BeamFormulation::Timoshenko.label(),
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut el.formulation,
                                BeamFormulation::EulerBernoulli,
                                BeamFormulation::EulerBernoulli.label(),
                            )
                            .changed();
                    });

                let cur_form = el.formulation;
                if ui
                    .small_button("Apply Theory to All Beams")
                    .on_hover_text("Switch every beam in the model to this formulation")
                    .clicked()
                {
                    apply_formulation_all = Some(cur_form);
                }

                changed |= ui
                    .add(
                        egui::Slider::new(&mut el.roll_angle_deg, -180.0..=180.0)
                            .text("Roll γ (°)"),
                    )
                    .changed();

                ui.add_space(3.0);
                ui.label(RichText::new("Member End Releases (Local Hinges)").small().strong());
                ui.horizontal(|ui| {
                    ui.label("Start i:");
                    changed |= ui.checkbox(&mut el.releases_i[4], "My").changed();
                    changed |= ui.checkbox(&mut el.releases_i[5], "Mz").changed();
                    changed |= ui.checkbox(&mut el.releases_i[3], "T").changed();
                    changed |= ui.checkbox(&mut el.releases_i[0], "N").changed();
                });
                ui.horizontal(|ui| {
                    ui.label("End  j:");
                    changed |= ui.checkbox(&mut el.releases_j[4], "My").changed();
                    changed |= ui.checkbox(&mut el.releases_j[5], "Mz").changed();
                    changed |= ui.checkbox(&mut el.releases_j[3], "T").changed();
                    changed |= ui.checkbox(&mut el.releases_j[0], "N").changed();
                });
            }

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Mesh Refine:");
                ui.add(egui::DragValue::new(&mut app.subdivide_count).range(2..=16).suffix(" segs"));
                if ui.button("Subdivide").clicked() {
                    trigger_subdivide = true;
                }
            });

            if let Some(er) = app.solution.as_ref().and_then(|s| s.element_results.get(&eid)) {
                ui.separator();
                let max_my = er
                    .stations
                    .iter()
                    .map(|s| s.moment_my.abs())
                    .fold(0.0_f64, f64::max);
                let max_vz = er
                    .stations
                    .iter()
                    .map(|s| s.shear_vz.abs())
                    .fold(0.0_f64, f64::max);
                let n0 = er.stations.first().map(|s| s.axial_n).unwrap_or(0.0);
                ui.label(
                    RichText::new(format!(
                        "Length L  = {:.3} m\nAxial N₀  = {:+.2} kN\nPeak |Vz| = {:.2} kN\nPeak |My| = {:.2} kN·m\nUtil σ/fy = {:.1}%",
                        er.length,
                        n0 * 1e-3,
                        max_vz * 1e-3,
                        max_my * 1e-3,
                        er.max_utilization * 100.0,
                    ))
                    .monospace()
                    .small(),
                );
            }
        }
    });

    if let Some(f) = apply_formulation_all {
        app.push_undo();
        for e in &mut app.model.elements {
            e.formulation = f;
        }
        changed = true;
    }

    if trigger_subdivide {
        app.push_undo();
        let new_ids = app.model.subdivide_element(eid, app.subdivide_count);
        app.selected_element = new_ids.first().copied();
        changed = true;
    }

    if changed {
        app.mark_model_changed();
    }
}

pub fn draw_bottom_panel(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.selectable_value(
            &mut app.bottom_tab,
            BottomTab::MemberDiagrams,
            "📈 Selected Member Diagrams",
        );
        ui.selectable_value(
            &mut app.bottom_tab,
            BottomTab::ReactionsTable,
            "⚓ Support Reactions Table",
        );
        ui.selectable_value(
            &mut app.bottom_tab,
            BottomTab::DisplacementsTable,
            "📏 Nodal Displacements Table",
        );
    });
    ui.separator();

    match app.bottom_tab {
        BottomTab::MemberDiagrams => draw_member_diagrams_tab(app, ui),
        BottomTab::ReactionsTable => draw_reactions_table(app, ui),
        BottomTab::DisplacementsTable => draw_displacements_table(app, ui),
    }
}

fn draw_member_diagrams_tab(app: &Beam3dApp, ui: &mut egui::Ui) {
    let Some(sol) = &app.solution else {
        ui.label("Solve the model to view member diagrams.");
        return;
    };
    let eid = app
        .selected_element
        .or_else(|| app.model.elements.first().map(|e| e.id));
    let Some(eid) = eid else {
        ui.label("No beam elements in model.");
        return;
    };
    let Some(er) = sol.element_results.get(&eid) else {
        return;
    };

    ui.label(
        RichText::new(format!(
            "Internal Force & Deflection Curves — Beam B{} (L = {:.3} m)  [Select any beam in 3D viewport to switch]",
            eid, er.length
        ))
        .small()
        .color(Color32::from_gray(185)),
    );

    let avail_w = ui.available_width();
    let plot_w = ((avail_w - 24.0) / 3.0).max(160.0);
    let plot_h = ui.available_height().max(120.0);

    let uz_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.disp_local[2] * 1e3])
        .collect();
    let uy_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.disp_local[1] * 1e3])
        .collect();

    let my_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.moment_my * 1e-3])
        .collect();
    let mz_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.moment_mz * 1e-3])
        .collect();

    let vz_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.shear_vz * 1e-3])
        .collect();
    let n_pts: PlotPoints = er
        .stations
        .iter()
        .map(|s| [s.x, s.axial_n * 1e-3])
        .collect();

    ui.horizontal(|ui| {
        Plot::new("plot_deflection")
            .width(plot_w)
            .height(plot_h)
            .legend(Legend::default())
            .x_axis_label("x (m)")
            .y_axis_label("mm")
            .show(ui, |pui| {
                pui.hline(HLine::new("", 0.0).color(Color32::from_gray(85)));
                pui.line(
                    Line::new("uz (mm)", uz_pts)
                        .color(Color32::from_rgb(75, 195, 255))
                        .fill(0.0),
                );
                pui.line(Line::new("uy (mm)", uy_pts).color(Color32::from_rgb(130, 235, 155)));
            });

        Plot::new("plot_moments")
            .width(plot_w)
            .height(plot_h)
            .legend(Legend::default())
            .x_axis_label("x (m)")
            .y_axis_label("kN·m")
            .show(ui, |pui| {
                pui.hline(HLine::new("", 0.0).color(Color32::from_gray(85)));
                pui.line(
                    Line::new("My (kN·m)", my_pts)
                        .color(Color32::from_rgb(250, 145, 70))
                        .fill(0.0),
                );
                pui.line(Line::new("Mz (kN·m)", mz_pts).color(Color32::from_rgb(235, 95, 195)));
            });

        Plot::new("plot_shear_axial")
            .width(plot_w)
            .height(plot_h)
            .legend(Legend::default())
            .x_axis_label("x (m)")
            .y_axis_label("kN")
            .show(ui, |pui| {
                pui.hline(HLine::new("", 0.0).color(Color32::from_gray(85)));
                pui.line(
                    Line::new("Vz (kN)", vz_pts)
                        .color(Color32::from_rgb(245, 210, 75))
                        .fill(0.0),
                );
                pui.line(Line::new("Axial N (kN)", n_pts).color(Color32::from_rgb(100, 225, 165)));
            });
    });
}

fn draw_reactions_table(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    let Some(sol) = &app.solution else {
        ui.label("Solve the model to inspect support reactions.");
        return;
    };

    TableBuilder::new(ui)
        .striped(true)
        .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
        .column(Column::initial(70.0))
        .columns(Column::initial(95.0), 6)
        .header(20.0, |mut header| {
            for h in [
                "Node",
                "Rx (kN)",
                "Ry (kN)",
                "Rz (kN)",
                "Mx (kN·m)",
                "My (kN·m)",
                "Mz (kN·m)",
            ] {
                header.col(|ui| {
                    ui.strong(h);
                });
            }
        })
        .body(|mut body| {
            let mut sum = [0.0_f64; 6];
            for n in &app.model.nodes {
                if !n.is_supported() {
                    continue;
                }
                if let Some(nr) = sol.node_results.get(&n.id) {
                    for i in 0..6 {
                        sum[i] += nr.reactions[i];
                    }
                    body.row(18.0, |mut row| {
                        row.col(|ui| {
                            if ui.selectable_label(app.selected_node == Some(n.id), format!("N{}", n.id)).clicked() {
                                app.selected_node = Some(n.id);
                                app.selected_element = None;
                            }
                        });
                        for i in 0..6 {
                            row.col(|ui| {
                                ui.monospace(format!("{:+.3}", nr.reactions[i] * 1e-3));
                            });
                        }
                    });
                }
            }
            body.row(20.0, |mut row| {
                row.col(|ui| {
                    ui.strong("Σ Total");
                });
                for i in 0..3 {
                    row.col(|ui| {
                        ui.strong(format!("{:+.3}", sum[i] * 1e-3));
                    });
                }
                for _ in 3..6 {
                    row.col(|ui| {
                        ui.label("—");
                    });
                }
            });
        });
}

fn draw_displacements_table(app: &mut Beam3dApp, ui: &mut egui::Ui) {
    let Some(sol) = &app.solution else {
        ui.label("Solve the model to inspect nodal displacements.");
        return;
    };

    TableBuilder::new(ui)
        .striped(true)
        .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
        .column(Column::initial(65.0))
        .columns(Column::initial(90.0), 7)
        .header(20.0, |mut header| {
            for h in [
                "Node",
                "ux (mm)",
                "uy (mm)",
                "uz (mm)",
                "|u| (mm)",
                "rx (mrad)",
                "ry (mrad)",
                "rz (mrad)",
            ] {
                header.col(|ui| {
                    ui.strong(h);
                });
            }
        })
        .body(|mut body| {
            for n in &app.model.nodes {
                if let Some(nr) = sol.node_results.get(&n.id) {
                    let d = nr.displacements;
                    let mag = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    body.row(18.0, |mut row| {
                        row.col(|ui| {
                            if ui
                                .selectable_label(app.selected_node == Some(n.id), format!("N{}", n.id))
                                .clicked()
                            {
                                app.selected_node = Some(n.id);
                                app.selected_element = None;
                            }
                        });
                        for val in [d[0] * 1e3, d[1] * 1e3, d[2] * 1e3, mag * 1e3, d[3] * 1e3, d[4] * 1e3, d[5] * 1e3] {
                            row.col(|ui| {
                                ui.monospace(format!("{:+.3}", val));
                            });
                        }
                    });
                }
            }
        });
}

fn mm_drag(
    ui: &mut egui::Ui,
    label: &str,
    val_m: &mut f64,
    range_mm: std::ops::RangeInclusive<f64>,
) -> bool {
    let mut mm = *val_m * 1e3;
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        if ui
            .add(egui::DragValue::new(&mut mm).speed(1.0).range(range_mm).suffix(" mm"))
            .changed()
        {
            *val_m = mm * 1e-3;
            changed = true;
        }
    });
    changed
}
