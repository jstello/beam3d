use crate::app::{Beam3dApp, EditorTool, LeftTab};
use crate::mesh_gen::{ColorField, DiagramRibbon};
use beam3d_core::{StructuralModel, TemplateKind};
use eframe::egui::{self, Color32, RichText};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TutorialTrack {
    ViewportAndResults,
    Draw3dSpaceFrame,
    SectionsRollAndHinges,
    TimoshenkoAndRefinement,
}

impl TutorialTrack {
    pub const ALL: [TutorialTrack; 4] = [
        TutorialTrack::Draw3dSpaceFrame,
        TutorialTrack::ViewportAndResults,
        TutorialTrack::SectionsRollAndHinges,
        TutorialTrack::TimoshenkoAndRefinement,
    ];

    pub fn title(self) -> &'static str {
        match self {
            TutorialTrack::Draw3dSpaceFrame => "1. How to Draw in 3D (Build a Frame)",
            TutorialTrack::ViewportAndResults => "2. 3D Navigation & Post-Processing",
            TutorialTrack::SectionsRollAndHinges => "3. Sections, Roll Angle & Hinges",
            TutorialTrack::TimoshenkoAndRefinement => "4. Timoshenko Shear & Subdivision",
        }
    }
}

#[derive(Debug, Clone)]
pub struct TutorialState {
    pub open: bool,
    pub track: TutorialTrack,
    pub step: usize,
}

impl Default for TutorialState {
    fn default() -> Self {
        Self {
            open: false,
            track: TutorialTrack::Draw3dSpaceFrame,
            step: 0,
        }
    }
}

struct StepSpec {
    title: &'static str,
    instructions: &'static str,
    tip: &'static str,
    completed: bool,
    progress_label: Option<String>,
    quick_action_label: Option<&'static str>,
}

pub fn start_tutorial(app: &mut Beam3dApp, track: TutorialTrack) {
    app.tutorial.open = true;
    app.tutorial.track = track;
    app.tutorial.step = 0;

    match track {
        TutorialTrack::Draw3dSpaceFrame => {
            let mut empty = StructuralModel::default();
            empty.name = "My First 3D Canopy Frame".to_string();
            empty.description =
                "Interactive 3D drawing tutorial: 4 columns + 4 roof girders.".to_string();
            empty.gravity.enabled = true;
            app.load_model(empty, "Started Tutorial: How to Draw in 3D.");
            app.camera.target = nalgebra::Vector3::new(2.0, 2.0, 1.5);
            app.camera.distance = 14.0;
            app.workplane_z = 0.0;
            app.snap_grid = 1.0;
            app.tool = EditorTool::AddNode;
            app.show_node_labels = true;
        }
        TutorialTrack::ViewportAndResults => {
            app.load_model(
                TemplateKind::TiedArchBridge.build(),
                "Started Tutorial: 3D Navigation & Post-Processing.",
            );
            app.tool = EditorTool::Select;
        }
        TutorialTrack::SectionsRollAndHinges => {
            app.load_model(
                TemplateKind::PitchedPortalFrame.build(),
                "Started Tutorial: Sections, Roll Angle & Member Hinges.",
            );
            app.selected_element = app.model.elements.get(1).map(|e| e.id);
        }
        TutorialTrack::TimoshenkoAndRefinement => {
            app.load_model(
                TemplateKind::CantileverBenchmark.build(),
                "Started Tutorial: Timoshenko Shear vs Euler-Bernoulli.",
            );
            app.selected_element = Some(1);
            app.show_bottom_panel = true;
        }
    }
}

pub fn draw_tutorial_window(app: &mut Beam3dApp, ctx: &egui::Context) {
    if !app.tutorial.open {
        return;
    }

    let mut open_flag = app.tutorial.open;
    egui::Window::new("🎓 Interactive Guided Tutorial")
        .open(&mut open_flag)
        .default_width(370.0)
        .resizable(false)
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-320.0, 52.0))
        .show(ctx, |ui| {
            // Track selector
            let cur_track = app.tutorial.track;
            let mut chosen_track = cur_track;
            egui::ComboBox::from_id_salt("tutorial_track_combo")
                .selected_text(cur_track.title())
                .width(340.0)
                .show_ui(ui, |ui| {
                    for t in TutorialTrack::ALL {
                        ui.selectable_value(&mut chosen_track, t, t.title());
                    }
                });
            if chosen_track != cur_track {
                start_tutorial(app, chosen_track);
            }

            let steps = evaluate_steps(app, app.tutorial.track);
            let n_steps = steps.len();
            app.tutorial.step = app.tutorial.step.min(n_steps.saturating_sub(1));
            let cur_idx = app.tutorial.step;
            let spec = &steps[cur_idx];

            ui.add_space(4.0);
            // Step progress dots
            ui.horizontal(|ui| {
                for (i, s) in steps.iter().enumerate() {
                    let badge_col = if i == cur_idx {
                        Color32::from_rgb(95, 175, 255)
                    } else if s.completed {
                        Color32::from_rgb(70, 205, 120)
                    } else {
                        Color32::from_rgb(65, 75, 92)
                    };
                    let lbl = if s.completed {
                        format!("✓ {}", i + 1)
                    } else {
                        format!("{}", i + 1)
                    };
                    if ui
                        .add(egui::Button::new(RichText::new(lbl).small().strong()).fill(badge_col))
                        .clicked()
                    {
                        app.tutorial.step = i;
                    }
                }
            });

            ui.separator();
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(format!("Step {}/{}: {}", cur_idx + 1, n_steps, spec.title))
                        .strong()
                        .size(14.0)
                        .color(Color32::from_rgb(225, 238, 255)),
                );
                if spec.completed {
                    ui.label(
                        RichText::new("✓ Done")
                            .strong()
                            .color(Color32::from_rgb(80, 225, 130)),
                    );
                }
            });

            ui.add_space(4.0);
            ui.label(spec.instructions);

            if let Some(prog) = &spec.progress_label {
                ui.add_space(4.0);
                egui::Frame::NONE
                    .fill(Color32::from_rgb(20, 27, 38))
                    .corner_radius(5.0)
                    .inner_margin(egui::Margin::symmetric(8, 5))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(prog)
                                .monospace()
                                .small()
                                .color(Color32::from_rgb(135, 205, 255)),
                        );
                    });
            }

            ui.add_space(4.0);
            ui.label(
                RichText::new(format!("💡 Tip: {}", spec.tip))
                    .small()
                    .color(Color32::from_rgb(235, 200, 125)),
            );

            if let Some(action_lbl) = spec.quick_action_label {
                ui.add_space(4.0);
                if ui.small_button(action_lbl).clicked() {
                    run_quick_helper(app, app.tutorial.track, cur_idx);
                }
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(cur_idx > 0, egui::Button::new("← Previous"))
                    .clicked()
                {
                    app.tutorial.step = cur_idx.saturating_sub(1);
                }
                if cur_idx + 1 < n_steps {
                    let next_btn = if spec.completed {
                        egui::Button::new(
                            RichText::new("Next Step →")
                                .strong()
                                .color(Color32::from_rgb(15, 30, 20)),
                        )
                        .fill(Color32::from_rgb(80, 215, 135))
                    } else {
                        egui::Button::new("Next Step →")
                    };
                    if ui.add(next_btn).clicked() {
                        app.tutorial.step += 1;
                    }
                } else if ui.button("🎉 Finish Tutorial").clicked() {
                    app.tutorial.open = false;
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("↺ Restart").clicked() {
                        let t = app.tutorial.track;
                        start_tutorial(app, t);
                    }
                });
            });
        });

    if !open_flag {
        app.tutorial.open = false;
    }
}

fn evaluate_steps(app: &Beam3dApp, track: TutorialTrack) -> Vec<StepSpec> {
    match track {
        TutorialTrack::Draw3dSpaceFrame => {
            let ground_nodes = app
                .model
                .nodes
                .iter()
                .filter(|n| n.position[2].abs() < 1e-3)
                .count();
            let upper_nodes = app
                .model
                .nodes
                .iter()
                .filter(|n| n.position[2] >= 2.0)
                .count();
            let supported_nodes = app.model.nodes.iter().filter(|n| n.is_supported()).count();
            let n_beams = app.model.elements.len();

            vec![
                StepSpec {
                    title: "Place 4 Ground Nodes on Z = 0 m",
                    instructions: "In Beam3D, 3D cursor clicks land on an adjustable horizontal Workplane (Z) with grid snapping.\n\n1. Ensure '⊕ Add Node' is active in the top bar (Workplane Z = 0 m, Snap = 1 m).\n2. Left-click 4 corners on the 3D grid — for example (0,0), (4,0), (4,4), and (0,4).",
                    tip: "Right-drag anytime to orbit the camera without placing a node! Scroll to zoom.",
                    completed: ground_nodes >= 4,
                    progress_label: Some(format!("Ground nodes placed (Z = 0 m): {ground_nodes} / 4")),
                    quick_action_label: Some("⚡ Auto-place 4 ground nodes for me"),
                },
                StepSpec {
                    title: "Raise the 3D Workplane to Z = 3.0 m",
                    instructions: "To draw upward in 3D, raise the horizontal construction grid!\n\n• Press the ']' key three times (each press steps Z up by Snap),\n• OR click the '3m' / '+Z' button in the top-right 3D Drawing HUD,\n• OR drag 'Z:' in the top toolbar to 3.0 m.",
                    tip: "Watch the 3D grid plane physically rise to elevation Z = 3.0 m! '[' lowers Z back down.",
                    completed: app.workplane_z >= 2.5,
                    progress_label: Some(format!("Current Workplane Z = {:.2} m (Target: ≥ 3.0 m)", app.workplane_z)),
                    quick_action_label: Some("⚡ Set Workplane Z = 3.0 m & switch to Draw Beam"),
                },
                StepSpec {
                    title: "Draw Vertical Columns & Roof Beams",
                    instructions: "Existing nodes magnetically snap in full 3D even when your Workplane Z is higher!\n\n1. Select '📏 Draw Beam' in the top bar.\n2. Click ground node N1 (0,0,0), then click directly above it on the Z=3m grid (0,0,3) to create a vertical column!\n3. Continue clicking around the Z=3m grid to chain roof beams, and press Esc (or Right-click) to end a chain and connect the remaining columns.",
                    tip: "You can also select any ground node and click '⬆ Extrude Column (+ΔZ)' in the Right Inspector!",
                    completed: upper_nodes >= 4 && n_beams >= 6,
                    progress_label: Some(format!("Upper nodes (Z ≥ 2m): {upper_nodes}/4  |  Beams drawn: {n_beams}/8")),
                    quick_action_label: None,
                },
                StepSpec {
                    title: "Assign Ground Supports",
                    instructions: "Without supports, a 3D frame can move freely as a rigid body.\n\n1. Switch to '🖱 Select' (or press Esc).\n2. Click each ground node (N1..N4) in the 3D viewport.\n3. In the Right Panel ('Selection Inspector'), set Support to 'Fixed (All 6 DOFs)' or 'Pinned'.",
                    tip: "Cyan/green 3D base blocks appear beneath supported nodes.",
                    completed: supported_nodes >= 3,
                    progress_label: Some(format!("Supported nodes: {supported_nodes} / 4")),
                    quick_action_label: Some("⚡ Fix all Z = 0 ground nodes"),
                },
                StepSpec {
                    title: "Add Roof Loads & Solve!",
                    instructions: "Self-weight gravity is already enabled. Let's add live load and inspect the 3D results!\n\n1. Click any roof girder in the 3D viewport.\n2. Click '+ Dist Load' in the Right Inspector (adds -15 kN/m vertical load).\n3. Click '▶ Solve FEA' (or leave Auto-Solve checked) and try toggling '3D Diagram -> Bending Moment My'!",
                    tip: "Save your custom 3D frame anytime via 'Save...' in the top toolbar.",
                    completed: app.solution.is_some() && !app.model.distributed_loads.is_empty(),
                    progress_label: Some(if app.solution.is_some() {
                        "Model solved! Try 'Animate' in the Right Panel.".to_string()
                    } else {
                        "Waiting for valid supported frame + distributed load...".to_string()
                    }),
                    quick_action_label: None,
                },
            ]
        }
        TutorialTrack::ViewportAndResults => vec![
            StepSpec {
                title: "Orbit, Pan, Zoom & Camera Presets",
                instructions: "Explore the 24 m Tied-Arch Pedestrian Bridge:\n\n• Orbit: Left-drag or Right-drag in the 3D viewport\n• Pan: Middle-drag or Shift + Drag\n• Zoom: Two-finger scroll / mouse wheel\n• Presets: Click 'Iso', 'Front X-Z', 'Side Y-Z', 'Top X-Y', or press 'F' to Fit.",
                tip: "Check the bottom-left X/Y/Z gizmo to track your 3D orientation.",
                completed: true,
                progress_label: None,
                quick_action_label: None,
            },
            StepSpec {
                title: "Switch 3D Contour Colormaps & Ribbons",
                instructions: "In the Right Panel under '🎨 3D Post-Processor & View':\n\n1. Change 'Contour' to 'Axial Force N (kN)' — notice the arch ribs turn Blue (compression) while the tie girders & hangers turn Red (tension)!\n2. Change '3D Diagram' to 'Bending Moment My' to project shaded moment curtains directly onto the 3D bridge.",
                tip: "Check 'Reactions' to display green 3D support reaction vectors at the bridge bearings.",
                completed: app.color_field == ColorField::AxialForce
                    || app.diagram_ribbon != DiagramRibbon::None,
                progress_label: Some(format!(
                    "Contour: {} | Ribbon: {}",
                    app.color_field.label(),
                    app.diagram_ribbon.label()
                )),
                quick_action_label: Some("⚡ Show Axial Force + My Ribbon"),
            },
            StepSpec {
                title: "Inspect Any Member & 2D Plots",
                instructions: "Click any arch segment, hanger rod, or deck girder directly in the 3D viewport.\n\nLook at the bottom '📈 Selected Member Diagrams' panel to read exact continuous deflection, bending moment, shear, and axial curves along the member length!",
                tip: "Switch to '⚓ Support Reactions Table' at the bottom to verify total vertical equilibrium.",
                completed: app.selected_element.is_some() && app.show_bottom_panel,
                progress_label: app
                    .selected_element
                    .map(|id| format!("Currently inspecting Beam B{id}")),
                quick_action_label: None,
            },
        ],
        TutorialTrack::SectionsRollAndHinges => vec![
            StepSpec {
                title: "Edit Parametric Cross-Sections",
                instructions: "Open the '📐 Sections & Mat' tab in the Left Panel.\n\nDrag the Depth (h) or Flange Width (b) of 'IPE 400' and watch:\n1. The 2D blueprint preview & section properties (A, Iy, Iz, J) update live,\n2. The 3D extruded I-beams resize in the viewport, and\n3. Peak deflection & utilization update automatically!",
                tip: "You can also switch profile shapes between I-Beam, Hollow Box (HSS), Pipe, Rod, and Solid Rect.",
                completed: app.left_tab == LeftTab::Sections,
                progress_label: None,
                quick_action_label: Some("⚡ Open Sections & Mat Tab"),
            },
            StepSpec {
                title: "Rotate Cross-Section Roll Angle (γ)",
                instructions: "Click a column or rafter in the 3D viewport, then drag 'Roll γ (°)' in the Right Panel from 0° to 90°.\n\nNotice the 3D I-beam physically rotates 90° about its axis so weak-axis flanges take the load instead of the deep web!",
                tip: "Enable 'Local Axes' in the Right Panel to see RGB local (x, y, z) member triads.",
                completed: app
                    .model
                    .elements
                    .iter()
                    .any(|e| e.roll_angle_deg.abs() > 5.0),
                progress_label: None,
                quick_action_label: None,
            },
            StepSpec {
                title: "Add a Crown Moment Hinge (End Release)",
                instructions: "To model a 3-hinged arch/portal frame:\n\n1. Select rafter Beam B2 (left rafter meeting apex Node N3).\n2. In the Right Panel under 'Member End Releases', check 'End j: My'.\n3. A white 3D hinge marker appears near the apex, and the bending moment at the crown drops to 0.00 kN·m!",
                tip: "Static condensation automatically adjusts both element stiffness and trapezoidal snow fixed-end forces.",
                completed: app
                    .model
                    .elements
                    .iter()
                    .any(|e| e.has_releases_i() || e.has_releases_j()),
                progress_label: None,
                quick_action_label: Some("⚡ Release My at Rafter Apex"),
            },
        ],
        TutorialTrack::TimoshenkoAndRefinement => vec![
            StepSpec {
                title: "Compare Timoshenko vs Euler-Bernoulli",
                instructions: "This short, deep HEA 400 steel cantilever carries a 100 kN tip load.\n\n1. Note 'Peak Deflection |u|' in the Analysis Summary (~2.19 mm with Timoshenko shear).\n2. In the Right Panel, switch 'Theory' from 'Timoshenko' to 'Euler-Bernoulli'.\n3. Notice deflection drops (~1.89 mm) because Euler-Bernoulli neglects web shear deformation PL/(G·Asz)!",
                tip: "For deep beams (span/depth < 10), Timoshenko shear deformation accounts for 10–25% of total deflection.",
                completed: app
                    .model
                    .elements
                    .first()
                    .is_some_and(|e| e.formulation == beam3d_core::BeamFormulation::EulerBernoulli),
                progress_label: app.solution.as_ref().map(|s| {
                    format!(
                        "Current Tip Deflection |u| = {:.4} mm",
                        s.summary.max_displacement_m * 1e3
                    )
                }),
                quick_action_label: None,
            },
            StepSpec {
                title: "Subdivide Finite Element Mesh",
                instructions: "With Beam B1 selected, look at 'Mesh Refine' in the Right Panel and click 'Subdivide' (4 segs).\n\nBeam B1 splits into 4 collinear elements with intermediate nodes — and notice the exact tip deflection remains identical because our 2-node Timoshenko shape functions are locking-free!",
                tip: "Subdividing is super handy when you want intermediate nodes to attach cross-beams, braces, or point loads.",
                completed: app.model.elements.len() >= 2,
                progress_label: Some(format!("Elements in cantilever: {}", app.model.elements.len())),
                quick_action_label: None,
            },
        ],
    }
}

fn run_quick_helper(app: &mut Beam3dApp, track: TutorialTrack, step: usize) {
    match (track, step) {
        (TutorialTrack::Draw3dSpaceFrame, 0) => {
            app.push_undo();
            for pt in [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 4.0, 0.0], [0.0, 4.0, 0.0]] {
                app.model.find_or_add_node(pt, 1e-3);
            }
            app.mark_model_changed();
        }
        (TutorialTrack::Draw3dSpaceFrame, 1) => {
            app.workplane_z = 3.0;
            app.tool = EditorTool::DrawBeam;
        }
        (TutorialTrack::Draw3dSpaceFrame, 3) => {
            app.push_undo();
            for n in &mut app.model.nodes {
                if n.position[2].abs() < 1e-3 {
                    n.restraints = [true; 6];
                }
            }
            app.mark_model_changed();
        }
        (TutorialTrack::ViewportAndResults, 1) => {
            app.color_field = ColorField::AxialForce;
            app.diagram_ribbon = DiagramRibbon::MomentMy;
        }
        (TutorialTrack::SectionsRollAndHinges, 0) => {
            app.left_tab = LeftTab::Sections;
        }
        (TutorialTrack::SectionsRollAndHinges, 2) => {
            if let Some(el) = app.model.get_element_mut(2) {
                el.releases_j[4] = true;
                app.selected_element = Some(2);
                app.mark_model_changed();
            }
        }
        _ => {}
    }
}
