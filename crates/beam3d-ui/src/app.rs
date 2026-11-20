use crate::camera::{OrbitCamera, WorkplaneKind, point_to_segment_dist_2d};
use crate::mesh_gen::{
    ColorField, DiagramRibbon, LegendRange, ViewportDisplayOptions, build_scene_geometry,
    colormap_diverging, colormap_scientific,
};
use crate::panels::{draw_bottom_panel, draw_left_panel, draw_right_panel};
use crate::renderer::{CameraUniform, SceneGeometry, ViewportRenderer};
use crate::tutorials::{TutorialState, TutorialTrack, draw_tutorial_window, start_tutorial};
use beam3d_core::{
    DynamicConfig, DynamicSolution, LoadCoordSystem, ModalConfig, ModalSolution,
    NonlinearSolution, NonlinearStaticConfig, RayleighDamping, Solution, StructuralModel,
    TemplateKind, TimeHistoryExcitation, solve_dynamic_with_config, solve_linear_static,
    solve_modal_with_config, solve_nonlinear_static_with_config,
};
use eframe::egui::{self, Color32, CornerRadius, Pos2, Rect, RichText, Stroke};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EditorTool {
    Select,
    AddNode,
    DrawBeam,
    DrawShell,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeftTab {
    Structure,
    Sections,
    Loads,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BottomTab {
    MemberDiagrams,
    ReactionsTable,
    DisplacementsTable,
    AnalysisCharts,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalysisMode {
    LinearStatic,
    NonlinearStatic,
    Modal,
    Dynamic,
}

impl AnalysisMode {
    pub const ALL: [AnalysisMode; 4] = [
        AnalysisMode::LinearStatic,
        AnalysisMode::NonlinearStatic,
        AnalysisMode::Modal,
        AnalysisMode::Dynamic,
    ];

    pub fn label(self) -> &'static str {
        match self {
            AnalysisMode::LinearStatic => "Linear Static",
            AnalysisMode::NonlinearStatic => "Nonlinear Static (Large Deform.)",
            AnalysisMode::Modal => "Modal (Natural Frequencies)",
            AnalysisMode::Dynamic => "Dynamic Time-History",
        }
    }

    pub fn short_label(self) -> &'static str {
        match self {
            AnalysisMode::LinearStatic => "Linear Static",
            AnalysisMode::NonlinearStatic => "Large Deform.",
            AnalysisMode::Modal => "Modal",
            AnalysisMode::Dynamic => "Dynamic",
        }
    }
}

pub struct Beam3dApp {
    pub model: StructuralModel,
    undo_stack: Vec<StructuralModel>,
    redo_stack: Vec<StructuralModel>,

    pub solution: Option<Solution>,
    pub solver_error: Option<String>,
    pub auto_solve: bool,
    dirty_solve: bool,

    pub analysis_mode: AnalysisMode,
    pub nl_config: NonlinearStaticConfig,
    pub nl_solution: Option<NonlinearSolution>,
    pub modal_config: ModalConfig,
    pub modal_solution: Option<ModalSolution>,
    pub selected_mode_idx: usize,
    pub dynamic_config: DynamicConfig,
    pub dynamic_solution: Option<DynamicSolution>,
    pub dynamic_frame_idx: usize,
    pub dynamic_playing: bool,
    dynamic_playback_time: f64,
    pub dynamic_speed: f64,
    pub damping_ratio_pct: f64,
    pub excitation_kind: usize,
    pub exc_duration_s: f64,
    pub exc_freq_hz: f64,
    pub exc_base_amp_g: f64,
    pub exc_base_dir: [f64; 3],

    pub selected_node: Option<usize>,
    pub selected_element: Option<usize>,
    pub selected_shell: Option<usize>,
    pub hovered_node: Option<usize>,
    pub hovered_element: Option<usize>,
    pub hovered_shell: Option<usize>,

    pub selected_section_idx: usize,
    pub selected_material_idx: usize,
    pub new_node_pos: [f64; 3],
    pub new_beam_ni: usize,
    pub new_beam_nj: usize,
    pub subdivide_count: usize,

    pub tool: EditorTool,
    pub beam_draw_start: Option<usize>,
    pub shell_draw_points: Vec<[f64; 3]>,
    pub new_shell_nx: usize,
    pub new_shell_ny: usize,
    pub new_shell_thickness_mm: f64,
    pub new_shell_apply_load: bool,
    pub new_shell_pressure_kpa: f64,
    pub bulk_sload_coord: LoadCoordSystem,
    pub bulk_sload_kpa: [f64; 3],
    pub panel_gen_bounds: [f64; 5], // [x0, x1, y0, y1, z]
    pub workplane_kind: WorkplaneKind,
    pub workplane_offset: f64,
    pub snap_grid: f64,
    pub extrude_dz: f64,
    pub tutorial: TutorialState,

    pub left_tab: LeftTab,
    pub bottom_tab: BottomTab,
    pub show_bottom_panel: bool,

    pub show_grid: bool,
    pub show_extruded: bool,
    pub profile_scale: f32,
    pub show_deformed: bool,
    pub animate_deformed: bool,
    anim_time: f64,
    pub show_undeformed_ghost: bool,
    pub disp_scale: f64,
    pub show_supports: bool,
    pub show_loads: bool,
    pub show_reactions: bool,
    pub show_local_axes: bool,
    pub show_node_labels: bool,
    pub color_field: ColorField,
    pub diagram_ribbon: DiagramRibbon,
    pub diagram_scale: f32,

    pub camera: OrbitCamera,
    renderer: Option<ViewportRenderer>,
    scene_geo: SceneGeometry,
    pub status_msg: String,
}

impl Beam3dApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        configure_engineering_theme(&cc.egui_ctx);

        let renderer = cc
            .wgpu_render_state
            .as_ref()
            .map(ViewportRenderer::new);

        let model = TemplateKind::MultiStoryBuilding.build();
        let mut camera = OrbitCamera::default();
        let (min_p, max_p) = model.bounding_box();
        camera.fit_to_bounds(min_p, max_p);

        let mut app = Self {
            model,
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            solution: None,
            solver_error: None,
            auto_solve: true,
            dirty_solve: true,
            analysis_mode: AnalysisMode::LinearStatic,
            nl_config: NonlinearStaticConfig::default(),
            nl_solution: None,
            modal_config: ModalConfig::default(),
            modal_solution: None,
            selected_mode_idx: 0,
            dynamic_config: DynamicConfig::default(),
            dynamic_solution: None,
            dynamic_frame_idx: 0,
            dynamic_playing: false,
            dynamic_playback_time: 0.0,
            dynamic_speed: 1.0,
            damping_ratio_pct: 2.0,
            excitation_kind: 0,
            exc_duration_s: 0.25,
            exc_freq_hz: 2.0,
            exc_base_amp_g: 0.3,
            exc_base_dir: [1.0, 0.0, 0.0],
            selected_node: None,
            selected_element: Some(1),
            selected_shell: None,
            hovered_node: None,
            hovered_element: None,
            hovered_shell: None,
            selected_section_idx: 0,
            selected_material_idx: 0,
            new_node_pos: [0.0, 0.0, 0.0],
            new_beam_ni: 1,
            new_beam_nj: 2,
            subdivide_count: 4,
            tool: EditorTool::Select,
            beam_draw_start: None,
            shell_draw_points: Vec::new(),
            new_shell_nx: 4,
            new_shell_ny: 4,
            new_shell_thickness_mm: 150.0,
            new_shell_apply_load: true,
            new_shell_pressure_kpa: -5.0,
            bulk_sload_coord: LoadCoordSystem::Global,
            bulk_sload_kpa: [0.0, 0.0, -5.0],
            panel_gen_bounds: [0.0, 6.0, 0.0, 5.0, 0.0],
            workplane_kind: WorkplaneKind::XY,
            workplane_offset: 0.0,
            snap_grid: 0.5,
            extrude_dz: 3.0,
            tutorial: TutorialState::default(),
            left_tab: LeftTab::Structure,
            bottom_tab: BottomTab::MemberDiagrams,
            show_bottom_panel: true,
            show_grid: true,
            show_extruded: true,
            profile_scale: 1.0,
            show_deformed: true,
            animate_deformed: false,
            anim_time: 0.0,
            show_undeformed_ghost: true,
            disp_scale: 50.0,
            show_supports: true,
            show_loads: true,
            show_reactions: false,
            show_local_axes: false,
            show_node_labels: false,
            color_field: ColorField::VonMisesStress,
            diagram_ribbon: DiagramRibbon::None,
            diagram_scale: 1.0,
            camera,
            renderer,
            scene_geo: SceneGeometry::default(),
            status_msg: "Loaded 3D Braced Multi-Story Steel Frame.".to_string(),
        };
        app.run_solver();
        app.recompute_auto_disp_scale();
        app
    }

    /// Switches the active working plane (`XY`, `XZ`, `YZ`).
    /// If an anchor node (`beam_draw_start` or `selected_node`) is active, automatically aligns
    /// the new plane's offset to pass directly through that node so drawing columns/beams from it is immediate.
    pub fn set_workplane_kind(&mut self, kind: WorkplaneKind) {
        self.workplane_kind = kind;
        if let Some(anchor_id) = self.beam_draw_start.or(self.selected_node) {
            if let Some(n) = self.model.get_node(anchor_id) {
                self.workplane_offset = n.position[kind.normal_index()];
                self.status_msg = format!(
                    "Working plane: {} aligned through Node N{} ({} = {:.2} m)",
                    kind.full_label(),
                    anchor_id,
                    kind.normal_axis_name(),
                    self.workplane_offset
                );
                return;
            }
        }
        self.workplane_offset = 0.0;
        self.status_msg = format!(
            "Working plane: {} ({} = {:.2} m)",
            kind.full_label(),
            kind.normal_axis_name(),
            self.workplane_offset
        );
    }

    pub fn push_undo(&mut self) {
        if self.undo_stack.len() >= 40 {
            self.undo_stack.remove(0);
        }
        self.undo_stack.push(self.model.clone());
        self.redo_stack.clear();
    }

    pub fn undo(&mut self) {
        if let Some(prev) = self.undo_stack.pop() {
            self.redo_stack.push(self.model.clone());
            self.model = prev;
            self.mark_model_changed();
            self.status_msg = "Undid last edit.".to_string();
        }
    }

    pub fn redo(&mut self) {
        if let Some(next) = self.redo_stack.pop() {
            self.undo_stack.push(self.model.clone());
            self.model = next;
            self.mark_model_changed();
            self.status_msg = "Redid edit.".to_string();
        }
    }

    pub fn mark_model_changed(&mut self) {
        self.dirty_solve = true;
    }

    pub fn load_model(&mut self, new_model: StructuralModel, status: impl Into<String>) {
        self.push_undo();
        self.model = new_model;
        self.selected_node = None;
        self.selected_element = self.model.elements.first().map(|e| e.id);
        self.selected_shell = if self.selected_element.is_none() {
            self.model.shells.first().map(|s| s.id)
        } else {
            None
        };
        self.selected_section_idx = 0;
        self.selected_material_idx = 0;
        self.beam_draw_start = None;
        self.shell_draw_points.clear();
        let (min_p, max_p) = self.model.bounding_box();
        self.camera.set_preset_iso();
        self.camera.fit_to_bounds(min_p, max_p);
        self.run_solver();
        self.recompute_auto_disp_scale();
        self.status_msg = status.into();
    }

    pub fn sync_dynamic_config(&mut self) {
        self.dynamic_config.damping = if self.damping_ratio_pct <= 1e-6 {
            RayleighDamping::None
        } else {
            RayleighDamping::ModalRatio {
                zeta: (self.damping_ratio_pct * 0.01).clamp(0.0, 0.95),
            }
        };
        self.dynamic_config.excitation = match self.excitation_kind {
            0 => TimeHistoryExcitation::Step,
            1 => TimeHistoryExcitation::Impulse {
                duration_s: self.exc_duration_s.max(0.005),
            },
            2 => TimeHistoryExcitation::HalfSinePulse {
                duration_s: self.exc_duration_s.max(0.005),
            },
            3 => TimeHistoryExcitation::Harmonic {
                frequency_hz: self.exc_freq_hz.max(0.05),
            },
            4 => TimeHistoryExcitation::Ramp {
                rise_time_s: self.exc_duration_s.max(0.005),
            },
            5 => TimeHistoryExcitation::FreeVibration,
            _ => TimeHistoryExcitation::BaseSine {
                direction: self.exc_base_dir,
                amplitude_g: self.exc_base_amp_g,
                frequency_hz: self.exc_freq_hz.max(0.05),
                duration_s: self.exc_duration_s.max(0.05),
            },
        };
    }

    pub fn select_mode(&mut self, idx: usize) {
        if let Some(msol) = &self.modal_solution {
            if !msol.modes.is_empty() {
                let clamped = idx.min(msol.modes.len() - 1);
                self.selected_mode_idx = clamped;
                self.solution = Some(msol.modes[clamped].solution.clone());
                self.recompute_auto_disp_scale();
            }
        }
    }

    pub fn set_dynamic_frame(&mut self, idx: usize) {
        if let Some(dsol) = &self.dynamic_solution {
            if !dsol.frames.is_empty() {
                let clamped = idx.min(dsol.frames.len() - 1);
                self.dynamic_frame_idx = clamped;
                self.dynamic_playback_time = dsol.frames[clamped].time_s;
                self.solution = Some(dsol.frames[clamped].solution.clone());
            }
        }
    }

    pub fn run_solver(&mut self) {
        self.dirty_solve = false;
        self.dynamic_playing = false;

        match self.analysis_mode {
            AnalysisMode::LinearStatic => {
                match solve_linear_static(&self.model) {
                    Ok(sol) => {
                        self.status_msg = format!(
                            "Linear Static: {} DOFs in {:.2} ms — Max |u| = {:.3} mm, Max σ = {:.1} MPa",
                            sol.summary.free_dofs,
                            sol.summary.solve_time_ms,
                            sol.summary.max_displacement_m * 1e3,
                            sol.summary.max_stress_pa * 1e-6
                        );
                        self.solution = Some(sol);
                        self.solver_error = None;
                    }
                    Err(e) => {
                        self.solution = None;
                        self.solver_error = Some(e.to_string());
                        self.status_msg = format!("Solver warning: {e}");
                    }
                }
            }
            AnalysisMode::NonlinearStatic => {
                match solve_nonlinear_static_with_config(&self.model, &self.nl_config) {
                    Ok(nl) => {
                        self.status_msg = format!(
                            "Nonlinear Static (Large Deform.): {} steps ({} iters) in {:.1} ms — Max |u| = {:.3} mm, Max σ = {:.1} MPa",
                            nl.load_steps.len(),
                            nl.total_iterations,
                            nl.solution.summary.solve_time_ms,
                            nl.solution.summary.max_displacement_m * 1e3,
                            nl.solution.summary.max_stress_pa * 1e-6
                        );
                        let max_u = nl.solution.summary.max_displacement_m;
                        self.solution = Some(nl.solution.clone());
                        self.nl_solution = Some(nl);
                        self.solver_error = None;
                        // Automatically use true 1:1 physical scale when deformations are genuinely large
                        if max_u > 0.03 * self.model.characteristic_length() {
                            self.disp_scale = 1.0;
                        }
                    }
                    Err(e) => {
                        self.solution = None;
                        self.nl_solution = None;
                        self.solver_error = Some(e.to_string());
                        self.status_msg = format!("Nonlinear solver error: {e}");
                    }
                }
            }
            AnalysisMode::Modal => {
                match solve_modal_with_config(&self.model, &self.modal_config) {
                    Ok(msol) => {
                        let f1 = msol
                            .modes
                            .first()
                            .map(|m| m.frequency_hz)
                            .unwrap_or(0.0);
                        self.status_msg = format!(
                            "Modal: {} modes in {:.1} ms — f₁ = {:.3} Hz (T₁ = {:.3} s), Total Mass = {:.1} kg",
                            msol.modes.len(),
                            msol.solve_time_ms,
                            f1,
                            if f1 > 1e-9 { 1.0 / f1 } else { 0.0 },
                            msol.total_mass_kg
                        );
                        let idx = self
                            .selected_mode_idx
                            .min(msol.modes.len().saturating_sub(1));
                        self.selected_mode_idx = idx;
                        self.solution = msol.modes.get(idx).map(|m| m.solution.clone());
                        self.modal_solution = Some(msol);
                        self.solver_error = None;
                        self.show_deformed = true;
                        self.animate_deformed = true;
                        self.recompute_auto_disp_scale();
                    }
                    Err(e) => {
                        self.solution = None;
                        self.modal_solution = None;
                        self.solver_error = Some(e.to_string());
                        self.status_msg = format!("Modal solver error: {e}");
                    }
                }
            }
            AnalysisMode::Dynamic => {
                self.sync_dynamic_config();
                match solve_dynamic_with_config(&self.model, &self.dynamic_config) {
                    Ok(dsol) => {
                        self.status_msg = format!(
                            "{} Dynamic: {} steps in {:.1} ms — Peak |u| = {:.3} mm @ t = {:.3} s, Peak σ = {:.1} MPa",
                            if dsol.nonlinear { "Nonlinear" } else { "Linear" },
                            dsol.time_steps,
                            dsol.solve_time_ms,
                            dsol.peak_displacement_m * 1e3,
                            dsol.peak_displacement_time_s,
                            dsol.peak_stress_pa * 1e-6
                        );
                        self.dynamic_frame_idx = 0;
                        self.dynamic_playback_time = 0.0;
                        let peak_u = dsol.peak_displacement_m;
                        self.solution = dsol.frames.first().map(|f| f.solution.clone());
                        self.dynamic_solution = Some(dsol);
                        self.solver_error = None;
                        self.show_deformed = true;
                        self.animate_deformed = false;
                        self.dynamic_playing = true;
                        if peak_u > 0.03 * self.model.characteristic_length() {
                            self.disp_scale = 1.0;
                        } else if peak_u > 1e-9 {
                            let target_visual = 0.085 * self.model.characteristic_length();
                            self.disp_scale = (target_visual / peak_u).clamp(0.5, 2000.0);
                        }
                    }
                    Err(e) => {
                        self.solution = None;
                        self.dynamic_solution = None;
                        self.solver_error = Some(e.to_string());
                        self.status_msg = format!("Dynamic solver error: {e}");
                    }
                }
            }
        }
    }

    pub fn recompute_auto_disp_scale(&mut self) {
        let char_len = self.model.characteristic_length();
        match self.analysis_mode {
            AnalysisMode::NonlinearStatic => {
                if let Some(sol) = &self.solution {
                    let max_u = sol.summary.max_displacement_m;
                    if max_u > 0.03 * char_len {
                        self.disp_scale = 1.0;
                    } else if max_u > 1e-9 {
                        let target_visual = 0.075 * char_len;
                        self.disp_scale = (target_visual / max_u).clamp(0.5, 2000.0);
                    }
                }
            }
            AnalysisMode::Dynamic => {
                if let Some(dsol) = &self.dynamic_solution {
                    let peak_u = dsol.peak_displacement_m;
                    if peak_u > 0.03 * char_len {
                        self.disp_scale = 1.0;
                    } else if peak_u > 1e-9 {
                        let target_visual = 0.085 * char_len;
                        self.disp_scale = (target_visual / peak_u).clamp(0.5, 2000.0);
                    }
                }
            }
            AnalysisMode::LinearStatic | AnalysisMode::Modal => {
                if let Some(sol) = &self.solution {
                    let max_u = sol.summary.max_displacement_m;
                    if max_u > 1e-9 {
                        let target_visual = 0.075 * char_len;
                        self.disp_scale = (target_visual / max_u).clamp(0.5, 2000.0);
                    }
                }
            }
        }
    }

    fn save_json_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Beam3D JSON Model", &["json"])
            .set_file_name(format!(
                "{}.json",
                self.model.name.to_lowercase().replace(' ', "_")
            ))
            .save_file()
        {
            match self.model.to_json_pretty() {
                Ok(txt) => match std::fs::write(&path, txt) {
                    Ok(()) => self.status_msg = format!("Saved model to {}", path.display()),
                    Err(e) => self.status_msg = format!("Failed to write file: {e}"),
                },
                Err(e) => self.status_msg = format!("Serialization error: {e}"),
            }
        }
    }

    fn open_json_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Beam3D JSON Model", &["json"])
            .pick_file()
        {
            match std::fs::read_to_string(&path) {
                Ok(txt) => match StructuralModel::from_json(&txt) {
                    Ok(m) => self.load_model(m, format!("Loaded {}", path.display())),
                    Err(e) => self.status_msg = format!("Invalid JSON model: {e}"),
                },
                Err(e) => self.status_msg = format!("Failed to read file: {e}"),
            }
        }
    }
}

impl eframe::App for Beam3dApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        handle_global_shortcuts(self, &ctx);

        if self.dirty_solve && self.auto_solve {
            self.run_solver();
        }

        if self.analysis_mode == AnalysisMode::Dynamic && self.dynamic_playing {
            let dt = ctx.input(|i| i.stable_dt) as f64;
            if let Some(dsol) = &self.dynamic_solution {
                if let Some(last_frame) = dsol.frames.last() {
                    let dur = last_frame.time_s.max(1e-6);
                    self.dynamic_playback_time += dt * self.dynamic_speed;
                    if self.dynamic_playback_time > dur {
                        self.dynamic_playback_time %= dur;
                    }
                    let target_t = self.dynamic_playback_time;
                    let idx = dsol
                        .frames
                        .partition_point(|f| f.time_s < target_t)
                        .min(dsol.frames.len() - 1);
                    if idx != self.dynamic_frame_idx {
                        self.dynamic_frame_idx = idx;
                        self.solution = Some(dsol.frames[idx].solution.clone());
                    }
                    ctx.request_repaint();
                }
            }
        } else if self.animate_deformed && self.show_deformed {
            let dt = ctx.input(|i| i.stable_dt) as f64;
            self.anim_time += dt * 3.2;
            ctx.request_repaint();
        }

        // 1. Top Command Toolbar
        egui::Panel::top("top_toolbar")
            .exact_size(40.0)
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(
                        RichText::new("◆ Beam3D")
                            .strong()
                            .size(15.5)
                            .color(Color32::from_rgb(105, 185, 255)),
                    );
                    ui.separator();

                    egui::ComboBox::from_id_salt("templates_menu")
                        .selected_text("📂 Templates...")
                        .width(145.0)
                        .show_ui(ui, |ui| {
                            for kind in TemplateKind::ALL {
                                if ui.selectable_label(false, kind.title()).clicked() {
                                    self.load_model(
                                        kind.build(),
                                        format!("Loaded template: {}", kind.title()),
                                    );
                                }
                            }
                            ui.separator();
                            if ui
                                .selectable_label(false, "✨ New Empty Model")
                                .clicked()
                            {
                                self.load_model(
                                    StructuralModel::default(),
                                    "Created new empty workspace.",
                                );
                            }
                        });

                    if ui.button("Open...").clicked() {
                        self.open_json_dialog();
                    }
                    if ui.button("Save...").clicked() {
                        self.save_json_dialog();
                    }

                    egui::ComboBox::from_id_salt("tutorials_menu")
                        .selected_text("🎓 Tutorials")
                        .width(120.0)
                        .show_ui(ui, |ui| {
                            for track in TutorialTrack::ALL {
                                if ui.selectable_label(false, track.title()).clicked() {
                                    start_tutorial(self, track);
                                }
                            }
                        });

                    ui.separator();
                    if ui
                        .add_enabled(!self.undo_stack.is_empty(), egui::Button::new("↶ Undo"))
                        .clicked()
                    {
                        self.undo();
                    }
                    if ui
                        .add_enabled(!self.redo_stack.is_empty(), egui::Button::new("↷ Redo"))
                        .clicked()
                    {
                        self.redo();
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.toggle_value(&mut self.show_bottom_panel, "📊 Charts & Tables");
                        ui.checkbox(&mut self.auto_solve, "Auto-Solve");
                        let solve_btn = egui::Button::new(
                            RichText::new("▶ Solve FEA")
                                .strong()
                                .color(Color32::from_rgb(15, 30, 20)),
                        )
                        .fill(Color32::from_rgb(80, 215, 135));
                        if ui.add(solve_btn).clicked() {
                            self.run_solver();
                        }

                        let mut next_mode = self.analysis_mode;
                        egui::ComboBox::from_id_salt("analysis_mode_selector")
                            .selected_text(format!("⚙ {}", self.analysis_mode.short_label()))
                            .width(132.0)
                            .show_ui(ui, |ui| {
                                for m in AnalysisMode::ALL {
                                    ui.selectable_value(&mut next_mode, m, m.label());
                                }
                            });
                        if next_mode != self.analysis_mode {
                            self.analysis_mode = next_mode;
                            if matches!(
                                next_mode,
                                AnalysisMode::NonlinearStatic
                                    | AnalysisMode::Modal
                                    | AnalysisMode::Dynamic
                            ) {
                                self.bottom_tab = BottomTab::AnalysisCharts;
                                self.show_bottom_panel = true;
                            }
                            self.run_solver();
                        }
                    });
                });
            });

        // 1b. Modeling & 3D Working Plane Sub-Toolbar (prevents horizontal crowding!)
        egui::Panel::top("modeling_toolbar")
            .exact_size(32.0)
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(RichText::new("Tool:").small().color(Color32::from_gray(170)));
                    if ui
                        .selectable_label(self.tool == EditorTool::Select, "🖱 Select")
                        .clicked()
                    {
                        self.tool = EditorTool::Select;
                        self.beam_draw_start = None;
                        self.shell_draw_points.clear();
                    }
                    if ui
                        .selectable_label(self.tool == EditorTool::AddNode, "⊕ Add Node")
                        .clicked()
                    {
                        self.tool = EditorTool::AddNode;
                        self.beam_draw_start = None;
                        self.shell_draw_points.clear();
                    }
                    if ui
                        .selectable_label(self.tool == EditorTool::DrawBeam, "📏 Draw Beam")
                        .clicked()
                    {
                        self.tool = EditorTool::DrawBeam;
                        self.shell_draw_points.clear();
                    }
                    if ui
                        .selectable_label(self.tool == EditorTool::DrawShell, "▦ Draw Shell")
                        .on_hover_text("Click 4 corners to generate a 2D shell mesh (or 3 corners + Enter for a Tri3 shell)")
                        .clicked()
                    {
                        self.tool = EditorTool::DrawShell;
                        self.beam_draw_start = None;
                    }

                    ui.separator();
                    ui.label(
                        RichText::new("Plane:")
                            .small()
                            .color(Color32::from_gray(170)),
                    );
                    let mut chosen_plane = self.workplane_kind;
                    for pk in WorkplaneKind::ALL {
                        if ui
                            .selectable_label(self.workplane_kind == pk, pk.short_label())
                            .on_hover_text(format!("{} — press [P] to cycle", pk.full_label()))
                            .clicked()
                        {
                            chosen_plane = pk;
                        }
                    }
                    if chosen_plane != self.workplane_kind {
                        self.set_workplane_kind(chosen_plane);
                    }

                    ui.label(
                        RichText::new(format!("{}:", self.workplane_kind.normal_axis_name()))
                            .small(),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.workplane_offset)
                            .speed(0.25)
                            .suffix("m"),
                    );
                    ui.label(RichText::new("Snap:").small());
                    ui.add(
                        egui::DragValue::new(&mut self.snap_grid)
                            .speed(0.05)
                            .range(0.0..=5.0)
                            .suffix("m"),
                    );

                    if self.tool == EditorTool::DrawShell {
                        ui.separator();
                        ui.label(RichText::new("Mesh:").small());
                        ui.add(egui::DragValue::new(&mut self.new_shell_nx).range(1..=20));
                        ui.label(RichText::new("×").small());
                        ui.add(egui::DragValue::new(&mut self.new_shell_ny).range(1..=20));
                        ui.label(RichText::new("t:").small());
                        ui.add(
                            egui::DragValue::new(&mut self.new_shell_thickness_mm)
                                .speed(5.0)
                                .range(2.0..=2000.0)
                                .suffix(" mm"),
                        );
                        ui.checkbox(&mut self.new_shell_apply_load, "2D Load:");
                        if self.new_shell_apply_load {
                            ui.add(
                                egui::DragValue::new(&mut self.new_shell_pressure_kpa)
                                    .speed(0.5)
                                    .suffix(" kN/m²"),
                            );
                        }
                    }
                });
            });

        // 2. Bottom Status Bar
        egui::Panel::bottom("status_bar")
            .exact_size(24.0)
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(
                        RichText::new(&self.status_msg)
                            .small()
                            .color(Color32::from_gray(195)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(
                                "Orbit: Drag/Right-Drag | Pan: Shift+Drag | Plane: [P] | Offset: [ / ] | Fit: [F]",
                            )
                            .small()
                            .color(Color32::from_gray(140)),
                        );
                    });
                });
            });

        // 3. Left Model Builder Panel
        egui::Panel::left("left_model_panel")
            .default_size(300.0)
            .size_range(260.0..=420.0)
            .show(ui, |ui| {
                draw_left_panel(self, ui);
            });

        // 4. Right Inspector & Post-Processing Panel
        egui::Panel::right("right_inspector_panel")
            .default_size(305.0)
            .size_range(265.0..=420.0)
            .show(ui, |ui| {
                draw_right_panel(self, ui);
            });

        // 5. Optional Bottom 2D Member Diagrams & Reaction Tables Panel
        if self.show_bottom_panel {
            egui::Panel::bottom("bottom_results_panel")
                .default_size(205.0)
                .size_range(145.0..=360.0)
                .resizable(true)
                .show(ui, |ui| {
                    draw_bottom_panel(self, ui);
                });
        }

        // 6. Central 3D Hardware-Accelerated Viewport
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(Color32::from_rgb(19, 22, 29)))
            .show(ui, |ui| {
                draw_central_3d_viewport(self, ui, frame);
            });

        // 7. Interactive Guided Tutorial Window (when active)
        draw_tutorial_window(self, &ctx);
    }
}

fn handle_global_shortcuts(app: &mut Beam3dApp, ctx: &egui::Context) {
    if ctx.egui_wants_keyboard_input() {
        return;
    }
    let mut cycle_plane = false;
    ctx.input(|i| {
        if i.key_pressed(egui::Key::F) {
            let (min_p, max_p) = app.model.bounding_box();
            app.camera.fit_to_bounds(min_p, max_p);
        }
        if i.key_pressed(egui::Key::P) {
            cycle_plane = true;
        }
        let step = if app.snap_grid > 1e-3 {
            app.snap_grid
        } else {
            0.5
        };
        if i.key_pressed(egui::Key::CloseBracket) {
            app.workplane_offset += step;
            app.status_msg = format!(
                "Working plane {}: {} = {:.2} m",
                app.workplane_kind.short_label(),
                app.workplane_kind.normal_axis_name(),
                app.workplane_offset
            );
        }
        if i.key_pressed(egui::Key::OpenBracket) {
            app.workplane_offset -= step;
            app.status_msg = format!(
                "Working plane {}: {} = {:.2} m",
                app.workplane_kind.short_label(),
                app.workplane_kind.normal_axis_name(),
                app.workplane_offset
            );
        }
        if i.key_pressed(egui::Key::Enter)
            && app.tool == EditorTool::DrawShell
            && app.shell_draw_points.len() == 3
        {
            let pts = app.shell_draw_points.clone();
            app.shell_draw_points.clear();
            let mat_id = app.model.materials.last().map(|m| m.id).unwrap_or(1);
            app.push_undo();
            let n0 = app.model.find_or_add_node(pts[0], 1e-3);
            let n1 = app.model.find_or_add_node(pts[1], 1e-3);
            let n2 = app.model.find_or_add_node(pts[2], 1e-3);
            if let Some(sid) =
                app.model
                    .add_shell_tri([n0, n1, n2], mat_id, app.new_shell_thickness_mm * 1e-3)
            {
                if app.new_shell_apply_load && app.new_shell_pressure_kpa.abs() > 1e-6 {
                    app.model.apply_surface_load_to_shells(
                        &[sid],
                        LoadCoordSystem::Global,
                        [0.0, 0.0, app.new_shell_pressure_kpa * 1e3],
                        false,
                    );
                }
                app.selected_shell = Some(sid);
                app.selected_node = None;
                app.selected_element = None;
                app.mark_model_changed();
                app.status_msg = format!("Created Tri3 Shell S{sid}.");
            }
        }
        if i.key_pressed(egui::Key::Escape) {
            if app.beam_draw_start.is_some() || !app.shell_draw_points.is_empty() {
                app.beam_draw_start = None;
                app.shell_draw_points.clear();
            } else {
                app.tool = EditorTool::Select;
                app.selected_node = None;
                app.selected_element = None;
                app.selected_shell = None;
            }
        }
        if i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace) {
            if let Some(nid) = app.selected_node.take() {
                app.push_undo();
                app.model.remove_node(nid);
                app.mark_model_changed();
            } else if let Some(eid) = app.selected_element.take() {
                app.push_undo();
                app.model.remove_element(eid);
                app.mark_model_changed();
            } else if let Some(sid) = app.selected_shell.take() {
                app.push_undo();
                app.model.remove_shell(sid);
                app.mark_model_changed();
            }
        }
        if i.modifiers.command && i.key_pressed(egui::Key::Z) {
            if i.modifiers.shift {
                app.redo();
            } else {
                app.undo();
            }
        }
    });
    if cycle_plane {
        let next = app.workplane_kind.next();
        app.set_workplane_kind(next);
    }
}

fn draw_central_3d_viewport(app: &mut Beam3dApp, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
    let rect = ui.available_rect_before_wrap();
    let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());

    // Camera navigation
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.01 {
            app.camera.zoom(scroll);
        }
    }

    let shift_held = ui.input(|i| i.modifiers.shift);
    if response.dragged_by(egui::PointerButton::Middle)
        || (shift_held
            && (response.dragged_by(egui::PointerButton::Primary)
                || response.dragged_by(egui::PointerButton::Secondary)))
    {
        app.camera.pan(response.drag_delta(), rect.height());
    } else if response.dragged_by(egui::PointerButton::Secondary)
        || (app.tool == EditorTool::Select && response.dragged_by(egui::PointerButton::Primary))
    {
        app.camera.orbit(response.drag_delta());
    }

    // 3D Hover picking & workplane cursor snapping
    app.hovered_node = None;
    app.hovered_element = None;
    app.hovered_shell = None;
    let mut preview_point: Option<[f64; 3]> = None;

    if let Some(cursor) = response.hover_pos() {
        // 1. Closest node in screen pixels
        let mut best_node_dist = 14.0_f32;
        for n in &app.model.nodes {
            if let Some((sp, _)) = app.camera.project_to_screen(n.position, rect) {
                let d = (sp - cursor).length();
                if d < best_node_dist {
                    best_node_dist = d;
                    app.hovered_node = Some(n.id);
                }
            }
        }

        // 2. Closest beam segment if not directly hovering a node
        if app.hovered_node.is_none() {
            let mut best_elem_dist = 9.5_f32;
            for el in &app.model.elements {
                if let (Some(ni), Some(nj)) =
                    (app.model.get_node(el.node_i), app.model.get_node(el.node_j))
                {
                    if let (Some((sa, _)), Some((sb, _))) = (
                        app.camera.project_to_screen(ni.position, rect),
                        app.camera.project_to_screen(nj.position, rect),
                    ) {
                        let d = point_to_segment_dist_2d(cursor, sa, sb);
                        if d < best_elem_dist {
                            best_elem_dist = d;
                            app.hovered_element = Some(el.id);
                        }
                    }
                }
            }
        }

        // 2b. Closest 2D shell surface if not hovering a node or beam
        if app.hovered_node.is_none() && app.hovered_element.is_none() {
            let mut best_depth = f32::INFINITY;
            for shell in &app.model.shells {
                let ids = shell.nodes.as_slice();
                let mut proj: Vec<(Pos2, f32)> = Vec::with_capacity(ids.len());
                for &nid in ids {
                    if let Some(n) = app.model.get_node(nid) {
                        if let Some(sp) = app.camera.project_to_screen(n.position, rect) {
                            proj.push(sp);
                        }
                    }
                }
                if proj.len() == ids.len() {
                    let tris: &[[usize; 3]] = if proj.len() == 3 {
                        &[[0, 1, 2]]
                    } else {
                        &[[0, 1, 2], [0, 2, 3]]
                    };
                    for &[a, b, c] in tris {
                        if point_in_triangle_2d(cursor, proj[a].0, proj[b].0, proj[c].0) {
                            let avg_z = (proj[a].1 + proj[b].1 + proj[c].1) / 3.0;
                            if avg_z < best_depth {
                                best_depth = avg_z;
                                app.hovered_shell = Some(shell.id);
                            }
                        }
                    }
                }
            }
        }

        // 3. Workplane preview point for AddNode / DrawBeam / DrawShell tools (supports XY, XZ, and YZ planes!)
        if app.tool != EditorTool::Select {
            if let Some(nid) = app.hovered_node {
                preview_point = app.model.get_node(nid).map(|n| n.position);
            } else if let Some(mut hit) = app.camera.intersect_workplane(
                cursor,
                rect,
                app.workplane_kind,
                app.workplane_offset,
            ) {
                if app.snap_grid > 1e-4 {
                    let (ia, ib) = app.workplane_kind.in_plane_indices();
                    hit[ia] = (hit[ia] / app.snap_grid).round() * app.snap_grid;
                    hit[ib] = (hit[ib] / app.snap_grid).round() * app.snap_grid;
                }
                preview_point = Some(hit);
            }
        }
    }

    // Click handling
    if response.clicked_by(egui::PointerButton::Secondary) {
        app.beam_draw_start = None;
        app.shell_draw_points.clear();
    }

    if response.clicked_by(egui::PointerButton::Primary) {
        match app.tool {
            EditorTool::Select => {
                if let Some(nid) = app.hovered_node {
                    app.selected_node = Some(nid);
                    app.selected_element = None;
                    app.selected_shell = None;
                } else if let Some(eid) = app.hovered_element {
                    app.selected_element = Some(eid);
                    app.selected_node = None;
                    app.selected_shell = None;
                } else if let Some(sid) = app.hovered_shell {
                    app.selected_shell = Some(sid);
                    app.selected_node = None;
                    app.selected_element = None;
                } else {
                    app.selected_node = None;
                    app.selected_element = None;
                    app.selected_shell = None;
                }
            }
            EditorTool::AddNode => {
                if let Some(pt) = preview_point {
                    app.push_undo();
                    let nid = app.model.find_or_add_node(pt, 1e-3);
                    app.selected_node = Some(nid);
                    app.selected_element = None;
                    app.selected_shell = None;
                    app.mark_model_changed();
                    app.status_msg =
                        format!("Placed Node N{nid} at ({:.2}, {:.2}, {:.2}) m", pt[0], pt[1], pt[2]);
                }
            }
            EditorTool::DrawBeam => {
                let target_nid = if let Some(nid) = app.hovered_node {
                    Some(nid)
                } else if let Some(pt) = preview_point {
                    app.push_undo();
                    let nid = app.model.find_or_add_node(pt, 1e-3);
                    app.mark_model_changed();
                    Some(nid)
                } else {
                    None
                };

                if let Some(nid) = target_nid {
                    if let Some(start_nid) = app.beam_draw_start {
                        if start_nid != nid {
                            let sec_id = app.model.sections.first().map(|s| s.id).unwrap_or(1);
                            let mat_id = app.model.materials.first().map(|m| m.id).unwrap_or(1);
                            app.push_undo();
                            if let Some(eid) = app.model.add_beam(start_nid, nid, mat_id, sec_id) {
                                app.selected_element = Some(eid);
                                app.selected_node = None;
                                app.selected_shell = None;
                                app.mark_model_changed();
                                app.status_msg = format!(
                                    "Created Beam B{eid} (N{start_nid} → N{nid}). Press [P] to switch plane (XY/XZ/YZ) or Esc to end chain."
                                );
                            }
                            app.beam_draw_start = Some(nid);
                            if let Some(n) = app.model.get_node(nid) {
                                app.workplane_offset =
                                    n.position[app.workplane_kind.normal_index()];
                            }
                        }
                    } else {
                        app.beam_draw_start = Some(nid);
                        if let Some(n) = app.model.get_node(nid) {
                            app.workplane_offset = n.position[app.workplane_kind.normal_index()];
                        }
                        app.status_msg = format!(
                            "Start Node N{nid} selected ({} plane aligned at {} = {:.2} m) — click next point or press [P] to switch plane.",
                            app.workplane_kind.short_label(),
                            app.workplane_kind.normal_axis_name(),
                            app.workplane_offset
                        );
                    }
                }
            }
            EditorTool::DrawShell => {
                if let Some(pt) = preview_point {
                    app.shell_draw_points.push(pt);
                    let count = app.shell_draw_points.len();
                    if count < 4 {
                        app.status_msg = format!(
                            "Draw Shell: Corner {count}/4 placed ({:.2}, {:.2}, {:.2}) m — click corner {}{}",
                            pt[0],
                            pt[1],
                            pt[2],
                            count + 1,
                            if count == 3 { " (or press Enter for a Tri3 shell)" } else { "" }
                        );
                    } else {
                        let corners = [
                            app.shell_draw_points[0],
                            app.shell_draw_points[1],
                            app.shell_draw_points[2],
                            app.shell_draw_points[3],
                        ];
                        app.shell_draw_points.clear();
                        let mat_id = app.model.materials.last().map(|m| m.id).unwrap_or(1);
                        let load_opt = if app.new_shell_apply_load
                            && app.new_shell_pressure_kpa.abs() > 1e-6
                        {
                            Some((
                                LoadCoordSystem::Global,
                                [0.0, 0.0, app.new_shell_pressure_kpa * 1e3],
                            ))
                        } else {
                            None
                        };
                        app.push_undo();
                        let created = app.model.add_shell_mesh_quad(
                            corners,
                            app.new_shell_nx,
                            app.new_shell_ny,
                            mat_id,
                            app.new_shell_thickness_mm * 1e-3,
                            load_opt,
                        );
                        app.selected_shell = created.first().copied();
                        app.selected_node = None;
                        app.selected_element = None;
                        app.mark_model_changed();
                        app.status_msg = format!(
                            "Generated {}×{} 2D shell mesh ({} Quad4 elements, t = {:.0} mm).",
                            app.new_shell_nx,
                            app.new_shell_ny,
                            created.len(),
                            app.new_shell_thickness_mm
                        );
                    }
                }
            }
        }
    }

    // Build 3D scene geometry & render via wgpu
    let anim_factor = if app.analysis_mode == AnalysisMode::Dynamic {
        1.0
    } else if app.animate_deformed {
        if app.analysis_mode == AnalysisMode::Modal {
            app.anim_time.sin()
        } else {
            0.5 + 0.5 * app.anim_time.sin()
        }
    } else {
        1.0
    };

    let disp_opts = ViewportDisplayOptions {
        show_grid: app.show_grid,
        workplane_kind: app.workplane_kind,
        workplane_offset: app.workplane_offset,
        show_extruded: app.show_extruded,
        profile_scale: app.profile_scale,
        show_deformed: app.show_deformed,
        show_undeformed_ghost: app.show_undeformed_ghost,
        disp_scale: app.disp_scale * anim_factor,
        show_supports: app.show_supports,
        show_loads: app.show_loads,
        show_reactions: app.show_reactions,
        show_local_axes: app.show_local_axes,
        color_field: app.color_field,
        diagram_ribbon: app.diagram_ribbon,
        diagram_scale: app.diagram_scale,
        selected_node: app.selected_node,
        hovered_node: app.hovered_node,
        selected_element: app.selected_element,
        hovered_element: app.hovered_element,
        selected_shell: app.selected_shell,
        hovered_shell: app.hovered_shell,
        preview_point,
        beam_draw_start_node: app.beam_draw_start,
        shell_draw_points: app.shell_draw_points.clone(),
    };

    let legend = build_scene_geometry(
        &app.model,
        app.solution.as_ref(),
        &disp_opts,
        &mut app.scene_geo,
    );

    if let (Some(render_state), Some(renderer)) =
        (frame.wgpu_render_state(), app.renderer.as_mut())
    {
        let ppp = ui.ctx().pixels_per_point();
        let pw = (rect.width() * ppp).round() as u32;
        let ph = (rect.height() * ppp).round() as u32;
        let aspect = rect.width() / rect.height().max(1.0);
        let vp = app.camera.view_proj_matrix(aspect);
        let eye = app.camera.eye_position();
        let cam_uniform = CameraUniform {
            view_proj: vp.into(),
            eye_pos: [eye.x, eye.y, eye.z, 1.0],
        };

        renderer.render(
            render_state,
            pw,
            ph,
            &cam_uniform,
            &app.scene_geo,
            [0.075, 0.088, 0.114, 1.0],
        );

        ui.painter().image(
            renderer.egui_tex_id,
            rect,
            Rect::from_min_max(Pos2::ZERO, egui::pos2(1.0, 1.0)),
            Color32::WHITE,
        );
    }

    // 2D Vector HUD Overlays on top of the 3D Viewport
    let painter = ui.painter_at(rect);

    // Optional Node ID labels
    if app.show_node_labels {
        let eff_scale = if app.show_deformed && app.solution.is_some() {
            app.disp_scale * anim_factor
        } else {
            0.0
        };
        for n in &app.model.nodes {
            let mut p = n.position;
            if eff_scale.abs() > 1e-9 {
                if let Some(nr) = app.solution.as_ref().and_then(|s| s.node_results.get(&n.id)) {
                    p[0] += eff_scale * nr.displacements[0];
                    p[1] += eff_scale * nr.displacements[1];
                    p[2] += eff_scale * nr.displacements[2];
                }
            }
            if let Some((sp, _)) = app.camera.project_to_screen(p, rect) {
                painter.text(
                    sp + egui::vec2(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    format!("N{}", n.id),
                    egui::FontId::monospace(11.0),
                    Color32::from_rgb(225, 235, 250),
                );
            }
        }
    }

    // Hover Tooltip Readout
    if let Some(cursor) = response.hover_pos() {
        if let Some(nid) = app.hovered_node {
            if let Some(n) = app.model.get_node(nid) {
                let mut tip = format!(
                    "Node N{} ({:.2}, {:.2}, {:.2}) m",
                    n.id, n.position[0], n.position[1], n.position[2]
                );
                if let Some(nr) = app.solution.as_ref().and_then(|s| s.node_results.get(&nid)) {
                    let umag = (nr.displacements[0].powi(2)
                        + nr.displacements[1].powi(2)
                        + nr.displacements[2].powi(2))
                    .sqrt()
                        * 1e3;
                    tip.push_str(&format!("\n|u| = {umag:.3} mm"));
                }
                draw_floating_badge(&painter, cursor + egui::vec2(14.0, 14.0), &tip);
            }
        } else if let Some(eid) = app.hovered_element {
            if let Some(el) = app.model.get_element(eid) {
                let sec_name = app
                    .model
                    .get_section(el.section_id)
                    .map(|s| s.name.as_str())
                    .unwrap_or("?");
                let mut tip = format!("Beam B{} ({})", el.id, sec_name);
                if let Some(er) = app
                    .solution
                    .as_ref()
                    .and_then(|s| s.element_results.get(&eid))
                {
                    tip.push_str(&format!(
                        "\nL = {:.2} m | Util = {:.1}%",
                        er.length,
                        er.max_utilization * 100.0
                    ));
                }
                draw_floating_badge(&painter, cursor + egui::vec2(14.0, 14.0), &tip);
            }
        } else if let Some(sid) = app.hovered_shell {
            if let Some(sh) = app.model.get_shell(sid) {
                let mut tip = format!("Shell S{} (t = {:.0} mm)", sh.id, sh.thickness * 1e3);
                if let Some(sr) = app
                    .solution
                    .as_ref()
                    .and_then(|s| s.shell_results.get(&sid))
                {
                    tip.push_str(&format!(
                        "\n|u| = {:.3} mm | σ_vm = {:.2} MPa | Util = {:.1}%",
                        sr.centroid_disp_mag_m * 1e3,
                        sr.max_von_mises_pa * 1e-6,
                        sr.utilization * 100.0
                    ));
                }
                draw_floating_badge(&painter, cursor + egui::vec2(14.0, 14.0), &tip);
            }
        }
    }

    // Bottom-Left Orientation Gizmo Triad
    draw_orientation_gizmo(&painter, &app.camera, rect);

    // Bottom-Right Scientific Colorbar Legend
    if let Some(leg) = legend {
        draw_colorbar_legend(&painter, rect, leg);
    }

    // Top-Left Camera Preset Overlay Buttons
    let overlay_rect = Rect::from_min_size(rect.min + egui::vec2(10.0, 10.0), egui::vec2(320.0, 30.0));
    ui.scope_builder(egui::UiBuilder::new().max_rect(overlay_rect), |ui| {
        egui::Frame::NONE
            .fill(Color32::from_rgba_unmultiplied(22, 27, 36, 210))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(egui::Margin::symmetric(8, 4))
            .stroke(Stroke::new(1.0, Color32::from_rgb(52, 62, 80)))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.small_button("⛶ Fit [F]").clicked() {
                        let (min_p, max_p) = app.model.bounding_box();
                        app.camera.fit_to_bounds(min_p, max_p);
                    }
                    if ui.small_button("Iso").clicked() {
                        app.camera.set_preset_iso();
                    }
                    if ui.small_button("Front X-Z").clicked() {
                        app.camera.set_preset_front_xz();
                        app.set_workplane_kind(WorkplaneKind::XZ);
                    }
                    if ui.small_button("Side Y-Z").clicked() {
                        app.camera.set_preset_side_yz();
                        app.set_workplane_kind(WorkplaneKind::YZ);
                    }
                    if ui.small_button("Top X-Y").clicked() {
                        app.camera.set_preset_top_xy();
                        app.set_workplane_kind(WorkplaneKind::XY);
                    }
                });
            });
    });

    // Top-Right 3D Drawing HUD when AddNode or DrawBeam tool is active
    if app.tool != EditorTool::Select {
        let hud_w = 365.0;
        let hud_rect = Rect::from_min_size(
            egui::pos2(rect.right() - hud_w - 10.0, rect.top() + 10.0),
            egui::vec2(hud_w, 88.0),
        );
        ui.scope_builder(egui::UiBuilder::new().max_rect(hud_rect), |ui| {
            egui::Frame::NONE
                .fill(Color32::from_rgba_unmultiplied(18, 28, 42, 232))
                .corner_radius(CornerRadius::same(6))
                .inner_margin(egui::Margin::symmetric(8, 5))
                .stroke(Stroke::new(1.0, Color32::from_rgb(70, 145, 225)))
                .show(ui, |ui| {
                    let mode_name = match app.tool {
                        EditorTool::AddNode => "⊕ ADD NODE",
                        EditorTool::DrawBeam => "📏 DRAW BEAM",
                        EditorTool::DrawShell => "▦ DRAW SHELL (4 pts)",
                        EditorTool::Select => "",
                    };
                    let axis_name = app.workplane_kind.normal_axis_name();
                    let coord_str = if let Some(nid) = app.hovered_node {
                        format!("Snap → Node N{nid}")
                    } else if let Some(pt) = preview_point {
                        format!("XYZ: ({:.2}, {:.2}, {:.2}) m", pt[0], pt[1], pt[2])
                    } else {
                        format!(
                            "Plane {} ({axis_name} = {:.2} m) — orbit slightly if edge-on",
                            app.workplane_kind.short_label(),
                            app.workplane_offset
                        )
                    };
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(mode_name)
                                .small()
                                .strong()
                                .color(Color32::from_rgb(100, 220, 255)),
                        );
                        ui.separator();
                        ui.label(
                            RichText::new(coord_str)
                                .monospace()
                                .small()
                                .color(Color32::from_rgb(225, 238, 255)),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Plane [P]:").small());
                        let mut next_pk = app.workplane_kind;
                        for (pk, lbl) in [
                            (WorkplaneKind::XY, "XY Floor"),
                            (WorkplaneKind::XZ, "XZ Vert"),
                            (WorkplaneKind::YZ, "YZ Vert"),
                        ] {
                            if ui.selectable_label(app.workplane_kind == pk, lbl).clicked() {
                                next_pk = pk;
                            }
                        }
                        if next_pk != app.workplane_kind {
                            app.set_workplane_kind(next_pk);
                        }
                    });

                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format!("Offset {axis_name}:")).small());
                        for preset in [0.0, 3.0, 6.0] {
                            let active = (app.workplane_offset - preset).abs() < 1e-3;
                            if ui
                                .selectable_label(active, format!("{preset:.0}m"))
                                .clicked()
                            {
                                app.workplane_offset = preset;
                            }
                        }
                        let step = app.snap_grid.max(0.5);
                        if ui.small_button(format!("-{axis_name} [")).clicked() {
                            app.workplane_offset -= step;
                        }
                        if ui.small_button(format!("+{axis_name} ]")).clicked() {
                            app.workplane_offset += step;
                        }
                        if app.beam_draw_start.is_some() && ui.small_button("End Chain").clicked() {
                            app.beam_draw_start = None;
                        }
                    });
                });
        });
    }
}

fn draw_floating_badge(painter: &egui::Painter, pos: Pos2, text: &str) {
    let galley = painter.layout_no_wrap(
        text.to_string(),
        egui::FontId::monospace(11.5),
        Color32::from_rgb(235, 242, 252),
    );
    let pad = egui::vec2(7.0, 5.0);
    let box_rect = Rect::from_min_size(pos, galley.size() + pad * 2.0);
    painter.rect_filled(
        box_rect,
        5.0,
        Color32::from_rgba_unmultiplied(16, 20, 28, 230),
    );
    painter.rect_stroke(
        box_rect,
        5.0,
        Stroke::new(1.0, Color32::from_rgb(80, 125, 190)),
        egui::StrokeKind::Inside,
    );
    painter.galley(pos + pad, galley, Color32::WHITE);
}

fn draw_orientation_gizmo(painter: &egui::Painter, camera: &OrbitCamera, rect: Rect) {
    let origin = egui::pos2(rect.left() + 46.0, rect.bottom() - 46.0);
    let view = camera.view_matrix();
    let len = 28.0;

    let axes = [
        (
            nalgebra::Vector3::new(1.0, 0.0, 0.0),
            "X",
            Color32::from_rgb(245, 80, 80),
        ),
        (
            nalgebra::Vector3::new(0.0, 1.0, 0.0),
            "Y",
            Color32::from_rgb(80, 225, 110),
        ),
        (
            nalgebra::Vector3::new(0.0, 0.0, 1.0),
            "Z",
            Color32::from_rgb(85, 155, 255),
        ),
    ];

    painter.circle_filled(origin, 34.0, Color32::from_rgba_unmultiplied(18, 22, 30, 175));

    for (axis, lbl, col) in axes {
        let v = view.transform_vector(&axis);
        let tip = origin + egui::vec2(v.x * len, -v.y * len);
        painter.line_segment([origin, tip], Stroke::new(2.2, col));
        painter.text(
            tip + egui::vec2(v.x * 7.0, -v.y * 7.0),
            egui::Align2::CENTER_CENTER,
            lbl,
            egui::FontId::monospace(11.0),
            col,
        );
    }
}

fn draw_colorbar_legend(painter: &egui::Painter, rect: Rect, leg: LegendRange) {
    let panel_w = 215.0;
    let panel_h = 54.0;
    let panel = Rect::from_min_size(
        egui::pos2(rect.right() - panel_w - 12.0, rect.bottom() - panel_h - 12.0),
        egui::vec2(panel_w, panel_h),
    );
    painter.rect_filled(
        panel,
        6.0,
        Color32::from_rgba_unmultiplied(18, 22, 30, 218),
    );
    painter.rect_stroke(
        panel,
        6.0,
        Stroke::new(1.0, Color32::from_rgb(54, 65, 84)),
        egui::StrokeKind::Inside,
    );

    painter.text(
        egui::pos2(panel.left() + 8.0, panel.top() + 6.0),
        egui::Align2::LEFT_TOP,
        leg.field.label(),
        egui::FontId::proportional(11.5),
        Color32::from_rgb(215, 225, 242),
    );

    let bar = Rect::from_min_size(
        egui::pos2(panel.left() + 8.0, panel.top() + 23.0),
        egui::vec2(panel_w - 16.0, 11.0),
    );
    let n_strips = 36usize;
    let strip_w = bar.width() / n_strips as f32;
    for i in 0..n_strips {
        let t = (i as f32 + 0.5) / n_strips as f32;
        let rgba = if leg.diverging {
            colormap_diverging(t * 2.0 - 1.0)
        } else {
            colormap_scientific(t)
        };
        let c = Color32::from_rgb(
            (rgba[0] * 255.0) as u8,
            (rgba[1] * 255.0) as u8,
            (rgba[2] * 255.0) as u8,
        );
        let sr = Rect::from_min_size(
            egui::pos2(bar.left() + i as f32 * strip_w, bar.top()),
            egui::vec2(strip_w + 0.6, bar.height()),
        );
        painter.rect_filled(sr, 0.0, c);
    }
    painter.rect_stroke(
        bar,
        2.0,
        Stroke::new(1.0, Color32::from_gray(90)),
        egui::StrokeKind::Outside,
    );

    let unit = leg.field.unit();
    let mid = 0.5 * (leg.min_display + leg.max_display);
    painter.text(
        egui::pos2(bar.left(), bar.bottom() + 3.0),
        egui::Align2::LEFT_TOP,
        format!("{:.2}", leg.min_display),
        egui::FontId::monospace(10.0),
        Color32::from_gray(185),
    );
    painter.text(
        egui::pos2(bar.center().x, bar.bottom() + 3.0),
        egui::Align2::CENTER_TOP,
        format!("{mid:.2}"),
        egui::FontId::monospace(10.0),
        Color32::from_gray(165),
    );
    painter.text(
        egui::pos2(bar.right(), bar.bottom() + 3.0),
        egui::Align2::RIGHT_TOP,
        format!("{:.2} {unit}", leg.max_display),
        egui::FontId::monospace(10.0),
        Color32::from_gray(215),
    );
}

fn configure_engineering_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = Color32::from_rgb(24, 28, 36);
    visuals.window_fill = Color32::from_rgb(26, 31, 40);
    visuals.extreme_bg_color = Color32::from_rgb(15, 18, 24);
    visuals.faint_bg_color = Color32::from_rgb(30, 35, 45);
    visuals.selection.bg_fill = Color32::from_rgb(48, 108, 185);
    visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(28, 33, 43);
    visuals.widgets.inactive.bg_fill = Color32::from_rgb(36, 43, 56);
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(50, 62, 82);
    visuals.widgets.active.bg_fill = Color32::from_rgb(60, 118, 195);
    ctx.set_visuals(visuals);
}

fn point_in_triangle_2d(p: Pos2, a: Pos2, b: Pos2, c: Pos2) -> bool {
    let sign = |p1: Pos2, p2: Pos2, p3: Pos2| -> f32 {
        (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
    };
    let d1 = sign(p, a, b);
    let d2 = sign(p, b, c);
    let d3 = sign(p, c, a);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}
