mod app;
mod camera;
mod mesh_gen;
mod panels;
mod renderer;
mod tutorials;

use app::Beam3dApp;
use eframe::egui;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Beam3D — 3D Structural Space-Frame Finite Element Solver")
            .with_inner_size([1480.0, 900.0])
            .with_min_inner_size([1024.0, 640.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Beam3D",
        native_options,
        Box::new(|cc| Ok(Box::new(Beam3dApp::new(cc)))),
    )
}

#[cfg(test)]
mod tests {
    use crate::camera::{OrbitCamera, WorkplaneKind};
    use crate::mesh_gen::{
        ColorField, DiagramRibbon, ViewportDisplayOptions, build_scene_geometry,
    };
    use crate::renderer::{CameraUniform, SceneGeometry, ViewportRenderer};
    use beam3d_core::{StructuralModel, TemplateKind, solve_linear_static};
    use eframe::{egui_wgpu, wgpu};

    #[test]
    fn test_scene_generation_and_json_roundtrip_all_templates() {
        let mut scene = SceneGeometry::default();
        for kind in TemplateKind::ALL {
            let model = kind.build();
            let sol_orig = solve_linear_static(&model).expect("solve orig");
            let json = model.to_json_pretty().expect("serialize JSON");
            let parsed = StructuralModel::from_json(&json).expect("deserialize JSON");
            assert_eq!(model.nodes.len(), parsed.nodes.len());
            assert_eq!(model.elements.len(), parsed.elements.len());

            let sol = solve_linear_static(&parsed).expect("solve template");
            assert!(
                (sol_orig.summary.max_displacement_m - sol.summary.max_displacement_m).abs()
                    < 1e-12
            );
            for wp in WorkplaneKind::ALL {
                let opts = ViewportDisplayOptions {
                    show_grid: true,
                    workplane_kind: wp,
                    workplane_offset: 1.5,
                    show_extruded: true,
                    profile_scale: 1.0,
                    show_deformed: true,
                    show_undeformed_ghost: true,
                    disp_scale: 25.0,
                    show_supports: true,
                    show_loads: true,
                    show_reactions: true,
                    show_local_axes: true,
                    color_field: ColorField::VonMisesStress,
                    diagram_ribbon: DiagramRibbon::MomentMy,
                    diagram_scale: 1.0,
                    selected_node: parsed.nodes.first().map(|n| n.id),
                    hovered_node: None,
                    selected_element: parsed.elements.first().map(|e| e.id),
                    hovered_element: None,
                    preview_point: Some([1.0, 2.0, 1.5]),
                    beam_draw_start_node: parsed.nodes.first().map(|n| n.id),
                };
                build_scene_geometry(&parsed, Some(&sol), &opts, &mut scene);
                assert!(!scene.opaque_tris.is_empty());
                assert!(!scene.lines.is_empty());
            }

            for cf in ColorField::ALL {
                for ribbon in DiagramRibbon::ALL {
                    let opts = ViewportDisplayOptions {
                        show_grid: true,
                        workplane_kind: WorkplaneKind::XY,
                        workplane_offset: 0.0,
                        show_extruded: true,
                        profile_scale: 1.0,
                        show_deformed: true,
                        show_undeformed_ghost: true,
                        disp_scale: 25.0,
                        show_supports: true,
                        show_loads: true,
                        show_reactions: true,
                        show_local_axes: true,
                        color_field: cf,
                        diagram_ribbon: ribbon,
                        diagram_scale: 1.0,
                        selected_node: parsed.nodes.first().map(|n| n.id),
                        hovered_node: None,
                        selected_element: parsed.elements.first().map(|e| e.id),
                        hovered_element: None,
                        preview_point: Some([1.0, 2.0, 0.0]),
                        beam_draw_start_node: parsed.nodes.first().map(|n| n.id),
                    };
                    build_scene_geometry(&parsed, Some(&sol), &opts, &mut scene);
                    assert!(!scene.opaque_tris.is_empty());
                    assert!(!scene.lines.is_empty());
                }
            }
        }
    }

    #[test]
    fn test_wgpu_metal_pipeline_and_offscreen_render() {
        use eframe::egui::mutex::RwLock;
        use std::future::Future;
        use std::pin::Pin;
        use std::sync::Arc;
        use std::task::{Context, Poll, Waker};

        fn spin_block_on<F: Future>(mut fut: F) -> F::Output {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);
            // SAFETY: fut is stack-pinned for the duration of this loop.
            let mut pinned = unsafe { Pin::new_unchecked(&mut fut) };
            loop {
                if let Poll::Ready(val) = pinned.as_mut().poll(&mut cx) {
                    return val;
                }
                std::thread::yield_now();
            }
        }

        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let Ok(adapter) = spin_block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        })) else {
            return;
        };

        let (device, queue) = spin_block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
            .expect("create Metal wgpu device");

        let egui_renderer = egui_wgpu::Renderer::new(
            &device,
            wgpu::TextureFormat::Rgba8Unorm,
            egui_wgpu::RendererOptions::default(),
        );

        let render_state = egui_wgpu::RenderState {
            adapter,
            available_adapters: Vec::new(),
            instance,
            device,
            queue,
            target_format: wgpu::TextureFormat::Rgba8Unorm,
            renderer: Arc::new(RwLock::new(egui_renderer)),
        };

        let mut vp_renderer = ViewportRenderer::new(&render_state);
        let model = TemplateKind::TiedArchBridge.build();
        let sol = solve_linear_static(&model).expect("solve bridge");
        let mut scene = SceneGeometry::default();
        let opts = ViewportDisplayOptions {
            show_grid: true,
            workplane_kind: WorkplaneKind::XZ,
            workplane_offset: 0.0,
            show_extruded: true,
            profile_scale: 1.0,
            show_deformed: true,
            show_undeformed_ghost: true,
            disp_scale: 30.0,
            show_supports: true,
            show_loads: true,
            show_reactions: true,
            show_local_axes: true,
            color_field: ColorField::VonMisesStress,
            diagram_ribbon: DiagramRibbon::MomentMy,
            diagram_scale: 1.0,
            selected_node: None,
            hovered_node: None,
            selected_element: Some(1),
            hovered_element: None,
            preview_point: None,
            beam_draw_start_node: None,
        };
        build_scene_geometry(&model, Some(&sol), &opts, &mut scene);

        let mut cam = OrbitCamera::default();
        let (bmin, bmax) = model.bounding_box();
        cam.fit_to_bounds(bmin, bmax);
        let vp = cam.view_proj_matrix(1280.0 / 800.0);
        let eye = cam.eye_position();
        let uniform = CameraUniform {
            view_proj: vp.into(),
            eye_pos: [eye.x, eye.y, eye.z, 1.0],
        };

        vp_renderer.render(
            &render_state,
            1280,
            800,
            &uniform,
            &scene,
            [0.075, 0.088, 0.114, 1.0],
        );
    }
}
