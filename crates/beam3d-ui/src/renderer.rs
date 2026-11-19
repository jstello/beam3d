use bytemuck::{Pod, Zeroable};
use eframe::{egui, egui_wgpu, wgpu};
use std::borrow::Cow;
use std::num::NonZeroU64;

const MSAA_SAMPLES: u32 = 4;
const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct MeshVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct LineVertex {
    pub pos: [f32; 3],
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
    pub eye_pos: [f32; 4],
}

#[derive(Default)]
pub struct SceneGeometry {
    pub opaque_tris: Vec<MeshVertex>,
    pub transparent_tris: Vec<MeshVertex>,
    pub lines: Vec<LineVertex>,
}

impl SceneGeometry {
    pub fn clear(&mut self) {
        self.opaque_tris.clear();
        self.transparent_tris.clear();
        self.lines.clear();
    }
}

const SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    eye_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct MeshIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct MeshOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_mesh(in: MeshIn) -> MeshOut {
    var out: MeshOut;
    out.clip_pos = camera.view_proj * vec4<f32>(in.pos, 1.0);
    out.world_pos = in.pos;
    out.normal = in.normal;
    out.color = in.color;
    return out;
}

@fragment
fn fs_mesh(in: MeshOut) -> @location(0) vec4<f32> {
    let n_len = length(in.normal);
    if n_len < 1e-4 {
        return in.color;
    }
    var n = in.normal / n_len;
    let v = normalize(camera.eye_pos.xyz - in.world_pos);
    // Two-sided CAD lighting so thin plates, caps, and ribbons look crisp from both sides
    if dot(n, v) < 0.0 {
        n = -n;
    }

    let l_key = normalize(vec3<f32>(0.48, -0.58, 0.66));
    let l_fill = normalize(vec3<f32>(-0.65, 0.42, 0.35));
    let sky_hemi = 0.5 * n.z + 0.5;
    let ambient = mix(0.28, 0.44, sky_hemi);

    let diff_key = max(dot(n, l_key), 0.0) * 0.56;
    let diff_fill = max(dot(n, l_fill), 0.0) * 0.22;

    let h = normalize(l_key + v);
    let spec = pow(max(dot(n, h), 0.0), 36.0) * 0.16;

    let lit_rgb = clamp(in.color.rgb * (ambient + diff_key + diff_fill) + vec3<f32>(spec), vec3<f32>(0.0), vec3<f32>(1.0));
    return vec4<f32>(lit_rgb, in.color.a);
}

struct LineIn {
    @location(0) pos: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct LineOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_line(in: LineIn) -> LineOut {
    var out: LineOut;
    var clip = camera.view_proj * vec4<f32>(in.pos, 1.0);
    // Slight forward depth nudge so CAD silhouette & grid lines never z-fight with faces
    clip.z = max(clip.z - 0.00015 * clip.w, 0.0);
    out.clip_pos = clip;
    out.color = in.color;
    return out;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

pub struct ViewportRenderer {
    width: u32,
    height: u32,
    msaa_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    pub egui_tex_id: egui::TextureId,

    camera_buf: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,

    opaque_mesh_pipeline: wgpu::RenderPipeline,
    transparent_mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,

    opaque_vbuf: wgpu::Buffer,
    opaque_cap: usize,
    trans_vbuf: wgpu::Buffer,
    trans_cap: usize,
    line_vbuf: wgpu::Buffer,
    line_cap: usize,
}

impl ViewportRenderer {
    pub fn new(render_state: &egui_wgpu::RenderState) -> Self {
        let device = &render_state.device;
        let width = 800;
        let height = 600;

        let (msaa_view, depth_view, resolve_view) = create_attachments(device, width, height);
        let egui_tex_id = render_state.renderer.write().register_native_texture(
            device,
            &resolve_view,
            wgpu::FilterMode::Linear,
        );

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("beam3d_viewport_shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(SHADER_WGSL)),
        });

        let camera_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("beam3d_camera_uniform"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let camera_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("beam3d_camera_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: NonZeroU64::new(std::mem::size_of::<CameraUniform>() as u64),
                },
                count: None,
            }],
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("beam3d_camera_bg"),
            layout: &camera_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buf.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("beam3d_pipeline_layout"),
            bind_group_layouts: &[Some(&camera_bgl)],
            immediate_size: 0,
        });

        let mesh_attrs =
            wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4];
        let line_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];

        let alpha_blend = wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::SrcAlpha,
                dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::One,
                dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                operation: wgpu::BlendOperation::Add,
            },
        };

        let make_mesh_pipeline = |label: &str, depth_write: bool| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_mesh"),
                    buffers: &[Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<MeshVertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &mesh_attrs,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs_mesh"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: COLOR_FORMAT,
                        blend: Some(alpha_blend),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    cull_mode: None,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(wgpu::CompareFunction::LessEqual),
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: MSAA_SAMPLES,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                multiview_mask: None,
                cache: None,
            })
        };

        let opaque_mesh_pipeline = make_mesh_pipeline("beam3d_opaque_mesh", true);
        let transparent_mesh_pipeline = make_mesh_pipeline("beam3d_transparent_mesh", false);

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("beam3d_line_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<LineVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &line_attrs,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: Some(alpha_blend),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: MSAA_SAMPLES,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview_mask: None,
            cache: None,
        });

        let init_mesh_cap = 32_768;
        let init_line_cap = 32_768;
        let opaque_vbuf = create_vertex_buf::<MeshVertex>(device, "beam3d_opaque_vb", init_mesh_cap);
        let trans_vbuf = create_vertex_buf::<MeshVertex>(device, "beam3d_trans_vb", init_mesh_cap);
        let line_vbuf = create_vertex_buf::<LineVertex>(device, "beam3d_line_vb", init_line_cap);

        Self {
            width,
            height,
            msaa_view,
            depth_view,
            resolve_view,
            egui_tex_id,
            camera_buf,
            camera_bind_group,
            opaque_mesh_pipeline,
            transparent_mesh_pipeline,
            line_pipeline,
            opaque_vbuf,
            opaque_cap: init_mesh_cap,
            trans_vbuf,
            trans_cap: init_mesh_cap,
            line_vbuf,
            line_cap: init_line_cap,
        }
    }

    pub fn render(
        &mut self,
        render_state: &egui_wgpu::RenderState,
        pixel_width: u32,
        pixel_height: u32,
        cam_uniform: &CameraUniform,
        scene: &SceneGeometry,
        bg_color: [f64; 4],
    ) {
        let w = pixel_width.clamp(64, 8192);
        let h = pixel_height.clamp(64, 8192);
        let device = &render_state.device;
        let queue = &render_state.queue;

        if w != self.width || h != self.height {
            self.width = w;
            self.height = h;
            let (msaa_v, depth_v, resolve_v) = create_attachments(device, w, h);
            self.msaa_view = msaa_v;
            self.depth_view = depth_v;
            self.resolve_view = resolve_v;
            render_state
                .renderer
                .write()
                .update_egui_texture_from_wgpu_texture(
                    device,
                    &self.resolve_view,
                    wgpu::FilterMode::Linear,
                    self.egui_tex_id,
                );
        }

        queue.write_buffer(&self.camera_buf, 0, bytemuck::bytes_of(cam_uniform));

        ensure_and_upload(
            device,
            queue,
            &mut self.opaque_vbuf,
            &mut self.opaque_cap,
            &scene.opaque_tris,
            "beam3d_opaque_vb",
        );
        ensure_and_upload(
            device,
            queue,
            &mut self.trans_vbuf,
            &mut self.trans_cap,
            &scene.transparent_tris,
            "beam3d_trans_vb",
        );
        ensure_and_upload(
            device,
            queue,
            &mut self.line_vbuf,
            &mut self.line_cap,
            &scene.lines,
            "beam3d_line_vb",
        );

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("beam3d_viewport_encoder"),
        });

        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("beam3d_viewport_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.msaa_view,
                    resolve_target: Some(&self.resolve_view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: bg_color[0],
                            g: bg_color[1],
                            b: bg_color[2],
                            a: bg_color[3],
                        }),
                        store: wgpu::StoreOp::Discard,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });

            rpass.set_bind_group(0, &self.camera_bind_group, &[]);

            if !scene.opaque_tris.is_empty() {
                rpass.set_pipeline(&self.opaque_mesh_pipeline);
                rpass.set_vertex_buffer(0, self.opaque_vbuf.slice(..));
                rpass.draw(0..scene.opaque_tris.len() as u32, 0..1);
            }

            if !scene.lines.is_empty() {
                rpass.set_pipeline(&self.line_pipeline);
                rpass.set_vertex_buffer(0, self.line_vbuf.slice(..));
                rpass.draw(0..scene.lines.len() as u32, 0..1);
            }

            if !scene.transparent_tris.is_empty() {
                rpass.set_pipeline(&self.transparent_mesh_pipeline);
                rpass.set_vertex_buffer(0, self.trans_vbuf.slice(..));
                rpass.draw(0..scene.transparent_tris.len() as u32, 0..1);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
    }
}

fn create_attachments(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::TextureView, wgpu::TextureView, wgpu::TextureView) {
    let size = wgpu::Extent3d {
        width,
        height,
        depth_or_array_layers: 1,
    };

    let msaa_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("beam3d_msaa_color"),
        size,
        mip_level_count: 1,
        sample_count: MSAA_SAMPLES,
        dimension: wgpu::TextureDimension::D2,
        format: COLOR_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });

    let depth_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("beam3d_msaa_depth"),
        size,
        mip_level_count: 1,
        sample_count: MSAA_SAMPLES,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });

    let resolve_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("beam3d_resolve_color"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: COLOR_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });

    (
        msaa_tex.create_view(&wgpu::TextureViewDescriptor::default()),
        depth_tex.create_view(&wgpu::TextureViewDescriptor::default()),
        resolve_tex.create_view(&wgpu::TextureViewDescriptor::default()),
    )
}

fn create_vertex_buf<T>(device: &wgpu::Device, label: &str, cap: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: (cap.max(16) * std::mem::size_of::<T>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

fn ensure_and_upload<T: Pod>(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buf: &mut wgpu::Buffer,
    cap: &mut usize,
    data: &[T],
    label: &str,
) {
    if data.is_empty() {
        return;
    }
    if data.len() > *cap {
        let new_cap = data.len().next_power_of_two();
        *buf = create_vertex_buf::<T>(device, label, new_cap);
        *cap = new_cap;
    }
    queue.write_buffer(buf, 0, bytemuck::cast_slice(data));
}
