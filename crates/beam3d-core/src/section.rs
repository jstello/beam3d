use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Parametric and custom 3D beam cross-section profiles.
/// Local cross-section axes: `y` = width (horizontal for standard girders), `z` = depth (vertical).
/// Strong-axis bending occurs in the local `x-z` plane about the local `y` axis (`iy`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum SectionProfile {
    IBeam {
        depth: f64,
        width: f64,
        web_thickness: f64,
        flange_thickness: f64,
    },
    HollowBox {
        depth: f64,
        width: f64,
        thickness: f64,
    },
    Pipe {
        outer_diameter: f64,
        thickness: f64,
    },
    SolidRect {
        depth: f64,
        width: f64,
    },
    SolidCircle {
        diameter: f64,
    },
    Custom {
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
        asy: f64,
        asz: f64,
        depth: f64,
        width: f64,
    },
}

/// Computed geometric & stiffness properties of a cross-section (SI units: m, m², m⁴).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SectionProperties {
    pub area: f64,
    /// Second moment of area about local y-axis (bending in x-z plane).
    pub iy: f64,
    /// Second moment of area about local z-axis (bending in x-y plane).
    pub iz: f64,
    /// St. Venant torsional constant.
    pub j: f64,
    /// Effective Timoshenko shear area along local y-axis.
    pub asy: f64,
    /// Effective Timoshenko shear area along local z-axis.
    pub asz: f64,
    /// Extreme fiber half-width along local y-axis.
    pub half_width_y: f64,
    /// Extreme fiber half-depth along local z-axis.
    pub half_depth_z: f64,
}

/// A 2D edge segment in `(y, z)` cross-section space used to extrude 3D beam surfaces.
#[derive(Debug, Clone, Copy)]
pub struct ProfileEdge2D {
    pub p0: [f32; 2],
    pub p1: [f32; 2],
    pub n0: [f32; 2],
    pub n1: [f32; 2],
    /// Whether to draw a crisp feature line along p0 across the beam length.
    pub sharp_edge: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CrossSection {
    pub id: usize,
    pub name: String,
    pub profile: SectionProfile,
}

impl CrossSection {
    pub fn properties(&self) -> SectionProperties {
        self.profile.properties()
    }
}

impl SectionProfile {
    pub fn summary_label(&self) -> String {
        match *self {
            SectionProfile::IBeam {
                depth,
                width,
                web_thickness,
                flange_thickness,
            } => format!(
                "I {:.0}x{:.0}x{:.1}/{:.1} mm",
                depth * 1e3,
                width * 1e3,
                web_thickness * 1e3,
                flange_thickness * 1e3
            ),
            SectionProfile::HollowBox {
                depth,
                width,
                thickness,
            } => format!(
                "HSS {:.0}x{:.0}x{:.1} mm",
                depth * 1e3,
                width * 1e3,
                thickness * 1e3
            ),
            SectionProfile::Pipe {
                outer_diameter,
                thickness,
            } => format!("Pipe Ø{:.0}x{:.1} mm", outer_diameter * 1e3, thickness * 1e3),
            SectionProfile::SolidRect { depth, width } => {
                format!("Rect {:.0}x{:.0} mm", depth * 1e3, width * 1e3)
            }
            SectionProfile::SolidCircle { diameter } => {
                format!("Rod Ø{:.0} mm", diameter * 1e3)
            }
            SectionProfile::Custom { area, .. } => {
                format!("Custom (A={:.1} cm²)", area * 1e4)
            }
        }
    }

    pub fn properties(&self) -> SectionProperties {
        match *self {
            SectionProfile::IBeam {
                depth,
                width,
                web_thickness,
                flange_thickness,
            } => {
                let d = depth.max(1e-4);
                let b = width.max(1e-4);
                let tf = flange_thickness.clamp(1e-5, 0.48 * d);
                let tw = web_thickness.clamp(1e-5, 0.95 * b);
                let hw = (d - 2.0 * tf).max(1e-6);

                let area = 2.0 * b * tf + hw * tw;
                let iy = (b * d.powi(3) - (b - tw) * hw.powi(3)) / 12.0;
                let iz = (2.0 * tf * b.powi(3) + hw * tw.powi(3)) / 12.0;
                let j = (2.0 * b * tf.powi(3) + (d - tf) * tw.powi(3)) / 3.0;
                let asz = d * tw;
                let asy = (5.0 / 3.0) * b * tf;

                SectionProperties {
                    area,
                    iy,
                    iz,
                    j,
                    asy,
                    asz,
                    half_width_y: 0.5 * b,
                    half_depth_z: 0.5 * d,
                }
            }
            SectionProfile::HollowBox {
                depth,
                width,
                thickness,
            } => {
                let d = depth.max(1e-4);
                let b = width.max(1e-4);
                let t = thickness.clamp(1e-5, 0.48 * d.min(b));
                let bi = (b - 2.0 * t).max(0.0);
                let di = (d - 2.0 * t).max(0.0);

                let area = b * d - bi * di;
                let iy = (b * d.powi(3) - bi * di.powi(3)) / 12.0;
                let iz = (d * b.powi(3) - di * bi.powi(3)) / 12.0;
                // Bredt's closed thin-walled torsion formula
                let am = (b - t) * (d - t);
                let perimeter = 2.0 * ((b - t) + (d - t));
                let j = (4.0 * am * am * t) / perimeter.max(1e-9);
                let asz = 2.0 * (d - t) * t;
                let asy = 2.0 * (b - t) * t;

                SectionProperties {
                    area,
                    iy,
                    iz,
                    j,
                    asy,
                    asz,
                    half_width_y: 0.5 * b,
                    half_depth_z: 0.5 * d,
                }
            }
            SectionProfile::Pipe {
                outer_diameter,
                thickness,
            } => {
                let do_ = outer_diameter.max(1e-4);
                let ro = 0.5 * do_;
                let t = thickness.clamp(1e-5, 0.98 * ro);
                let ri = (ro - t).max(0.0);

                let area = PI * (ro * ro - ri * ri);
                let iy = 0.25 * PI * (ro.powi(4) - ri.powi(4));
                let iz = iy;
                let j = 2.0 * iy;
                // Cowper's shear factor for hollow circular section (nu = 0.3)
                let m = ri / ro;
                let m2 = m * m;
                let nu = 0.3;
                let kappa = (6.0 * (1.0 + nu) * (1.0 + m2).powi(2))
                    / ((7.0 + 6.0 * nu) * (1.0 + m2).powi(2) + (20.0 + 12.0 * nu) * m2);
                let as_ = kappa * area;

                SectionProperties {
                    area,
                    iy,
                    iz,
                    j,
                    asy: as_,
                    asz: as_,
                    half_width_y: ro,
                    half_depth_z: ro,
                }
            }
            SectionProfile::SolidRect { depth, width } => {
                let d = depth.max(1e-4);
                let b = width.max(1e-4);
                let area = b * d;
                let iy = b * d.powi(3) / 12.0;
                let iz = d * b.powi(3) / 12.0;
                // Roark's rectangular St. Venant torsion constant
                let a = 0.5 * d.max(b);
                let c = 0.5 * d.min(b);
                let ratio = c / a;
                let j = a * c.powi(3) * ((16.0 / 3.0) - 3.36 * ratio * (1.0 - ratio.powi(4) / 12.0));
                let as_ = (5.0 / 6.0) * area;

                SectionProperties {
                    area,
                    iy,
                    iz,
                    j,
                    asy: as_,
                    asz: as_,
                    half_width_y: 0.5 * b,
                    half_depth_z: 0.5 * d,
                }
            }
            SectionProfile::SolidCircle { diameter } => {
                let d = diameter.max(1e-4);
                let r = 0.5 * d;
                let area = PI * r * r;
                let iy = 0.25 * PI * r.powi(4);
                let iz = iy;
                let j = 2.0 * iy;
                let as_ = 0.9 * area;

                SectionProperties {
                    area,
                    iy,
                    iz,
                    j,
                    asy: as_,
                    asz: as_,
                    half_width_y: r,
                    half_depth_z: r,
                }
            }
            SectionProfile::Custom {
                area,
                iy,
                iz,
                j,
                asy,
                asz,
                depth,
                width,
            } => SectionProperties {
                area: area.max(1e-8),
                iy: iy.max(1e-12),
                iz: iz.max(1e-12),
                j: j.max(1e-12),
                asy: asy.max(1e-8),
                asz: asz.max(1e-8),
                half_width_y: 0.5 * width.max(1e-3),
                half_depth_z: 0.5 * depth.max(1e-3),
            },
        }
    }

    /// Generates 2D perimeter edges (for side-wall extrusion) and end-cap triangles in `(y, z)`.
    pub fn extrusion_geometry(&self) -> (Vec<ProfileEdge2D>, Vec<[[f32; 2]; 3]>) {
        match *self {
            SectionProfile::IBeam {
                depth,
                width,
                web_thickness,
                flange_thickness,
            } => {
                let hz = 0.5 * depth.max(1e-3) as f32;
                let hy = 0.5 * width.max(1e-3) as f32;
                let tf = (flange_thickness as f32).clamp(0.05 * hz, 0.45 * 2.0 * hz);
                let tw = 0.5 * (web_thickness as f32).clamp(0.05 * hy, 0.9 * 2.0 * hy);
                let iz = hz - tf;

                // 12-vertex CCW outer polygon in (y, z)
                let pts: [[f32; 2]; 12] = [
                    [-hy, -hz],
                    [hy, -hz],
                    [hy, -iz],
                    [tw, -iz],
                    [tw, iz],
                    [hy, iz],
                    [hy, hz],
                    [-hy, hz],
                    [-hy, iz],
                    [-tw, iz],
                    [-tw, -iz],
                    [-hy, -iz],
                ];
                let edges = polygon_to_sharp_edges(&pts);
                // 3 rectangles = 6 triangles for end caps
                let caps = vec![
                    // Bottom flange
                    [[-hy, -hz], [hy, -hz], [hy, -iz]],
                    [[-hy, -hz], [hy, -iz], [-hy, -iz]],
                    // Web
                    [[-tw, -iz], [tw, -iz], [tw, iz]],
                    [[-tw, -iz], [tw, iz], [-tw, iz]],
                    // Top flange
                    [[-hy, iz], [hy, iz], [hy, hz]],
                    [[-hy, iz], [hy, hz], [-hy, hz]],
                ];
                (edges, caps)
            }
            SectionProfile::HollowBox {
                depth,
                width,
                thickness,
            } => {
                let hz = 0.5 * depth.max(1e-3) as f32;
                let hy = 0.5 * width.max(1e-3) as f32;
                let t = (thickness as f32).clamp(0.04 * hz.min(hy), 0.45 * 2.0 * hz.min(hy));
                let iy = hy - t;
                let iz = hz - t;

                let outer = [[-hy, -hz], [hy, -hz], [hy, hz], [-hy, hz]];
                // CW inner loop so right-hand normal points inward into the cavity
                let inner = [[-iy, -iz], [-iy, iz], [iy, iz], [iy, -iz]];
                let mut edges = polygon_to_sharp_edges(&outer);
                edges.extend(polygon_to_sharp_edges(&inner));

                let caps = vec![
                    // Bottom strip
                    [[-hy, -hz], [hy, -hz], [iy, -iz]],
                    [[-hy, -hz], [iy, -iz], [-iy, -iz]],
                    // Right strip
                    [[hy, -hz], [hy, hz], [iy, iz]],
                    [[hy, -hz], [iy, iz], [iy, -iz]],
                    // Top strip
                    [[hy, hz], [-hy, hz], [-iy, iz]],
                    [[hy, hz], [-iy, iz], [iy, iz]],
                    // Left strip
                    [[-hy, hz], [-hy, -hz], [-iy, -iz]],
                    [[-hy, hz], [-iy, -iz], [-iy, iz]],
                ];
                (edges, caps)
            }
            SectionProfile::Pipe {
                outer_diameter,
                thickness,
            } => {
                let ro = 0.5 * outer_diameter.max(1e-3) as f32;
                let t = (thickness as f32).clamp(0.05 * ro, 0.9 * ro);
                let ri = ro - t;
                let n = 20usize;
                let mut edges = Vec::with_capacity(2 * n);
                let mut caps = Vec::with_capacity(2 * n);
                for k in 0..n {
                    let a0 = (k as f32 / n as f32) * std::f32::consts::TAU;
                    let a1 = ((k + 1) as f32 / n as f32) * std::f32::consts::TAU;
                    let (c0, s0) = (a0.cos(), a0.sin());
                    let (c1, s1) = (a1.cos(), a1.sin());
                    let o0 = [ro * c0, ro * s0];
                    let o1 = [ro * c1, ro * s1];
                    let i0 = [ri * c0, ri * s0];
                    let i1 = [ri * c1, ri * s1];
                    edges.push(ProfileEdge2D {
                        p0: o0,
                        p1: o1,
                        n0: [c0, s0],
                        n1: [c1, s1],
                        sharp_edge: k % 5 == 0,
                    });
                    edges.push(ProfileEdge2D {
                        p0: i1,
                        p1: i0,
                        n0: [-c1, -s1],
                        n1: [-c0, -s0],
                        sharp_edge: false,
                    });
                    caps.push([i0, o0, o1]);
                    caps.push([i0, o1, i1]);
                }
                (edges, caps)
            }
            SectionProfile::SolidCircle { diameter } => {
                let r = 0.5 * diameter.max(1e-3) as f32;
                let n = 20usize;
                let mut edges = Vec::with_capacity(n);
                let mut caps = Vec::with_capacity(n);
                for k in 0..n {
                    let a0 = (k as f32 / n as f32) * std::f32::consts::TAU;
                    let a1 = ((k + 1) as f32 / n as f32) * std::f32::consts::TAU;
                    let (c0, s0) = (a0.cos(), a0.sin());
                    let (c1, s1) = (a1.cos(), a1.sin());
                    let p0 = [r * c0, r * s0];
                    let p1 = [r * c1, r * s1];
                    edges.push(ProfileEdge2D {
                        p0,
                        p1,
                        n0: [c0, s0],
                        n1: [c1, s1],
                        sharp_edge: k % 5 == 0,
                    });
                    caps.push([[0.0, 0.0], p0, p1]);
                }
                (edges, caps)
            }
            SectionProfile::SolidRect { depth, width }
            | SectionProfile::Custom { depth, width, .. } => {
                let hz = 0.5 * depth.max(1e-3) as f32;
                let hy = 0.5 * width.max(1e-3) as f32;
                let pts = [[-hy, -hz], [hy, -hz], [hy, hz], [-hy, hz]];
                let edges = polygon_to_sharp_edges(&pts);
                let caps = vec![
                    [[-hy, -hz], [hy, -hz], [hy, hz]],
                    [[-hy, -hz], [hy, hz], [-hy, hz]],
                ];
                (edges, caps)
            }
        }
    }
}

fn polygon_to_sharp_edges(pts: &[[f32; 2]]) -> Vec<ProfileEdge2D> {
    let n = pts.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let p0 = pts[i];
        let p1 = pts[(i + 1) % n];
        let dy = p1[0] - p0[0];
        let dz = p1[1] - p0[1];
        let len = (dy * dy + dz * dz).sqrt().max(1e-8);
        // For CCW loop in (y, z), outward normal is (+dz, -dy) / len
        let normal = [dz / len, -dy / len];
        out.push(ProfileEdge2D {
            p0,
            p1,
            n0: normal,
            n1: normal,
            sharp_edge: true,
        });
    }
    out
}
