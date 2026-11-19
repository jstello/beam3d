use crate::section::{CrossSection, SectionProfile};
use serde::{Deserialize, Serialize};

/// 6-DOF restraint flags `[ux, uy, uz, rx, ry, rz]`.
pub type Restraints = [bool; 6];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SupportPreset {
    Free,
    Pinned,
    Fixed,
    RollerZ,
    Custom,
}

impl SupportPreset {
    pub fn from_restraints(r: &Restraints) -> Self {
        match *r {
            [false, false, false, false, false, false] => SupportPreset::Free,
            [true, true, true, false, false, false] => SupportPreset::Pinned,
            [true, true, true, true, true, true] => SupportPreset::Fixed,
            [false, false, true, false, false, false] => SupportPreset::RollerZ,
            _ => SupportPreset::Custom,
        }
    }

    pub fn to_restraints(self) -> Option<Restraints> {
        match self {
            SupportPreset::Free => Some([false; 6]),
            SupportPreset::Pinned => Some([true, true, true, false, false, false]),
            SupportPreset::Fixed => Some([true; 6]),
            SupportPreset::RollerZ => Some([false, false, true, false, false, false]),
            SupportPreset::Custom => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            SupportPreset::Free => "Free",
            SupportPreset::Pinned => "Pinned (ux, uy, uz)",
            SupportPreset::Fixed => "Fixed (All 6 DOFs)",
            SupportPreset::RollerZ => "Vertical Roller (uz)",
            SupportPreset::Custom => "Custom",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Node {
    pub id: usize,
    pub label: String,
    /// Global coordinates `[X, Y, Z]` in meters (`+Z` is vertical up).
    pub position: [f64; 3],
    /// Restrained DOFs `[ux, uy, uz, rx, ry, rz]`.
    pub restraints: Restraints,
    /// Optional prescribed displacements/rotations `[m, m, m, rad, rad, rad]` applied on restrained DOFs.
    #[serde(default)]
    pub prescribed: [f64; 6],
}

impl Node {
    pub fn new(id: usize, position: [f64; 3]) -> Self {
        Self {
            id,
            label: format!("N{id}"),
            position,
            restraints: [false; 6],
            prescribed: [0.0; 6],
        }
    }

    pub fn is_supported(&self) -> bool {
        self.restraints.iter().any(|&b| b)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Material {
    pub id: usize,
    pub name: String,
    /// Young's modulus E (Pa).
    pub e: f64,
    /// Shear modulus G (Pa).
    pub g: f64,
    /// Mass density rho (kg/m³).
    pub density: f64,
    /// Yield strength fy (Pa) for stress utilization check.
    pub yield_strength: f64,
}

impl Material {
    pub fn structural_steel_s355(id: usize) -> Self {
        Self {
            id,
            name: "Structural Steel S355".to_string(),
            e: 210.0e9,
            g: 80.77e9,
            density: 7850.0,
            yield_strength: 355.0e6,
        }
    }

    pub fn aluminum_6061_t6(id: usize) -> Self {
        Self {
            id,
            name: "Aluminum 6061-T6".to_string(),
            e: 68.9e9,
            g: 26.0e9,
            density: 2700.0,
            yield_strength: 276.0e6,
        }
    }

    pub fn concrete_c30(id: usize) -> Self {
        Self {
            id,
            name: "Concrete C30/37".to_string(),
            e: 33.0e9,
            g: 13.75e9,
            density: 2500.0,
            yield_strength: 30.0e6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BeamFormulation {
    Timoshenko,
    EulerBernoulli,
}

impl BeamFormulation {
    pub fn label(self) -> &'static str {
        match self {
            BeamFormulation::Timoshenko => "Timoshenko (Shear-Deformable)",
            BeamFormulation::EulerBernoulli => "Euler-Bernoulli (Slender)",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct BeamElement {
    pub id: usize,
    pub label: String,
    pub node_i: usize,
    pub node_j: usize,
    pub material_id: usize,
    pub section_id: usize,
    /// Cross-section roll angle (degrees) about the local x-axis.
    pub roll_angle_deg: f64,
    pub formulation: BeamFormulation,
    /// Member end-releases `[ux, uy, uz, rx, ry, rz]` at start node i (local coordinates).
    #[serde(default)]
    pub releases_i: [bool; 6],
    /// Member end-releases `[ux, uy, uz, rx, ry, rz]` at end node j (local coordinates).
    #[serde(default)]
    pub releases_j: [bool; 6],
}

impl BeamElement {
    pub fn new(
        id: usize,
        node_i: usize,
        node_j: usize,
        material_id: usize,
        section_id: usize,
    ) -> Self {
        Self {
            id,
            label: format!("B{id}"),
            node_i,
            node_j,
            material_id,
            section_id,
            roll_angle_deg: 0.0,
            formulation: BeamFormulation::Timoshenko,
            releases_i: [false; 6],
            releases_j: [false; 6],
        }
    }

    pub fn has_releases_i(&self) -> bool {
        self.releases_i.iter().any(|&r| r)
    }

    pub fn has_releases_j(&self) -> bool {
        self.releases_j.iter().any(|&r| r)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NodalLoad {
    pub id: usize,
    pub node_id: usize,
    /// Global forces `[Fx, Fy, Fz]` in N.
    pub forces: [f64; 3],
    /// Global moments `[Mx, My, Mz]` in N·m.
    pub moments: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LoadCoordSystem {
    Global,
    Local,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DistributedLoad {
    pub id: usize,
    pub element_id: usize,
    pub coord_system: LoadCoordSystem,
    /// Intensity `[qx, qy, qz]` at start node i in N/m.
    pub q_start: [f64; 3],
    /// Intensity `[qx, qy, qz]` at end node j in N/m.
    pub q_end: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GravitySettings {
    pub enabled: bool,
    /// Acceleration vector `[ax, ay, az]` in m/s² (default `[0.0, 0.0, -9.80665]`).
    pub acceleration: [f64; 3],
}

impl Default for GravitySettings {
    fn default() -> Self {
        Self {
            enabled: false,
            acceleration: [0.0, 0.0, -9.80665],
        }
    }
}

/// Complete 3D frame model ready for serialization, interactive editing, and FEA solving.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StructuralModel {
    pub name: String,
    pub description: String,
    pub nodes: Vec<Node>,
    pub materials: Vec<Material>,
    pub sections: Vec<CrossSection>,
    pub elements: Vec<BeamElement>,
    pub nodal_loads: Vec<NodalLoad>,
    pub distributed_loads: Vec<DistributedLoad>,
    pub gravity: GravitySettings,
}

impl Default for StructuralModel {
    fn default() -> Self {
        Self {
            name: "Untitled 3D Frame".to_string(),
            description: String::new(),
            nodes: Vec::new(),
            materials: vec![Material::structural_steel_s355(1)],
            sections: vec![CrossSection {
                id: 1,
                name: "IPE 300".to_string(),
                profile: SectionProfile::IBeam {
                    depth: 0.300,
                    width: 0.150,
                    web_thickness: 0.0071,
                    flange_thickness: 0.0107,
                },
            }],
            elements: Vec::new(),
            nodal_loads: Vec::new(),
            distributed_loads: Vec::new(),
            gravity: GravitySettings::default(),
        }
    }
}

impl StructuralModel {
    pub fn next_node_id(&self) -> usize {
        self.nodes.iter().map(|n| n.id).max().unwrap_or(0) + 1
    }

    pub fn next_material_id(&self) -> usize {
        self.materials.iter().map(|m| m.id).max().unwrap_or(0) + 1
    }

    pub fn next_section_id(&self) -> usize {
        self.sections.iter().map(|s| s.id).max().unwrap_or(0) + 1
    }

    pub fn next_element_id(&self) -> usize {
        self.elements.iter().map(|e| e.id).max().unwrap_or(0) + 1
    }

    pub fn next_nodal_load_id(&self) -> usize {
        self.nodal_loads.iter().map(|l| l.id).max().unwrap_or(0) + 1
    }

    pub fn next_distributed_load_id(&self) -> usize {
        self.distributed_loads
            .iter()
            .map(|l| l.id)
            .max()
            .unwrap_or(0)
            + 1
    }

    pub fn get_node(&self, id: usize) -> Option<&Node> {
        self.nodes.iter().find(|n| n.id == id)
    }

    pub fn get_node_mut(&mut self, id: usize) -> Option<&mut Node> {
        self.nodes.iter_mut().find(|n| n.id == id)
    }

    pub fn get_material(&self, id: usize) -> Option<&Material> {
        self.materials.iter().find(|m| m.id == id)
    }

    pub fn get_section(&self, id: usize) -> Option<&CrossSection> {
        self.sections.iter().find(|s| s.id == id)
    }

    pub fn get_element(&self, id: usize) -> Option<&BeamElement> {
        self.elements.iter().find(|e| e.id == id)
    }

    pub fn get_element_mut(&mut self, id: usize) -> Option<&mut BeamElement> {
        self.elements.iter_mut().find(|e| e.id == id)
    }

    pub fn add_node(&mut self, position: [f64; 3]) -> usize {
        let id = self.next_node_id();
        self.nodes.push(Node::new(id, position));
        id
    }

    /// Finds an existing node within `tol` meters or creates a new one.
    pub fn find_or_add_node(&mut self, position: [f64; 3], tol: f64) -> usize {
        let tol2 = tol * tol;
        for n in &self.nodes {
            let dx = n.position[0] - position[0];
            let dy = n.position[1] - position[1];
            let dz = n.position[2] - position[2];
            if dx * dx + dy * dy + dz * dz <= tol2 {
                return n.id;
            }
        }
        self.add_node(position)
    }

    pub fn add_beam(
        &mut self,
        node_i: usize,
        node_j: usize,
        material_id: usize,
        section_id: usize,
    ) -> Option<usize> {
        if node_i == node_j {
            return None;
        }
        if self.get_node(node_i).is_none() || self.get_node(node_j).is_none() {
            return None;
        }
        let id = self.next_element_id();
        self.elements
            .push(BeamElement::new(id, node_i, node_j, material_id, section_id));
        Some(id)
    }

    pub fn remove_node(&mut self, node_id: usize) {
        self.nodes.retain(|n| n.id != node_id);
        let removed_elements: Vec<usize> = self
            .elements
            .iter()
            .filter(|e| e.node_i == node_id || e.node_j == node_id)
            .map(|e| e.id)
            .collect();
        self.elements
            .retain(|e| e.node_i != node_id && e.node_j != node_id);
        self.nodal_loads.retain(|l| l.node_id != node_id);
        self.distributed_loads
            .retain(|l| !removed_elements.contains(&l.element_id));
    }

    pub fn remove_element(&mut self, element_id: usize) {
        self.elements.retain(|e| e.id != element_id);
        self.distributed_loads
            .retain(|l| l.element_id != element_id);
    }

    /// Subdivides a beam element into `n_segments` collinear finite elements,
    /// preserving outer end releases and interpolating any distributed loads.
    pub fn subdivide_element(&mut self, element_id: usize, n_segments: usize) -> Vec<usize> {
        if n_segments < 2 {
            return vec![element_id];
        }
        let Some(orig) = self.get_element(element_id).cloned() else {
            return Vec::new();
        };
        let (Some(ni), Some(nj)) = (
            self.get_node(orig.node_i).cloned(),
            self.get_node(orig.node_j).cloned(),
        ) else {
            return Vec::new();
        };

        let orig_dloads: Vec<DistributedLoad> = self
            .distributed_loads
            .iter()
            .filter(|d| d.element_id == element_id)
            .cloned()
            .collect();

        self.remove_element(element_id);

        let mut chain_nodes = Vec::with_capacity(n_segments + 1);
        chain_nodes.push(ni.id);
        for k in 1..n_segments {
            let t = k as f64 / n_segments as f64;
            let pos = [
                ni.position[0] + t * (nj.position[0] - ni.position[0]),
                ni.position[1] + t * (nj.position[1] - ni.position[1]),
                ni.position[2] + t * (nj.position[2] - ni.position[2]),
            ];
            chain_nodes.push(self.add_node(pos));
        }
        chain_nodes.push(nj.id);

        let mut new_elem_ids = Vec::with_capacity(n_segments);
        for k in 0..n_segments {
            let eid = self.next_element_id();
            let mut elem = BeamElement::new(
                eid,
                chain_nodes[k],
                chain_nodes[k + 1],
                orig.material_id,
                orig.section_id,
            );
            elem.roll_angle_deg = orig.roll_angle_deg;
            elem.formulation = orig.formulation;
            if k == 0 {
                elem.releases_i = orig.releases_i;
            }
            if k + 1 == n_segments {
                elem.releases_j = orig.releases_j;
            }
            self.elements.push(elem);
            new_elem_ids.push(eid);

            let t0 = k as f64 / n_segments as f64;
            let t1 = (k + 1) as f64 / n_segments as f64;
            for dl in &orig_dloads {
                let lid = self.next_distributed_load_id();
                let q0 = [
                    dl.q_start[0] + t0 * (dl.q_end[0] - dl.q_start[0]),
                    dl.q_start[1] + t0 * (dl.q_end[1] - dl.q_start[1]),
                    dl.q_start[2] + t0 * (dl.q_end[2] - dl.q_start[2]),
                ];
                let q1 = [
                    dl.q_start[0] + t1 * (dl.q_end[0] - dl.q_start[0]),
                    dl.q_start[1] + t1 * (dl.q_end[1] - dl.q_start[1]),
                    dl.q_start[2] + t1 * (dl.q_end[2] - dl.q_start[2]),
                ];
                self.distributed_loads.push(DistributedLoad {
                    id: lid,
                    element_id: eid,
                    coord_system: dl.coord_system,
                    q_start: q0,
                    q_end: q1,
                });
            }
        }

        new_elem_ids
    }

    /// Returns `(min_xyz, max_xyz)` bounding box of all nodes in meters.
    pub fn bounding_box(&self) -> ([f64; 3], [f64; 3]) {
        if self.nodes.is_empty() {
            return ([-5.0, -5.0, 0.0], [5.0, 5.0, 5.0]);
        }
        let mut min_p = [f64::INFINITY; 3];
        let mut max_p = [f64::NEG_INFINITY; 3];
        for n in &self.nodes {
            for k in 0..3 {
                min_p[k] = min_p[k].min(n.position[k]);
                max_p[k] = max_p[k].max(n.position[k]);
            }
        }
        (min_p, max_p)
    }

    pub fn characteristic_length(&self) -> f64 {
        let (min_p, max_p) = self.bounding_box();
        let dx = max_p[0] - min_p[0];
        let dy = max_p[1] - min_p[1];
        let dz = max_p[2] - min_p[2];
        (dx * dx + dy * dy + dz * dz).sqrt().max(1.0)
    }

    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}
