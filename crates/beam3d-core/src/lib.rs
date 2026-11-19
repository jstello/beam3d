pub mod element;
pub mod model;
pub mod section;
pub mod solver;
pub mod templates;

#[cfg(test)]
mod tests;

pub use element::{ElementFrame, StationSample};
pub use model::{
    BeamElement, BeamFormulation, DistributedLoad, GravitySettings, LoadCoordSystem, Material,
    NodalLoad, Node, Restraints, StructuralModel, SupportPreset,
};
pub use section::{CrossSection, ProfileEdge2D, SectionProfile, SectionProperties};
pub use solver::{
    AnalysisSummary, ElementResult, NodeResult, Solution, SolverConfig, SolverError,
    solve_linear_static, solve_linear_static_with_config,
};
pub use templates::TemplateKind;

pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}
