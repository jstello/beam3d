pub mod dynamics;
pub mod element;
pub mod model;
pub mod nonlinear;
pub mod section;
pub mod shell;
pub mod solver;
pub mod templates;

#[cfg(test)]
mod tests;

pub use dynamics::{
    DynamicConfig, DynamicFrame, DynamicHistoryPoint, DynamicSolution, MassFormulation,
    ModalConfig, ModalSolution, ModeShape, RayleighDamping, TimeHistoryExcitation, solve_dynamic,
    solve_dynamic_with_config, solve_modal, solve_modal_with_config,
};
pub use element::{ElementFrame, StationSample};
pub use model::{
    BeamElement, BeamFormulation, DistributedLoad, GravitySettings, LoadCoordSystem, Material,
    NodalLoad, Node, Restraints, ShellElement, ShellNodes, StructuralModel, SupportPreset,
    SurfaceLoad,
};
pub use nonlinear::{
    LoadStepRecord, NonlinearSolution, NonlinearStaticConfig, solve_nonlinear_static,
    solve_nonlinear_static_with_config,
};
pub use section::{CrossSection, ProfileEdge2D, SectionProfile, SectionProperties};
pub use shell::{ShellFrame, ShellResult};
pub use solver::{
    AnalysisSummary, ElementResult, NodeResult, Solution, SolverConfig, SolverError,
    solve_linear_static, solve_linear_static_with_config,
};
pub use templates::TemplateKind;

pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}
