# Beam3D — 3D Space-Frame Finite Element Solver & Interactive UI

A pure-Rust 3D structural beam/space-frame finite element solver and native hardware-accelerated desktop application (`egui` + `wgpu`).

## Workspace Structure

- **`crates/beam3d-core`**: Standalone 3D linear static FEA library
  - **2-Node, 12-DOF Space Frame Elements**: Both **Timoshenko** (shear-deformable) and **Euler–Bernoulli** formulations, arbitrary cross-section roll angle $\gamma$, and local coordinate transformation.
  - **Member End Releases (Hinges)**: Exact element-level static condensation for moment/torsion/axial releases (`[ux, uy, uz, rx, ry, rz]`) at either or both member ends, including coupled condensation of distributed fixed-end forces and post-solve hinge rotation recovery.
  - **Cross-Section Library**: Parametric **I/H Wide-Flange**, **Hollow Rectangular Box (HSS)**, **Circular Pipe/Tube**, **Solid Rectangle**, **Solid Circle**, plus **Custom** section properties ($A, I_y, I_z, J, A_{sy}, A_{sz}$).
  - **Loading & Boundary Conditions**: Nodal forces & moments, prescribed support displacements/rotations, self-weight gravity, and uniform or trapezoidal linearly varying distributed loads (in Global or Local coordinates) integrated with exact Timoshenko shape functions.
  - **Sparse Direct Solver**: Assembled in compressed sparse column format and solved via `faer` supernodal sparse $LL^T$ Cholesky (with sparse $LU$ fallback), residual verification, support reaction recovery, and exact interior station recovery ($N, V_y, V_z, T, M_y, M_z$, deflections, combined normal & von Mises stress, and yield utilization).
- **`crates/beam3d-ui`**: Native Desktop CAD/CAE Interface (`eframe` + custom 4x MSAA `wgpu` 3D pipeline)
  - Extruded 3D beam profiles that bend and twist along the solved deformed shape.
  - Contour colormaps (Displacement, Von Mises Stress, Normal Stress, Yield Utilization %, $M_y, M_z, V_z, V_y, N, T$) and 3D shaded internal force/moment ribbons.
  - Interactive viewport picking, click-to-place nodes & chain beams on a snapped workplane, Undo/Redo (`Cmd+Z` / `Cmd+Shift+Z`), live 2D cross-section preview, 2D selected-member force/deflection plots, reaction/displacement tables, built-in structural templates, and JSON project import/export.

## Quick Start

```bash
# Run analytical verification unit tests
cargo test --workspace

# Launch the native 3D desktop application (optimized release build recommended)
cargo run --release -p beam3d-ui
```

## Viewport Controls

| Action | Control |
|---|---|
| **Orbit** | Drag (in Select tool) or Right-drag |
| **Pan** | Middle-drag or `Shift` + Drag |
| **Zoom** | Scroll wheel / two-finger trackpad scroll |
| **Fit to Model** | `F` key or `⛶ Fit [F]` button |
| **Delete Selected** | `Delete` / `Backspace` |
| **Undo / Redo** | `Cmd+Z` / `Cmd+Shift+Z` |
| **End Beam Chain** | `Esc` or Right-click |
