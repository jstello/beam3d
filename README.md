# Beam3D — 3D Structural Finite Element Solver & Native Rust UI

A pure-Rust 3D structural finite element solver (beams + 2D shells, linear & nonlinear large deformations, modal & transient dynamics) and native hardware-accelerated desktop application built with `egui` + `wgpu`.

## Features & Workspace Structure

- **`crates/beam3d-core`**: Standalone 3D FEA library (written from scratch in Rust)
  - **3D Space-Frame Beams (`element.rs`):** 2-node, 12-DOF **Timoshenko** (shear-deformable) and **Euler–Bernoulli** beam elements, arbitrary cross-section roll angle $\gamma$, and exact element-level **static condensation** for member end-releases (hinges).
  - **2D Shell / Surface Elements (`shell.rs`):** Flat 3D **Mindlin–Reissner + Plane-Stress** shells — 4-node quads (`Quad4`) with 2×2 Gauss bending/membrane integration + 1×1 Selective Reduced Integration (SRI) to prevent thin-plate shear locking, and 3-node triangles (`Tri3`) with invariant drilling-DOF stabilization. Fully coupled with 3D beams in the same 6-DOF/node global stiffness matrix.
  - **Large Deformations / Geometric Nonlinearity (`nonlinear.rs`):** Incremental Newton–Raphson corotational 3D beam & EICR shell solver with adaptive step halving, $SO(3)$ rotation updates, P-Delta amplification, membrane stress stiffening, and deformed-frame internal force recovery.
  - **Structural Dynamics (`dynamics.rs`):** Consistent & lumped mass matrices (plus nodal lumped `added_mass`), **Modal Analysis** (natural frequencies, periods, and mass-normalized 3D mode shapes via dense & subspace eigensolvers), and **Linear & Nonlinear Transient Time-History** integration (implicit Newmark-$\beta$ / HHT-$\alpha$, Rayleigh damping, and `Step`, `Ramp`, `Impulse`, `HalfSinePulse`, `Harmonic`, `BaseSine` seismic, and `FreeVibration` excitations).
  - **Cross-Section Library (`section.rs`):** Parametric **I/H Wide-Flange**, **Hollow Rectangular Box (HSS)**, **Circular Pipe/Tube**, **Solid Rectangle**, **Solid Circle**, and **Custom** section properties ($A, I_y, I_z, J, A_{sy}, A_{sz}$).
  - **Loads & Boundary Conditions:** Nodal forces & moments, prescribed support displacements/rotations, self-weight gravity, uniform/trapezoidal 1D distributed beam loads (3-point Gauss–Legendre exact integration), and 2D distributed surface pressures (`Global` or `Local Normal`) on shells.
  - **Sparse Linear Algebra (`solver.rs`):** Global stiffness assembled in CSC format and solved via **`faer`** supernodal sparse Cholesky ($LL^T$) with sparse $LU$ fallback, residual verification, reaction recovery, and exact interior station force/deflection/stress recovery. Local element transformations and condensation use **`nalgebra`**.
  - **Analytical Verification Suite (`tests.rs`):** 21 automated core tests verifying Euler–Bernoulli & Timoshenko cantilevers, propped-cantilever hinges, Castigliano 3D coupled bending/torsion, `Quad4`/`Tri3` membrane patches, Navier's simply supported plate, 90° circular elastica, P-Delta column amplification, shell stress stiffening, analytical cantilever/plate modal frequencies, DAF = 2.0 step-load energy conservation, harmonic resonance, and nonlinear dynamics.
- **`crates/beam3d-ui`**: Native Desktop CAD/CAE Interface (`eframe` + custom 4x MSAA `wgpu` 3D pipeline)
  - Extruded 3D beam profiles with $SO(3)$ frame interpolation (undistorted through 90°+ large rotations) alongside smooth 3D deformed shell surfaces.
  - Switchable **`Linear Static`**, **`Nonlinear Static (Large Deform.)`**, **`Modal`** (mode selector + harmonic animation), and **`Dynamic (Time History)`** (interactive playback scrubber + time-history & load-deflection `egui_plot` charts).
  - Scientific contour colormaps (Displacement, Von Mises Stress, Normal Stress, Yield Utilization %, $M_y, M_z, V_z, V_y, N, T$) and 3D shaded internal force/moment ribbons.
  - Interactive 3D sketching across switchable **`XY` / `XZ` / `YZ`** working planes (`Select`, `Add Node`, `Draw Beam`, `Draw Shell` with auto-meshing `Nx × Ny`, and one-click bulk 2D surface loads).
  - 8 built-in templates (including *Flexible Cantilever 90° Elastica*, *3D Concrete Floor Slab on Steel Beams*, *2D Simply Supported Square Plate*, *3D Braced Multi-Story Steel Frame*, and *3D Tied-Arch Pedestrian Bridge*), Undo/Redo, live 2D cross-section preview, 2D `egui_plot` member diagrams, reaction/displacement tables, interactive in-app tutorials, and JSON save/open.

## Quick Start

```bash
# Run analytical verification tests (beams + 2D shells + GPU render check)
cargo test --workspace

# Launch the native 3D desktop application (optimized release build recommended)
cargo run --release -p beam3d-ui
```

## Tutorials & How to Draw in 3D

- **In-App Interactive Coach:** Click **`🎓 Tutorials`** in the top toolbar to launch step-by-step live-checked tutorials.
- **Written Guide:** See [docs/TUTORIALS.md](docs/TUTORIALS.md) for a walkthrough of 3D workplane drawing (`XY` / `XZ` / `YZ`), magnetic node snapping, column extrusion, supports, loads, hinges, and post-processing.

## Viewport Controls

| Action | Control |
|---|---|
| **Orbit** | Drag (in Select tool) or Right-drag |
| **Pan** | Middle-drag or `Shift` + Drag |
| **Zoom** | Scroll wheel / two-finger trackpad scroll |
| **Fit to Model** | `F` key or `⛶ Fit [F]` button |
| **Cycle Working Plane (`XY` → `XZ` → `YZ`)** | `P` key or `Plane: XY / XZ / YZ` buttons |
| **Step Plane Offset (`Z`, `Y`, or `X`)** | `]` / `[` keys (or top-right 3D Drawing HUD) |
| **Finish 3-Point Triangle Shell** | `Enter` (in Draw Shell tool after 3 corners) |
| **Delete Selected** | `Delete` / `Backspace` |
| **Undo / Redo** | `Cmd+Z` / `Cmd+Shift+Z` |
| **End Beam Chain / Cancel** | `Esc` or Right-click |

## Feedback & Issues

Have a bug report, verification discrepancy, or feature/benchmark idea? Please open a [GitHub Issue](https://github.com/jstello/beam3d/issues) — issues are the main place for feedback and roadmap ideas.

## License

MIT — see [LICENSE](LICENSE).
