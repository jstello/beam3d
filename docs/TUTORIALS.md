# Beam3D Tutorials — Basic Operations & Drawing in 3D

> **Interactive In-App Coach:** You can also follow interactive, live-checked tutorials directly inside the desktop app! Click **`🎓 Tutorials`** in the top toolbar to launch any of the 4 guided tracks.

---

## Coordinate System & Units

Beam3D uses a right-handed **Z-up** structural engineering coordinate system:

- **Ground Plane (`X–Y`):** Horizontal floor/plan axes (`+X` right, `+Y` into screen in Front view).
- **Vertical Axis (`+Z`):** Elevation pointing straight up (`Z = 0 m` is default ground level; gravity acts along `-Z`).
- **Member Local Axes (`x, y, z`):**
  - Local `x` runs from `Node i` → `Node j`.
  - Local `z` points upward (web depth direction `h`) for horizontal girders when `Roll γ = 0°`, so **strong-axis bending (`My`, inertia `Iy`)** resists vertical gravity loads.
  - Local `y` is the flange width direction `b` (**weak-axis bending `Mz`, inertia `Iz`**).

| Quantity | UI Display Unit | JSON Storage Unit (SI) |
|---|---|---|
| Coordinates / Length | `m` | `m` |
| Cross-Section Dimensions | `mm` | `m` |
| Forces / Distributed Loads | `kN`, `kN/m` | `N`, `N/m` |
| Moments / Torque | `kN·m` | `N·m` |
| Displacements / Rotations | `mm`, `mrad` | `m`, `rad` |
| Stress / Moduli | `MPa`, `GPa` | `Pa` |

---

## Tutorial 1: How to Draw in 3D (Build a Space Frame from Scratch)

Beam3D supports **all three orthogonal 3D working planes** plus automatic node alignment:
1. **Switchable 3D Working Planes (`XY`, `XZ`, `YZ`):**
   - **`XY` (Floor Plane, normal `Z`):** Slate horizontal grid for floor plans and roof girders.
   - **`XZ` (Front Vertical Plane, normal `Y`):** Upright blue grid for drawing vertical columns, portal frames, and trusses in `X–Z`.
   - **`YZ` (Side Vertical Plane, normal `X`):** Upright green grid for drawing vertical side frames and columns in `Y–Z`.
   - Switch anytime using **`Plane: [XY] [XZ] [YZ]`** in the top bar (always visible!), the top-right 3D Drawing HUD, or by pressing **`P`** on your keyboard!
2. **Smart Start-Node Plane Alignment & 3D Magnetic Snapping:**
   - Hovering near *any* existing node magnetically snaps your cursor to that 3D node (`Snap → Node N#`).
   - Whenever you pick a start node in **`📏 Draw Beam`** (or switch planes while a node is selected), the active plane **automatically slides to pass directly through that node** — so clicking any ground node `(x, y, 0)` in `XZ` or `YZ` mode immediately lets you draw a vertical column straight up to `(x, y, H)`!

```mermaid
flowchart LR
    A["1. Plane = XY (Z=0)\nPlace Ground Nodes"] --> B["2. Press 'P' → Plane = XZ\n(Vertical Grid)"]
    B --> C["3. Draw Beam: Click Ground Node\n→ Click Straight Up at Z = 3m"]
    C --> D["4. Press 'P' → Plane = XY\n(Auto-aligns at Z = 3m!) & Chain Roof"]
    D --> E["5. Add Supports, Loads\n& Click Solve FEA"]
```

### Step-by-Step: Drawing a 4 m × 4 m × 3 m Steel Canopy

1. **Start a Clean Workspace**
   - Click **`📂 Templates...` → `✨ New Empty Model`**.
   - Check **`Labels`** in the Right Panel (`🎨 3D Post-Processor & View`) so `N1`, `N2`, ... are visible in 3D.

2. **Place the 4 Column Base Nodes on the `XY` Floor Plane (`Z = 0 m`)**
   - Click **`⊕ Add Node`** in the top toolbar (with **`Plane: XY`** and **`Z: 0.0m`**).
   - Set **`Snap: 1.0 m`**.
   - Left-click four corners on the floor grid: `(0, 0, 0)`, `(4, 0, 0)`, `(4, 4, 0)`, and `(0, 4, 0)` → creates `N1..N4`.
   - *(Tip: **Right-drag** anytime to orbit the 3D camera without placing a node!)*

3. **Switch to the Vertical `XZ` Plane to Draw Vertical Columns**
   - Click **`📏 Draw Beam`** in the top toolbar.
   - Press **`P`** (or click **`XZ`** next to `Plane:` in the top bar) — an upright blue vertical grid appears!
   - Click ground node **`N1`** `(0, 0, 0)` — the vertical `XZ` grid automatically aligns through `N1` (`Y = 0 m`). Move your cursor straight up along the vertical grid line to `(0, 0, 3.0)` and click → **Vertical Column `B1` (`N1 → N5`) is created!**
   - Press **`Esc`** (or **Right-click**) to end the chain.
   - Click ground node **`N3`** `(4, 4, 0)` — watch the vertical `XZ` grid **automatically slide to `Y = 4.0 m`** right through `N3`! Move straight up to `(4, 4, 3.0)` and click → **Vertical Column `N3 → N7` is created!** Repeat for `N2` and `N4`.

4. **Switch Back to `XY` to Close the Roof Girders**
   - With a column top node active (at `Z = 3.0 m`), press **`P`** until **`Plane: XY`** is active — the horizontal `XY` grid automatically jumps up to **`Z = 3.0 m`** (roof level)!
   - Click around the four column tops `N5 → N6 → N7 → N8 → N5` to chain all 4 roof girders, then press **`Esc`**.

   > **Shortcut — One-Click Column Extrusion:** Instead of raising the workplane manually, you can also select any ground node with **`🖱 Select`**, set **`ΔZ: 3.0 m`** in the Right Panel (`Selection Inspector`), and click **`⬆ Extrude Column (+ΔZ)`** to create the upper node and vertical column in a single click!

   > **Exact Coordinate Entry:** Prefer typing exact numbers? Open the **`🏗 Structure`** tab in the Left Panel, enter `X, Y, Z`, click **`+ Add Node at Coordinates`**, and connect any two nodes via **`+ Connect`**.

---

## Tutorial 2: Assigning Supports & Prescribed Settlements

A 3D frame without supports is a rigid-body mechanism and cannot be solved until restrained.

1. Switch to **`🖱 Select`** (or press `Esc`).
2. Click ground node **`N1`** in the 3D viewport.
3. In the Right Panel (**`Selection Inspector`**), open the **`Support`** dropdown:
   - **`Fixed (All 6 DOFs)`** — restrains translations `ux, uy, uz` and rotations `rx, ry, rz` (renders a 3D teal base block).
   - **`Pinned (ux, uy, uz)`** — restrains translations only, allowing free rotation (renders a 3D green pyramid).
   - **`Vertical Roller (uz)`** — restrains vertical translation `uz` only (renders a 3D blue pyramid on a roller plate).
   - **`Custom`** — toggle individual checkboxes (`ux`, `uy`, `uz`, `rx`, `ry`, `rz`).
4. Repeat for `N2`, `N3`, and `N4`.
5. *(Optional — Support Settlement)* Expand **`Prescribed Settlement / Rotation`** on a supported node and enter e.g. `uz = -0.010 m` (`-10 mm`) to simulate foundation differential settlement!

---

## Tutorial 3: Cross-Sections, Materials & Roll Angle ($\gamma$)

### Editing or Creating Parametric Sections
1. Open the **`📐 Sections & Mat`** tab in the Left Panel.
2. Click **`+ New`** or edit an existing section:
   - Choose from **I / H Wide-Flange**, **Hollow Rectangular (HSS)**, **Circular Pipe / Tube**, **Solid Rectangular**, **Solid Circular Rod**, or **Custom Properties**.
   - Drag dimensions in **`mm`** — the **2D blueprint cross-section preview**, section constants ($A, I_y, I_z, J, A_{sy}, A_{sz}$), and **3D solid extruded profiles** update live.
3. Assign a section to any beam by selecting the beam in 3D and choosing **`Section`** in the Right Panel.

### Rotating a Beam's Cross-Section (`Roll γ`)
- Select any column or girder and drag **`Roll γ (°)`** between `-180°` and `+180°`.
- At `γ = 90°`, an I-beam turns on its side so vertical loads bend its weak axis (`Iz`) instead of its strong web (`Iy`). Enable **`Local Axes`** in the Right Panel to visualize the RGB local `(x, y, z)` triads at member midpoints.

---

## Tutorial 4: Applying Loads & Self-Weight

1. **Self-Weight Gravity:** Open **`⬇ Loads`** in the Left Panel and check **`Include Member Self-Weight`** (automatically computes $\rho A g$ along every member).
2. **Nodal Forces & Moments:**
   - Select a node in 3D and click **`+ Nodal Load`** in the Right Panel (or click **`+ Add`** under *Nodal Loads* in the `⬇ Loads` tab).
   - Enter global forces `Fx, Fy, Fz` (`kN`) and/or moments `Mx, My, Mz` (`kN·m`). Orange-red 3D force arrows and magenta moment arrows appear immediately.
3. **Uniform & Trapezoidal Distributed Loads:**
   - Select a beam in 3D and click **`+ Dist Load`** in the Right Panel.
   - Choose **`Global`** (e.g. gravity/snow along `-Z` or wind along `+X`) or **`Local`** (e.g. wind pressure normal to an inclined rafter along local `z`).
   - Set start intensity `q₁` and end intensity `q₂` (`kN/m`). Click **`=Uni`** to copy `q₁` into `q₂` for a uniform load, or set different values for a linear trapezoidal/triangular ramp!

---

## Tutorial 5: Member End Releases (Pin-Ended Braces & Hinges)

By default, frame joints are **moment-resisting (rigid)**. To model pinned diagonal wind braces, bridge hangers, or simply-supported girders framing into columns via shear tabs:

1. Select the beam element in the 3D viewport.
2. In the Right Panel under **`Member End Releases (Local Hinges)`**, check **`My`** and **`Mz`** at **`Start i`** and/or **`End j`**.
3. White 3D hinge markers appear inset from the released ends, and Beam3D performs exact static condensation so bending moments vanish at the hinge while preserving distributed fixed-end shear transfer.
   *(Note: Avoid releasing torsion `T` at **both** ends of the same element unless intended, as a bar pinned torsionally at both ends can spin freely about its own axis.)*

---

## Tutorial 6: Solving, Post-Processing & Saving JSON

1. With **`Auto-Solve`** checked, Beam3D re-solves automatically in `< 1 ms` after every edit — or click **`▶ Solve FEA`** anytime.
2. **3D Contours & Ribbons (Right Panel → `🎨 3D Post-Processor & View`):**
   - **Contour:** Color-map beams by *Displacement `|u|`*, *Von Mises Stress*, *Combined Normal Stress*, *Yield Utilization (%)*, *Bending Moment*, *Shear*, *Axial Force `N`* (diverging Blue = compression, Red = tension), or *Torque*.
   - **3D Diagram:** Project translucent 3D *Bending Moment `My`/`Mz`*, *Shear `Vz`/`Vy`*, or *Axial `N`* ribbons directly off every beam in 3D.
   - **Deformed Shape & Animate:** Exaggerate 3D bending & twisting (`Auto Scale` picks an ideal visual scale) and check **`Animate`** to watch smooth harmonic deflection.
3. **2D Member Charts & Tables (Bottom Panel):**
   - Select any beam in 3D to inspect exact continuous `uz/uy`, `My/Mz`, and `Vz/N` plots along `x ∈ [0, L]`, or switch tabs to view the **Support Reactions** and **Nodal Displacements** tables.
4. **Save / Load Projects:** Click **`Save...`** or **`Open...`** in the top toolbar to export or import human-readable `.json` structural models.
