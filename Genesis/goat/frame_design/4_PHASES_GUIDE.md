**Goal:** Simulate the 4 morphological reconfiguration phases of the GOAT frame (Circular → Rover → Reconfiguration → Sphere) based on *Polzin et al., Science Robotics 2025*.

***

## 1. Physical Background of the GOAT Robot

The GOAT robot is built around two interconnected fiberglass rings forming a pattern of four symmetric convex lenses. Two orthogonal tendons T1 (X-axis) and T2 (Y-axis), driven by electric winches from the central payload, allow the frame morphology to be reconfigured.

### The 4 Phases from Fig. 2B(i) of the Paper

| Phase | Name | T1 (`u1`) | T2 (`u2`) | Physical description |
|-------|------|-----------|-----------|----------------------|
| ① | Circular | Long (u=1) | Short (u=0) | Near-circular frame, T2 contracted along X |
| ② | Rover | Long (u=1) | Long (u=1) | Elongated elliptical frame, T1/T2 ratio ≈ 1 |
| ③ | Reconfiguration | Mid-stroke (u=0.5) | Mid-stroke (u=0.5) | 3D transition, beginning of out-of-plane folding |
| ④ | Sphere | Short (u=0) | Short (u=0) | Compact spherical frame, wheels folded inward |

> **`u` convention:** `u=1` means tendon at full tension (length reduced by `CONTRACTION_MAX`). `u=0` means tendon released (nominal length). This is the inverse of the physical length `l` in the paper (`l` short → contracted frame → `u` high).

### Role of the Two Tendons

- **T1 (X-axis, 0° and 180°)** controls in-plane reconfiguration. Shortening T1 stretches the frame along X and compresses it along Y → aspect ratio increases.
- **T2 (Y-axis, 90° and 270°)** controls out-of-plane folding. Shortening T2 causes the frame to leave its plane and fold into a sphere.
- The reconfiguration path in `(l1, l2)` space must avoid the torsional instability region (torsion stiffness < 0, Fig. 2C of the paper).

***

## 2. Geometry Generation (OBJ Files)

### 2.1 Wavy Rings (`ring_A.obj`, `ring_B.obj`)

Both rings are generated as toroidal tubes with a sinusoidal Z profile (amplitude `wave_amp = 0.05 m`, frequency 2, phase offset `phase_A = 0` and `phase_B = π`). The π phase offset between the two rings creates the convex lens pattern characteristic of the GOAT.

```python
cz = wave_amp * np.sin(2*U + phase)  # sinusoidal Z profile
```

**Parameters**: `R = 0.25 m`, `tube_r = 0.025 m`, `n_major = 80` sections, `n_tube = 12` sections.

### 2.2 Struts (`strut_0.obj` to `strut_3.obj`)

The 4 rigid struts connect the two rings at 45°, 135°, 225°, 315° (i.e., at the lens crossing points). They are generated as cylinders between the corresponding points of ring_A and ring_B at those angles.

### 2.3 Physical Tendons (`tendon_T1.obj`, `tendon_T2.obj`)

Tendons are generated as thin cylindrical tubes (`tendon_r = 0.006 m`) connecting the centroid of the angular anchor zones on ring_A and ring_B.

**Important detail:** `n=20` cylinder sections are used to guarantee a sufficient number of particles relative to Genesis's `n_support_neighbors=4` parameter. Too few particles triggered a *"kth out of bounds"* error in the PBD solver.

***

## 3. Simulation Architecture in Genesis

### 3.1 PBD Materials

All physical entities use `gs.materials.PBD.Cloth` with adapted stiffness parameters:

| Entity | `rho` (kg/m³) | `stretch_compliance` | `bending_compliance` | Role |
|--------|---------------|---------------------|---------------------|------|
| Rings (ring_A/B) | 800 | 1e-9 | 1e-6 | Soft, deformable frame |
| Struts | 1200 | 1e-12 | 1e-9 | Rigid connections |
| Tendons T1/T2 | 500 | 1e-13 | 1e-5 | Quasi-inextensible cables |

> **Note:** A very low `stretch_compliance` (1e-13) makes the tendon quasi-inextensible, simulating a steel cable in XPBD.

### 3.2 Scene Options

```python
sim_options = SimOptions(dt=2e-3, substeps=20, gravity=(0,0,0))
vis_options = VisOptions(show_world_frame=True, n_support_neighbors=4)
```

- **`gravity=(0,0,0)`**: gravity disabled to isolate tendon effects without the frame collapsing.
- **`substeps=20`**: numerical stability of the XPBD solver with very stiff tendons.
- **`n_support_neighbors=4`**: critical fix (see §4.2).

***

## 4. Bug History and Solutions

### 4.1 Bug 1: `TypeError: perf_dispatch() got an unexpected keyword argument 'first_warmup'`

**Cause:** Version conflict between Genesis 0.4.6 and Quadrants 0.6.2. The `@qd.perf_dispatch(first_warmup=...)` decorator in `constraint/solver.py` used an API absent from Quadrants 0.6.2.

**Fix:**
```bash
pip install --upgrade quadrants  # → version 0.8.0
```

Quadrants is the GPU compilation backend of Genesis (fork of Taichi, June 2025). It does not guarantee backward compatibility, making version conflicts common if both packages are not updated together.

***

### 4.2 Bug 2: `kth out of bounds` in the Viewer

**Cause:** Tendon meshes had too few particles relative to the viewer's `n_support_neighbors` parameter. A cylinder generated with `n=12` sections and few height segments resulted in a total particle count insufficient for the rendering support neighborhood.

**Fixes applied simultaneously:**
1. `n=20` sections in `cylinder_between()` for tendons (more particles).
2. `n_support_neighbors=4` (reduced value) in `VisOptions`.

***

### 4.3 Bug 3: `set_external_force()` Does Not Exist

**Cause:** The initial approach attempted to apply external forces directly on ring particles to simulate tendon traction. The method `set_external_force()` does not exist in Genesis's `PBD2DEntity` API.

**What exists in Genesis PBD:**
- `get_particles_pos()` / `set_particles_pos()` — direct read/write of positions
- `get_particles_vel()` / `set_particles_vel()` — read/write of velocities
- No method to apply persistent external forces in the PBD solver

**Solutions explored:**

| Solution | Description | Result |
|----------|-------------|--------|
| **A: Fictitious forces** | Apply forces via `set_particles_vel` proportional to (u1, u2) | Frame does not move — forces do not persist across XPBD steps |
| **B: Pure kinematics** | Directly apply target positions via `set_particles_pos` each step (global morph with anisotropic scaling) | Works visually but non-physical — no tendon, no mechanics |
| **C: `edges_info.len_rest` patch** | Directly modify the rest length of XPBD solver edges to simulate a winch | Physically correct approach — tendons contract as PBD entities and exert forces on neighboring entities |

***

### 4.4 Fundamental Limitation: PBD Inter-Entity Coupling

**Unresolved core issue:** In Genesis, the PBD solver does not automatically couple edges between two distinct entities. Rings and tendons are independent `PBD2DEntity` objects. Tendon contraction (reducing `len_rest`) does not translate into forces on ring particles unless:

1. Tendon and ring particles **share nodes** (co-location), which is not natively supported when importing separate meshes.
2. **Explicit distance constraints** are created between particles belonging to different entities — an API not publicly documented in Genesis 0.4.6.
3. The **rigid solver** is used with joints, which does not apply to PBD.

**Consequence:** Solution C (patching `len_rest`) correctly contracts the tendon as a visual object, but does not mechanically transmit this tension to the rings. The deformation measurements in logs (`L_T1_mm`, `L_T2_mm`) reflect the tendon mesh bounding box, not the frame deformation.

***

## 5. Final Implemented Solution: Solution C

### 5.1 XPBD Physical Winch Principle

```python
def set_tendon_lengths(u1: float, u2: float):
    f1 = 1.0 - CONTRACTION_MAX * u1   # length factor ∈ [0.65, 1.0]
    f2 = 1.0 - CONTRACTION_MAX * u2
    for i in range(n_edges_T1):
        solver.edges_info[e_start_T1 + i].len_rest = len_rest_T1[i] * f1
    for i in range(n_edges_T2):
        solver.edges_info[e_start_T2 + i].len_rest = len_rest_T2[i] * f2
```

`CONTRACTION_MAX = 0.35` → maximum contraction of 35%, consistent with tendon displacements observed in the paper (~10–30 cm on a 1.65 m frame, i.e., ~10–20%).

### 5.2 4-Phase Control Sequence

```
Phase ①  Circular         u1=1.0, u2=0.0   T1 contracted, T2 free
Phase ②  Rover            u1=1.0, u2=1.0   T1+T2 contracted, elongated frame
Phase ③  Reconfiguration  u1=0.5, u2=0.5   mid-stroke, 3D transition
Phase ④  Sphere           u1=0.0, u2=0.0   tendons released, rest shape
```

Transitions between phases use cosine easing:

$$ s(\alpha) = \frac{1 - \cos(\pi \alpha)}{2}, \quad \alpha \in [0,1] $$

to avoid velocity discontinuities that destabilize the XPBD solver.

### 5.3 Kinematic Pinning of Struts

At each step, strut positions are reset to their reference values:

```python
def pin_struts():
    for ent, ref in zip(ent_struts, strut_refs):
        ent.set_particles_pos(ref.copy())
```

This prevents struts from drifting (zero gravity but residual inertia) and maintains the correct connection geometry between the two rings.

***

## 6. Simulation Log Analysis

### 6.1 Logs from the Previous Version (`four_phases_log.csv` — Solution A)

The values `L_T1_mm ≈ 19 mm` and `L_T2_mm ≈ 16 mm` remained **constant across all phases**. This confirmed that the force-based approach (`set_external_force`) produced no measurable frame deformation — particles did not move.

### 6.2 What Logs Measure in Solution C

The `measure_tendon_length` metric computes the norm of the tendon bounding box:

```python
np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)) * 1e3  # mm
```

This captures the **geometric deformation of the tendon itself**, not the tension transmitted to the rings. It is a proxy for tendon contraction, not frame reconfiguration.

***

## 7. Identified Limitations of Genesis

### 7.1 No External Force API for PBD

The `PBD2DEntity` API only exposes position and velocity read/write methods. There is no method to inject persistent forces between steps, unlike the rigid or FEM solvers.

**Impact:** Impossible to simulate a winch as a restoring force (spring-like) without modifying Genesis source code or using the `edges_info.len_rest` patch.

### 7.2 No PBD Inter-Entity Coupling

PBD constraints (stretch, bending) are defined intra-entity at `scene.build()` time. There is no public API in Genesis 0.4.6 to create distance constraints between particles belonging to two different entities.

**Impact:** The physical tendon cannot exert mechanical force on adjacent rings via the XPBD solver. Force transmission would require node sharing or a contact API not exposed for PBD.

### 7.3 Automatic Remeshing

Genesis performs automatic remeshing (`Remeshing for tetrahedralization`) when adding PBD entities. This remeshing can change the number and indices of particles relative to the original OBJ mesh, making a direct mapping between OBJ vertex indices and solver particle indices impossible.

**Impact:** Anchor computation by angle (`arctan2` on vertices) must be performed after `scene.reset()` using `get_particles_pos()`, not on the original OBJ mesh.

### 7.4 `n_support_neighbors` and Small Meshes

The Genesis viewer requires a minimum number of neighboring particles for support rendering. Overly coarse meshes (< ~20 particles) trigger a *"kth out of bounds"* error that crashes the viewer on startup.

**Workaround:** Increase mesh resolution (`n=20` sections minimum) and/or reduce `n_support_neighbors` in `VisOptions`.

***

## 8. Future Improvement Directions

### 8.1 Coupling via Shared Nodes

Generate a single mesh combining rings + tendons with common nodes at anchor points. This would allow the XPBD solver to naturally propagate tendon constraints to the rings without any additional API.

**Difficulty:** Requires coherent multi-entity mesh generation and control of node indexing after Genesis remeshing.

### 8.2 FEM or Rigid Solver with Joints

Genesis's FEM solver exposes more API surface for forces and constraints. A hybrid approach (frame in FEM + tendons as length constraints) would be physically closer to the ANCF model in the paper.

### 8.3 Full ANCF Model

The paper uses an ANCF (*Absolute Nodal Coordinate Formulation*) method to predict 3D deformations of elastic rods. This model accurately captures flexural rigidity, torsional rigidity, and elastic potential energy. A direct Python implementation (without Genesis) using `scipy.optimize` or JAX could faithfully reproduce the curves from Fig. 2B of the paper.

### 8.4 Calibrating `CONTRACTION_MAX`

The value `CONTRACTION_MAX = 0.35` is chosen empirically. For a faithful simulation, it should be calibrated from Fig. 2C of the paper: the maximum gap width is 1.8 m (frame of ~1.65 m diameter), and the minimum reduction is approximately 0.3 m, i.e., ~17%. A value of 0.20 would be more physically realistic.

***

## 9. Final File: `four_phases_tendon.py`

### Code Structure

```
four_phases_tendon.py
├── PART 1 — OBJ Generation
│   ├── make_wavy_ring_mesh()      → ring_A.obj, ring_B.obj
│   ├── cylinder_between()         → strut_0..3.obj
│   └── make_tendon_mesh()         → tendon_T1.obj, tendon_T2.obj
│
└── PART 2 — Genesis Simulation
    ├── Scene initialization        → PBD Cloth, viewer 960×720
    ├── set_tendon_lengths(u1, u2)  → edges_info.len_rest patch (XPBD winch)
    ├── pin_struts()                → kinematic maintenance of connections
    ├── measure_tendon_length()     → deformation proxy (bounding box)
    └── 4-phase loop                → 520 steps, cosine easing, CSV log
```

### Key Parameters

| Parameter | Value | Reason |
|-----------|-------|--------|
| `R` | 0.25 m | Ring radius (frame ~0.5 m, 1/3 of real scale at 1.65 m) |
| `wave_amp` | 0.05 m | Ring wave amplitude (creates the lens pattern) |
| `CONTRACTION_MAX` | 0.35 | Maximum tendon contraction (35% of initial length) |
| `dt` | 2e-3 s | Stable time step for XPBD with `substeps=20` |
| `substeps` | 20 | Number of XPBD sub-steps for stiff tendon stability |
| `N_TRANS` | 50 | Transition steps between phases (cosine easing) |
| `N_HOLD` | 80 | Stable hold steps for visibility in the viewer |

***

## 10. Implementation Decision Summary

| Decision | Physical justification | Genesis constraint |
|----------|----------------------|-------------------|
| PBD Cloth for all bodies | Modeling flexible elastic rods as cloth | Only solver exposing `set_particles_pos` in bulk |
| `gravity=(0,0,0)` | Isolate tendon mechanics from self-weight | Without this, the frame collapses immediately without rigid support |
| Strut pinning at each step | Struts are not mechanically coupled to the rings | No PBD inter-entity joints in Genesis |
| `edges_info.len_rest` patch | Only way to impose active contraction in the XPBD solver | No `set_external_force` API for PBD |
| Cosine easing on transitions | Avoid velocity discontinuities that destabilize XPBD | PBD solver is explicit and sensitive to impulses |
| Bounding box measurement | Simple and robust against remeshing | OBJ → Genesis particle mapping is not direct |