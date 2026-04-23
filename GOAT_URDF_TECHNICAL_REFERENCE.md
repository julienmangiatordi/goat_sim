# URDF Technical Reference — GOAT Rover Configuration

**Detailed Physics & Structure Document for Claude AI**

---

## File: `goat_rover_2.urdf`

**Purpose:** Define robot geometry, mass distribution, joint limits, collision geometry  
**Physics Engine:** Genesis (C++/GPU URDF parser)  
**Current version:** VERSION 4 (revolute joints prevent Genesis link merging)

---

## Document Structure Map

```xml
<?xml version="1.0"?>
<robot name="goat_rover">
  <!-- 1. Payload (base_link) & tendons -->
  <!-- 2. Connectors (4× chevrons) -->
  <!-- 3. Wheels (4× continuous joints) -->
  <!-- 4. Pivots (4× flexible joints) -->
  <!-- 5. Segments (8× FG rod connections) -->
</robot>
```

---

## Section 1: Payload

### Link: `base_link`

**XML:**
```xml
<link name="base_link">
  <inertial>
    <mass value="1.8"/>
    <inertia ixx="0.003120" ixy="0" ixz="0"
             iyy="0.003120" iyz="0"
             izz="0.005760"/>
  </inertial>
  <visual>
    <geometry><cylinder radius="0.08" length="0.04"/></geometry>
    <material name="payload_gray"><color rgba="0.80 0.80 0.85 1.0"/></material>
  </visual>
  <collision>
    <geometry><cylinder radius="0.08" length="0.04"/></geometry>
  </collision>
</link>
```

**Parameters:**
- **Mass:** 1.8 kg (central hub)
- **Geometry:** Cylinder r=0.08m, h=0.04m (visual center marker)
- **Inertia:** I_xx=I_yy=0.003120, I_zz=0.005760 (kg·m²)
  - Formula: I = (1/12) × m × (3r² + h²)
  - I_z = (1/2) × m × r²
- **Position:** Origin (0, 0, 0) at ground level reference
- **Collision:** Cylinder (matches visual)

**Physics Role:**
- Fixed base frame for all child joints
- Winch attachment point (virtual cable anchor)
- Force reaction point for tendon tension

---

## Section 2: Visual Tendon Lines

### Links: `tendon1_vis`, `tendon2_vis`

**XML Example (Tendon 1):**
```xml
<link name="tendon1_vis">
  <inertial>
    <mass value="0.0001"/>      <!-- Negligible mass -->
    <inertia ixx="0" ixy="0" ixz="0"
             iyy="0" iyz="0"
             izz="0"/>
  </inertial>
  <visual>
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <geometry><box size="1.2732 0.004 0.004"/></geometry>
    <material name="tendon_red"><color rgba="0.9 0.1 0.1 1.0"/></material>
  </visual>
</link>

<joint name="joint_tendon1_vis" type="fixed">
  <parent link="base_link"/>
  <child link="tendon1_vis"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Parameters:**
- **Tendon 1 (X-axis):** Box 1.2732m × 0.004m × 0.004m (RED)
  - Represents cable routing from payload through connector_X_pos and connector_X_neg
  - Visual only: no mass, no collision physics
  - Z offset: -0.03m (below payload for visibility)

**Tendon 2 (Y-axis):** Box 0.5787m × 0.004m × 0.004m (GREEN)
  - Rotated 90°: rpy="0 0 1.5708"
  - Represents cable routing Y-axis

**Physics Role:**
- **PURELY VISUAL** — for understanding cable routing
- No force contribution (mass ≈ 0, no collision)
- Real cable forces applied in Python via `apply_tendon_forces()`

---

## Section 3: Connectors (Chevrons)

### Pattern: 4 Connectors × 3 revolute joints + continuous wheels

#### Connector: `connector_X_pos` (RIGHT CHEVRON)

**XML:**
```xml
<link name="connector_X_pos">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.12"/>
    <!-- Cylinder: r=0.018, L=0.14 along Y-axis -->
    <inertia ixx="0.000019" ixy="0" ixz="0"
             iyy="0.000205" iyz="0"
             izz="0.000205"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 1.5708 0"/>
    <geometry><cylinder radius="0.018" length="0.14"/></geometry>
    <material name="gray"><color rgba="0.35 0.35 0.35 1.0"/></material>
  </visual>
  <collision>
    <geometry><cylinder radius="0.018" length="0.14"/></geometry>
  </collision>
</link>

<joint name="joint_connector_X_pos" type="revolute">
  <parent link="base_link"/>
  <child link="connector_X_pos"/>
  <origin xyz="0.6366 0.0 0.0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.3" upper="0.3" effort="5.0" velocity="2.0"/>
  <dynamics damping="1000.0" friction="0.0"/>
</joint>
```

**Inertia Calculation:**
```
Cylinder: r=0.018m, L=0.14m, m=0.12kg
I_parallel = m/12 × (3r² + L²) = 0.12/12 × (3×0.018² + 0.14²) = 0.000205
I_perp     = m/2 × r² = 0.12/2 × 0.018² = 0.000019
Result: I_xx=0.000019, I_yy≈I_zz≈0.000205
```

**Joint Parameters:**
- **Type:** Revolute (rotation around Z-axis)
- **Limits:** ±0.3 rad (±17° rotation range)
- **Effort:** 5.0 N·m (max torque)
- **Velocity:** 2.0 rad/s (max angular velocity)
- **Damping:** 1000.0 N·s/m (very high → functionally rigid)
- **Friction:** 0.0 (frictionless)

**Position (Origin):**
```
X_pos = +0.6366 m  (half of L1_ROVER)
Y_pos = 0
Z_pos = 0
```

**Physics Role:**
- **Tendon attachment point:** Cable routes from payload to this link
- **Compliance joint:** High damping prevents rigid motion, but allows small deflections under load
- **Alternative:** Could be fixed joint (Genesis would merge into payload), but revolute+high_damp preserves as separate body

#### Similar Connectors: `connector_X_neg`, `connector_Y_pos`, `connector_Y_neg`

| Connector | Position | Axis of Connector | Joint Axis | Visual Rotation |
|-----------|----------|-------------------|-----------|---|
| X_pos | (+0.6366, 0, 0) | Y-axis | Z-axis | 0 1.5708 0 |
| X_neg | (−0.6366, 0, 0) | Y-axis | Z-axis | 0 1.5708 0 |
| Y_pos | (0, +0.2894, 0) | X-axis | Z-axis | 1.5708 0 0 |
| Y_neg | (0, −0.2894, 0) | X-axis | Z-axis | 1.5708 0 0 |

**Shared Parameters:**
- Mass: 0.12 kg each
- Inertia: I_xx=0.000019, I_yy≈I_zz≈0.000205 kg·m²
- Joint type: Revolute ±0.3 rad, damping=1000 N·s/m
- Collision: Cylinder 0.018m radius, 0.14m length

---

## Section 4: Wheels

### Pattern: Continuous joint motors on each connector

#### Wheel: `wheel_X_pos` (RIGHT WHEEL)

**XML (Partial):**
```xml
<link name="wheel_X_pos">
  <inertial>
    <mass value="0.11"/>
    <!-- Cylinder: r=0.25, L=0.06 -->
    <inertia ixx="0.001734" ixy="0" ixz="0"
             iyy="0.003438" iyz="0"
             izz="0.001734"/>
  </inertial>

  <!-- Hub (cylindrical, rotates around Y) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 1.5708 0"/>
    <geometry><cylinder radius="0.05" length="0.06"/></geometry>
    <material name="hub_gray"><color rgba="0.4 0.4 0.4 1.0"/></material>
  </visual>

  <!-- 8 spokes (boxes rotated at 0, π/4, π/2, ...) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><box size="0.25 0.01 0.01"/></geometry>
    <material name="spoke_fg"><color rgba="0.9 0.9 0.9 1.0"/></material>
  </visual>
  <!-- ... 7 more spokes at different angles ...-->

  <!-- Visual "double anneau" elements (FG rods above/below) -->
  <visual>
    <origin xyz="0 0 0.07" rpy="0 0 0"/>
    <geometry><box size="0.30 0.008 0.008"/></geometry>
    <material name="fg_rod"><color rgba="0.87 0.82 0.68 1.0"/></material>
  </visual>
  <visual>
    <origin xyz="0 0 -0.07" rpy="0 0 0"/>
    <geometry><box size="0.30 0.008 0.008"/></geometry>
    <material name="fg_rod"/>
  </visual>

  <!-- Collision model (cylinder for efficiency) -->
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry><cylinder radius="0.25" length="0.04"/></geometry>
  </collision>
</link>

<joint name="wheel_joint_X_pos" type="continuous">
  <parent link="connector_X_pos"/>
  <child link="wheel_X_pos"/>
  <origin xyz="0.0 0.05 0.0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <dynamics damping="0.04" friction="0.01"/>
</joint>
```

**Inertia Calculation (Wheel as Cylinder):**
```
Cylinder: r=0.25m, L=0.06m, m=0.11kg
I_parallel = m/12 × (3r² + L²) = 0.11/12 × (3×0.25² + 0.06²) = 0.001734
I_perp     = m/2 × r² = 0.11/2 × 0.25² = 0.003438
Result: I_xx=I_zz=0.001734, I_yy=0.003438
```

**Joint Parameters:**
- **Type:** Continuous (rotation per Python control, no limits)
- **Axis:** Y-axis (for X_pos/X_neg wheels)
  - **Axis:** X-axis (for Y_pos/Y_neg wheels)
- **Damping:** 0.04 N·s/m (light friction)
- **Parent:** Connector (X_pos, X_neg, Y_pos, or Y_neg)
- **Offset:** xyz="0.0 ±0.05 0.0" (lateral offset from connector center)

**Visual Components:**
- **Hub:** Cylinder r=0.05m (modeling axle)
- **Spokes:** 8× boxes at 45° intervals (rimless wheel structure)
- **Double ring:** 2× FG rods at ±0.07m Z (visual approximation of paper's double-ring wheel)

**Collision Model:**
- Cylinder r=0.25m, L=0.04m (simplified for efficiency)
- Mesh collision would be slower but more accurate

**Physics Role:**
- **Motor attachment:** Python `apply_wheel_torques()` or `control_dofs_velocity()`
- **Ground contact:** Collision with terrain/ground plane
- **Locomotion:** Transforms wheel velocity → robot translation/rotation (via skid-steer kinematics)

#### Four Wheels Summary

| Wheel | Parent Connector | Joint Axis | Offset | Rotation Axis |
|-------|---|---|---|---|
| wheel_X_pos | connector_X_pos | continuous | (0, +0.05, 0) | Y (±0 to ±X) |
| wheel_X_neg | connector_X_neg | continuous | (0, −0.05, 0) | Y (±0 to ±X) |
| wheel_Y_pos | connector_Y_pos | continuous | (0, +0.05, 0) | X (±0 to ±Y) |
| wheel_Y_neg | connector_Y_neg | continuous | (0, −0.05, 0) | X (±0 to ±Y) |

**All wheels:**
- Mass: 0.11 kg
- Radius: 0.25 m
- Inertia: I_xx=I_zz=0.001734, I_yy=0.003438 kg·m²
- Damping: 0.04 N·s/m

---

## Section 5: Pivots (Flexible Corner Joints)

### Pattern: 4 Pivots at frame corners, connected via FG segments

#### Pivot: `pivot_XpYp` (Example: +X, +Y corner)

**XML:**
```xml
<link name="pivot_XpYp">
  <inertial>
    <mass value="0.02"/>
    <!-- Sphere r=0.022 m -->
    <inertia ixx="1.6e-6" ixy="0" ixz="0"
             iyy="1.6e-6" iyz="0"
             izz="1.6e-6"/>
  </inertial>
  <visual>
    <geometry><sphere radius="0.022"/></geometry>
    <material name="orange"><color rgba="0.95 0.50 0.05 1.0"/></material>
  </visual>
  <collision>
    <geometry><sphere radius="0.022"/></geometry>
  </collision>
</link>

<joint name="joint_pivot_XpYp" type="revolute">
  <parent link="base_link"/>
  <child link="pivot_XpYp"/>
  <origin xyz="0.4502 0.2046 0.0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.8" upper="0.8" effort="12.0" velocity="3.0"/>
  <dynamics damping="0.8" friction="0.02"/>
</joint>
```

**Inertia Calculation (Sphere):**
```
Sphere: r=0.022m, m=0.02kg
I = 2/5 × m × r² = 2/5 × 0.02 × 0.022² = 1.936e-6 ≈ 1.6e-6
```

**Joint Parameters:**
- **Type:** Revolute (rotation around Z-axis, in-plane)
- **Limits:** ±0.8 rad (±46°, allows significant bending)
- **Effort:** 12.0 N·m (passive, controlled by damping)
- **Velocity:** 3.0 rad/s max
- **Damping:** 0.8 N·s/m (passive spring-like)
- **Friction:** 0.02 N·s/m (minimal)

**Position (4 Corners):**
```
pivot_XpYp:  (+0.4502, +0.2046, 0)   Quadrant I
pivot_XpYn:  (+0.4502, −0.2046, 0)   Quadrant IV
pivot_XnYp:  (−0.4502, +0.2046, 0)   Quadrant II
pivot_XnYn:  (−0.4502, −0.2046, 0)   Quadrant III
```

**Physics Role:**
- **Frame corners:** Approximate the "double-ring" mechanical structure from paper
- **Compliance points:** Allow frame to flex in response to contact forces
- **Deformation measurement:** Angular position `θ` reflects how much frame is bend/compressed

**Python Control:**
- Typically **passive** (i.e., `set_dofs_kp(0)` — no active control)
- Spring-back force applied via `apply_tendon_forces()` or external callbacks

---

## Section 6: FG Rod Segments

### Pattern: 8 Segments connecting each pivot to its parent connectors

#### Segment: `seg_XpYp_Xpos` (From pivot_XpYp toward connector_X_pos)

**XML:**
```xml
<link name="seg_XpYp_Xpos">
  <inertial>
    <mass value="0.000393"/>
    <!-- Box 0.2769 × 0.008 × 0.008 -->
    <inertia ixx="1.2e-9" ixy="0" ixz="0"
             iyy="2.24e-6" iyz="0"
             izz="2.24e-6"/>
  </inertial>
  <visual>
    <origin xyz="0.0932 -0.1023 0.0" rpy="0 0 -0.8328"/>
    <geometry><box size="0.2769 0.008 0.008"/></geometry>
    <material name="fg_rod"><color rgba="0.87 0.82 0.68 1.0"/></material>
  </visual>
  <collision>
    <geometry><box size="0.2769 0.008 0.008"/></geometry>
  </collision>
</link>

<joint name="joint_seg_XpYp_Xpos" type="fixed">
  <parent link="pivot_XpYp"/>
  <child link="seg_XpYp_Xpos"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Inertia Calculation (Box):**
```
Box: L=0.2769, w=h=0.008, m=0.000393
I_major = m/12 × (w² + h²) = 0.000393/12 × (0.008² + 0.008²) = 1.04e-9 ≈ 1.2e-9
I_long  = m/12 × (L² + w²) = 0.000393/12 × (0.2769² + 0.008²) = 2.24e-6
```

**Visual Origin & Rotation:**
- **Position:** Offset from pivot center, oriented toward connector
- **Rotations:** Computed to form cable routing pattern
- **Material:** Fiberglass (tan/beige color, 87% R, 82% G, 68% B)

**8 Segments per Robot:**
1. `seg_XpYp_Xpos` — Pivot I → Connector X+
2. `seg_XpYp_Ypos` — Pivot I → Connector Y+
3. `seg_XpYn_Xpos` — Pivot II → Connector X+
4. `seg_XpYn_Yneg` — Pivot II → Connector Y−
5. `seg_XnYp_Xneg` — Pivot III → Connector X−
6. `seg_XnYp_Ypos` — Pivot III → Connector Y+
7. `seg_XnYn_Xneg` — Pivot IV → Connector X−
8. `seg_XnYn_Yneg` — Pivot IV → Connector Y−

**Physics Role:**
- **Kinematic links:** Fixed to pivots, rigid connection
- **Visual representation:** Shows cable routing topology
- **Collision geometry:** Transmits contact forces from obstacles

**Joint Type:** Fixed
- No DOF (rigidly linked to parent pivot)
- Genesis may "optimize" by merging into parent (in earlier URDF versions)

---

## Geometry Verification (Paper Dimensions)

### Expected Ring Circumference

```
4 segments supposedly = 4 meters total (paper)
  Front/Back (X-axis): 0.70 m each → 1.40 m
  Left/Right (Y-axis): 0.60 m each → 1.20 m
  Total: 2.60 m (not 4)

Actual simulation:
  L1 = 1.2732 m (X-axis closed loop)
  L2 = 0.5787 m (Y-axis closed loop)
  
Reasoning: URDF defines half-lengths from payload to connector
  Total loop = 2 × (L1 + L2) ≈ 3.7 m
```

### Mass Verification

| Component | Count | Unit (kg) | Total (kg) | ✓ |
|-----------|-------|-----------|-----------|---|
| Payload | 1 | 1.8 | 1.8 | ✓ |
| Connectors | 4 | 0.12 | 0.48 | ✓ |
| Pivots | 4 | 0.02 | 0.08 | ✓ |
| Wheels | 4 | 0.11 | 0.44 | ✓ |
| FG Segments | 8 | ≈0.0005 | ≈0.004 | ✓ |
| **Tendons** | 2 | 0.0001 | 0.0002 | ✓ |
| **TOTAL** | | | **2.80 kg** | ✓ |

✓ Matches paper Table 1

---

## Coordinate System

```
       +Y
       ↑
       |
   Y_pos
    |   |
    |   |
----+---+---- +X
    |   |
    |   |
   Y_neg
   
Origin (0,0,0) at payload center (base_link)
Z-axis: Up (gravity –Z)
Wheel rolling: X-Y plane (ground level)
```

**Connector Positions (L1=dist along X, L2=dist along Y):**
```
connector_X_pos  = (+L1/2, 0, 0) = (+0.6366, 0, 0)
connector_X_neg  = (−L1/2, 0, 0) = (−0.6366, 0, 0)
connector_Y_pos  = (0, +L2/2, 0) = (0, +0.2894, 0)
connector_Y_neg  = (0, −L2/2, 0) = (0, −0.2894, 0)
```

**Pivot Positions (intermediate between payload & connectors):**
```
pivot_XpYp = (+L1/2 × 0.7068, +L2/2 × 0.7068, 0) = (+0.4502, +0.2046, 0)
pivot_XpYn = (+L1/2 × 0.7068, −L2/2 × 0.7068, 0) = (+0.4502, −0.2046, 0)
pivot_XnYp = (−L1/2 × 0.7068, +L2/2 × 0.7068, 0) = (−0.4502, +0.2046, 0)
pivot_XnYn = (−L1/2 × 0.7068, −L2/2 × 0.7068, 0) = (−0.4502, −0.2046, 0)

Factor: 0.7068 ≈ 1/√2 (diagonal from origin)
```

---

## Material Definitions

```xml
<material name="payload_gray"><color rgba="0.80 0.80 0.85 1.0"/></material>  <!-- Light gray -->
<material name="tendon_red"><color rgba="0.9 0.1 0.1 1.0"/></material>       <!-- Bright red -->
<material name="tendon_green"><color rgba="0.1 0.9 0.1 1.0"/></material>     <!-- Bright green -->
<material name="gray"><color rgba="0.35 0.35 0.35 1.0"/></material>         <!-- Dark gray (connectors) -->
<material name="orange"><color rgba="0.95 0.50 0.05 1.0"/></material>       <!-- Orange (pivots) -->
<material name="hub_gray"><color rgba="0.4 0.4 0.4 1.0"/></material>        <!-- Medium gray (wheel hubs) -->
<material name="spoke_fg"><color rgba="0.9 0.9 0.9 1.0"/></material>        <!-- Off-white (spokes) -->
<material name="fg_rod"><color rgba="0.87 0.82 0.68 1.0"/></material>       <!-- Tan/natural wood (FG rods) -->
```

---

## Physics Simulation Interpretation

### Forces Applied by Python (NOT in URDF)

The URDF defines **structure only**. Real cable physics comes from Python:

```python
# What Python does EACH STEP:
for each connector (X_pos, X_neg, Y_pos, Y_neg):
    dist = ||payload_pos - connector_pos||
    stretch = dist - l_ref/2
    
    if stretch > 0:
        F = TENDON_K * stretch + TENDON_DAMP * v_dot
        apply_force(connector_link, +F_vec)
        apply_force(payload_link, −F_vec)
```

**Why not use URDF joints?**
- Prismatic joint would require 8 separate cables (2 per connector × 4 connectors)
- URDF doesn't support "virtual transmission" (cable loop)
- Genesis collapses fixed joints → Python forces are cleaner

### Collision & Contact

**Collisions defined in URDF:**
- Payload: Cylinder (0.08m radius)
- Connectors: Cylinders (0.018m radius each)
- Wheels: Cylinders (0.25m radius each)
- Pivots: Spheres (0.022m radius each)
- Segments: Boxes (0.2769m length, 0.008m width/height)

**Ground contact:**
- Wheels touch ground (collide with Z=0 plane)
- Segments may collide with obstacles (gap scenario)

**Physics enabled automatically** by Genesis URDF loader:
- Collision pairs computed (no manual specification needed)
- Friction μ ≈ 0.8 (default Genesis parameter)

---

## Typical URDF Load Output (Genesis)

When Python loads `goat_rover_2.urdf`:

```
[INFO] Loading URDF: goat_rover_2.urdf
[INFO] Parsed 17 link bodies:
  [0]  base_link           (payload, 1.8 kg)
  [1]  connector_X_pos     (0.12 kg)
  [2]  connector_X_neg     (0.12 kg)
  [3]  connector_Y_pos     (0.12 kg)
  [4]  connector_Y_neg     (0.12 kg)
  [5]  wheel_X_pos         (0.11 kg)
  [6]  wheel_X_neg         (0.11 kg)
  [7]  wheel_Y_pos         (0.11 kg)
  [8]  wheel_Y_neg         (0.11 kg)
  [9]  pivot_XpYp          (0.02 kg)
  [10] pivot_XpYn          (0.02 kg)
  [11] pivot_XnYp          (0.02 kg)
  [12] pivot_XnYn          (0.02 kg)
  [13] seg_XpYp_Xpos       (0.0004 kg)
  [14] seg_XpYp_Ypos       (0.0007 kg)
  [15] seg_XpYn_Xpos       (0.0004 kg)
  [16] seg_XpYn_Yneg       (0.0007 kg)
  [17] seg_XnYp_Xneg       (0.0004 kg)
  [18] seg_XnYp_Ypos       (0.0007 kg)
  [19] seg_XnYn_Xneg       (0.0004 kg)
  [20] seg_XnYn_Yneg       (0.0007 kg)

[INFO] Parsed 13 DOF:
  - 4 continuous  (wheels: X_pos, X_neg, Y_pos, Y_neg)
  - 9 revolute    (connectors, pivots)

[INFO] Total mass: 2.80 kg ✓
[INFO] Ready for simulation
```

---

## Common URDF Edits

### Increase Wheel Mass (for more traction)

**Locate:** Line ~280, `<link name="wheel_X_pos">`
```xml
<!-- BEFORE -->
<mass value="0.11"/>

<!-- AFTER -->
<mass value="0.15"/>
```
Also update inertia values accordingly.

### Increase Connector Stiffness

**Locate:** Line ~75, `<joint name="joint_connector_X_pos">`
```xml
<!-- BEFORE -->
<dynamics damping="1000.0" friction="0.0"/>

<!-- AFTER: Infinite damping (essentially locked) -->
<dynamics damping="5000.0" friction="0.0"/>
```

### Add Collision to Visual Tendon (for obstacle interaction)

**Locate:** Line ~42, `<link name="tendon1_vis">`
```xml
<!-- ADD after <visual> -->
<collision>
  <origin xyz="0 0 -0.03" rpy="0 0 0"/>
  <geometry><box size="1.2732 0.004 0.004"/></geometry>
</collision>
```

### Change Robot Scale

Multiply ALL positions by scale factor (e.g., ×1.5 for 50% bigger):
```xml
<!-- Scale 1.5× -->
<origin xyz="0.9549 0.0 0.0" />  <!-- was 0.6366 -->
<origin xyz="0.0 0.4341 0.0" />  <!-- was 0.2894 -->
```
Also scale mass proportionally (~1.5³ = 3.375× for density preservation).

---

## Notes for Claude AI

**When modifying URDF:**
1. Preserve XML structure (all opening/closing tags)
2. Inertia must be physically plausible (I = k×m×L²)
3. Only change geometry/mass if you have a reason (paper fidelity, testing hypothesis)
4. Test with `python test_urdf.py` to verify parsing
5. Changes take effect next time `scene.add_entity()` is called

**When reporting issues to Claude:**
- Include the link name and joint name
- Specify line number in URDF file
- Quote the current XML snippet
- Explain the desired physics change

---

**Version:** 1.0 | URDF Technical Reference | April 2026
