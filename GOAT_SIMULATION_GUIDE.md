

---

## URDF Architecture

**File:** `goat_rover_2.urdf` (main model)  
**Alternative versions:** 
- `goat_v5_soft_joints.urdf` (earlier cable-attaching variant)
- Deprecated: v3, v4 (earlier iterations)

### Component Hierarchy

```
base_link (PAYLOAD, 1.8 kg)
│
├─ tendon1_vis ──────── [VISUAL ONLY: red bar along X-axis, L=1.2732m]
├─ tendon2_vis ──────── [VISUAL ONLY: green bar along Y-axis, L=0.5787m]
│
├─ connector_X_pos ──── [RIGHT CHEVRON, 0.12 kg @ (+0.6366, 0, 0)]
│   └─ wheel_X_pos ──── [CONTINUOUS DRIVE WHEEL, 0.11 kg, Y-axis rotation]
│
├─ connector_X_neg ──── [LEFT CHEVRON, 0.12 kg @ (−0.6366, 0, 0)]
│   └─ wheel_X_neg ──── [CONTINUOUS DRIVE WHEEL, 0.11 kg, Y-axis rotation]
│
├─ connector_Y_pos ──── [FRONT CHEVRON, 0.12 kg @ (0, +0.2894, 0)]
│   └─ wheel_Y_pos ──── [CONTINUOUS DRIVE WHEEL, 0.11 kg, X-axis rotation]
│
├─ connector_Y_neg ──── [BACK CHEVRON, 0.12 kg @ (0, −0.2894, 0)]
│   └─ wheel_Y_neg ──── [CONTINUOUS DRIVE WHEEL, 0.11 kg, X-axis rotation]
│
├─ pivot_XpYp ────────── [ORANGE SPHERE, 0.02 kg @ (+0.4502, +0.2046, 0)]
│   ├─ seg_XpYp_Xpos ── [FG ROD, connects pivot→connector_X_pos]
│   └─ seg_XpYp_Ypos ── [FG ROD, connects pivot→connector_Y_pos]
│
├─ pivot_XpYn ────────── [ORANGE SPHERE, 0.02 kg @ (+0.4502, −0.2046, 0)]
│   ├─ seg_XpYn_Xpos ── [FG ROD, connects pivot→connector_X_pos]
│   └─ seg_XpYn_Yneg ── [FG ROD, connects pivot→connector_Y_neg]
│
├─ pivot_XnYp ────────── [ORANGE SPHERE, 0.02 kg @ (−0.4502, +0.2046, 0)]
│   ├─ seg_XnYp_Xneg ── [FG ROD, connects pivot→connector_X_neg]
│   └─ seg_XnYp_Ypos ── [FG ROD, connects pivot→connector_Y_pos]
│
└─ pivot_XnYn ────────── [ORANGE SPHERE, 0.02 kg @ (−0.4502, −0.2046, 0)]
    ├─ seg_XnYn_Xneg ── [FG ROD, connects pivot→connector_X_neg]
    └─ seg_XnYn_Yneg ── [FG ROD, connects pivot→connector_Y_neg]
```

### Link Types & Joints

| Component | Type | Joint | Purpose |
|-----------|------|-------|---------|
| **Payload** | Box (visual) + Cylinder (collision) | — | Central hub, mass 1.8 kg |
| **Tendons** | Box (visual) | Fixed | Show cable routing (no physics) |
| **Connectors** | Cylinder | Revolute ±0.3 rad | Chevron holders, mass 0.12 kg each |
| **Wheels** | Cylinder hub + box spokes | Continuous | Drive motors (optional Y-axis also) |
| **Pivots** | Sphere | Revolute ±0.8 rad | Flexible joints, allow frame bending |
| **FG Segments** | Box | Fixed (to pivots) | Connect structure, visual cables |

### Key Dimensions (from Paper)
```
R_FRAME   = 4.0 / (2π) = 0.6366 m
L1_ROVER  = 2 × R = 1.2732 m  (X-axis, long side)
L2_ROVER  = L1 / 2.2 = 0.5787 m  (Y-axis, short side, aspect ratio)
POS_X     = L1 / 2 = 0.6366 m  (connector offset from origin)
POS_Y     = L2 / 2 = 0.2894 m  (connector offset from origin)
WHEEL_R   = 0.25 m  (rimless wheel spokes length)
TRACK_WIDTH = L2 = 0.5787 m  (distance between wheel pairs)
```

### Mass Budget (Table 1, Paper)
| Component | Count | Unit Mass | Total |
|-----------|-------|-----------|-------|
| Payload (hub) | 1 | 1.80 kg | 1.80 kg |
| Connectors | 4 | 0.12 kg | 0.48 kg |
| Pivots | 4 | 0.02 kg | 0.08 kg |
| Wheels | 4 | 0.11 kg | 0.44 kg |
| **Total** | | | **2.80 kg** |

---

## Python Code Structure

### Main File: `goat_rover_sim.py`

#### Global Constants (Tunable)
```python
# Geometric
R_FRAME = 0.6366 m
ASPECT_RATIO = 2.2  # r = l2/l1
L1_ROVER = 1.2732 m  (long axis)
L2_ROVER = 0.5787 m  (short axis)
POS_X = 0.6366 m
POS_Y = 0.2894 m

# Dynamics - Cable Tendons
TENDON_K    = 2000.0 N/m      # Spring constant (tuned for dt=0.002s)
TENDON_DAMP = 80.0 N·s/m      # Damping (viscous)

# Dynamics - Frame Compliance
PIVOT_KP = 1.5 N·m/rad        # Spring (was 3.0 in v1, reduced for more flex)
PIVOT_KV = 0.3 N·m·s/rad      # Damping

# Motor Control
Motor torque limit = 4.41 N·m (paper, 45 kg·cm)
Motor KP = 1500.0              # Proportional gain
Motor KV = 30.0                # Derivative gain
```

#### Class: `GOATRover`

**Initialization & DOF Setup**
```python
GOATRover.__init__(entity)
  ├─ _init_dofs()          # Cache wheel DOF indices
  └─ set_solver(scene)     # Link to Genesis RigidSolver for force application
```

**Methods:**

| Method | Purpose | Parameters |
|--------|---------|------------|
| `configure_gains()` | Set motor PID + torque limits | None |
| `skid_steer(v_lin, v_ang, track)` | General skid-steer drive (eq. 3-4, paper) | velocities (m/s, rad/s) |
| `skid_steer_x_only(v_lin, v_ang)` | Gap scenario: only X wheels active | velocities |
| `stop()` | Halt all wheels | None |
| `apply_tendon_forces(l1, l2)` | Main cable deformation physics | Cable lengths (m) |
| `set_solver(scene)` | Establish force-application linkage | Genesis scene |

#### Tendon Force Physics: `apply_tendon_forces(l1, l2)`

**Algorithm:**
1. For each of 4 connectors (X_pos, X_neg, Y_pos, Y_neg):
   - Get connector position `c_pos`
   - Get payload position `p_pos`
   - Compute distance: `dist = ||p_pos - c_pos||`
   - Reference cable length: `l_half = l / 2`  (each side of connector)
   - Stretch: `stretch = dist - l_half`
   - If `stretch > 0`:
     - Force magnitude: `F = k * stretch + d * v_axial`
     - Direction: `unit = (p_pos - c_pos) / dist`
     - Apply force on connector: `+F × unit`
     - Apply reaction on payload: `-F × unit` (Newton III)

2. **Pivot Compliance** (additional loop):
   - For each pivot and its parent connectors:
   - Compute deviation from rest position
   - Apply restoring force: `F = K_pivot * delta`

**Example Call:**
```python
goat.apply_tendon_forces(l1=1.2732, l2=0.5787)  # Rover config
```

#### Scenario Management

**Scenario Functions:**

```python
scenario_straight(scene, goat)
  - Duration: 10 seconds
  - Command: constant v_lin=0.15 m/s, v_ang=0
  - Morphology: L1=1.2732, L2=0.5787 (rover)
  - Wheels: skid_steer (all 4 active)

scenario_turn(scene, goat)
  - Radius-controlled turning
  - Command: v_lin=0.1, v_ang=0.5 rad/s
  - Same morphology
  - Tests angular response

scenario_gap(scene, goat)  [NEW]
  - Obstacle: 2 walls creating 50cm corridor
  - Robot width: 57.87cm → MUST compress to pass
  - Command: v_lin=0.08 m/s (slow), v_ang=0
  - Wheels: skid_steer_x_only (Y wheels passive)
  - Tendons: stay at rover config initially, then compress dynamically
  - Expected: Front compresses (contact force), back bulges
  - Paper reference: Fig. 3B

scenario_aspect_ratio(scene, goat)
  - Sweeps L1 across range [rover → circle → sphere]
  - Demonstrates morphing flexibility
```

### Scene Setup: `run_simulation(scenario_name, ...)`

```python
def run_simulation(scenario_name="straight", no_viewer=False, backend="cpu"):
    1. Create Genesis scene
       scene.add_entity(GOATRover from URDF)
    
    2. Configure robot DOFs & gains
       goat.configure_gains()
    
    3. Set up physics
       - TimeStep: dt = 0.002 s (500 Hz)
       - Gravity: g = 9.81 m/s²
       - Ground friction: μ_s ≈ 0.8
    
    4. Execute scenario for N timesteps
       Each step:
         a) Read scenario command (v_lin, v_ang, l1, l2)
         b) Move wheels: goat.skid_steer(...)
         c) Apply cables: goat.apply_tendon_forces(...)
         d) Physics step: scene.step()
         e) Log state: [t, x, y, θ, vx, vy, ω, l1, l2]
         f) Render: scene.render() if viewer enabled
    
    5. Return logs (pandas DataFrame)
```

### Main Entry Point

```bash
python goat_rover_sim.py [OPTIONS]

Options:
  --scenario {straight|turn|gap|aspect_ratio}  Default: run all 4
  --no-viewer                                  Headless (faster)
  --backend {cpu|cuda}                         GPU acceleration
```

---

## Physics Model

### 1. Wheel Locomotion (Skid-Steer)

**Paper Equations (Eq. 3-4):**

For track width `d = L2`:
```
v_right  = (v_lin + d/2 * v_ang) / R_wheel
v_left   = (v_lin - d/2 * v_ang) / R_wheel
```

Where:
- `v_lin` = forward velocity (m/s)
- `v_ang` = angular velocity (rad/s)
- `d/2 = L2/2 = 0.2894 m`
- `R_wheel = 0.25 m`

**Code Implementation:**
```python
def skid_steer(self, v_lin, v_ang, track=TRACK_WIDTH):
    d2 = track / 2.0
    vR = (v_lin + d2 * v_ang) / WHEEL_R  # Right wheels
    vL = (v_lin - d2 * v_ang) / WHEEL_R  # Left wheels
    v_side = v_lin / WHEEL_R              # Front/Back wheels (passive or active)
    self.entity.control_dofs_velocity(
        np.array([vR, vL, v_side, v_side]),
        dofs_idx_local=self.dofs_wheels)
```

### 2. Tendon Cable Forces

**Cable Mechanics:**

Each tendon is a **closed loop** connecting two opposite connectors through the payload:
```
Tendon 1 (X-axis):
  payload ──winch── connector_X_pos ──cable── connector_X_neg ──── payload
  
  Reference length: l1 = 1.2732 m
  Each connector-to-payload segment: l1/2 = 0.6366 m
```

**Force Calculation per Connector:**

```python
# For connector at position c_pos, payload at p_pos
delta = p_pos - c_pos
dist = ||delta||
l_ref = l1/2  (reference half-length)
stretch = max(0, dist - l_ref)

# Tension-only (cables can't push)
F_mag = TENDON_K * stretch + TENDON_DAMP * v_dot
F_vec = F_mag * (delta / dist)

# Apply to connector (pulling toward payload)
apply_force(connector_idx, +F_vec)
apply_force(payload_idx, -F_vec)
```

**Parameters:**
- `TENDON_K = 2000 N/m` — Spring constant (tuned for stability with dt=0.002s)
- `TENDON_DAMP = 80 N·s/m` — Viscous damping
- Material: PE cable 0.6mm diameter (paper)

**Range of Motion:**
- Shortest: `l1 = 0.05 m` → sphere mode (highly compressed)
- Rover: `l1 = 1.2732 m` → typical elongated state
- Circle: `l1 = l2 = 0.5787 m` → r = 1

### 3. Pivot Compliance Modeling

**Physical Model:**

Pivots are flexible fiberglass joints. Each pivot connects 2 connectors via segments.

**Spring-Back Force:**

```python
# For each pivot-connector pair:
# Rest position = URDF nominal position
# Current position = actual position
# Deviation = current - rest
# Restoring force = K_pivot * deviation

K_pivot = 500 N/m  # stiffness of frame elastic linkage
```

**Purpose in Simulation:**

- Prevents structural collapse
- Allows frame to bend under contact forces (gap scenario)
- Models fiberglass compliance: EI/L ≈ 2.76 N·m/rad

### 4. Contact & Deformation (Gap Scenario)

**Obstacle Setup:**

```python
scene.add_entity(...)  # Two walls
p_wall_left = (-0.25, ...)
p_wall_right = (+0.25, ...)
gap_width = 0.50 m

robot_width = L2_ROVER = 0.5787 m  → EXCEEDS gap, must compress!
```

**Deformation Physics:**

When FG segments collide with walls:
1. Genesis collision detector fires
2. Contact forces applied to segments
3. Segments pull back via pivot stiffness
4. Payload receives reaction forces
5. Cables re-tension to compress the ring

**Expected Behavior (Paper Fig. 3B):**
- Front half (Y_pos) compresses forward
- Back half (Y_neg) bulges backward
- Side halves (X_pos/X_neg) remain stiff
- Result: Narrow ellipse that fits gap

---

## Simulation Scenarios

### 1. STRAIGHT
```
Duration: 10 s
Morphology: Rover (L1=1.2732, L2=0.5787)
Command: v_lin=0.15 m/s, v_ang=0
Expected: Straight forward motion
Data logged: position (x,y), heading (θ), velocities, tendon lengths
```

### 2. TURN
```
Duration: 10 s
Morphology: Rover
Command: v_lin=0.1 m/s, v_ang=0.5 rad/s
Expected: Circular path with radius R = v_lin / v_ang ≈ 0.2 m
Tests: Angular stability, skid-steer accuracy
```

### 3. GAP (NEW - Figure 3B)
```
Duration: 15 s
Morphology: Initially rover, auto-compresses on contact
Obstacle: 50cm corridor (robot native width=57.8cm)
Command: v_lin=0.08 m/s, v_ang=0 (slow, X-wheels only)
Wheels: skid_steer_x_only  (Y wheels don't spin)
Expected:
  - Robot approaches gap at ~2m/s equivalent (0.08 m/s forward)
  - Front contacts right wall → contact force
  - Cable tension increases → L2 decreases
  - Robot compresses to L2 ≈ 0.45m (fits inside 0.50m gap)
  - Rear bulges out past 0.45m as front half is pulled inward
  - Passes through, re-expands on other side
```

### 4. ASPECT_RATIO
```
Duration: 20 s
Morphology: Sweep L1 from 0.3→1.2→0.5 m (Sphere→Rover→Circle)
Command: Minimal (v_lin=0.02 m/s, or stationary)
Purpose: Demonstrate morphing capability without locomotion
Expected: Visual deformation, shows cable range
```

---

## Parameters & Tuning

### Critical Parameters

#### Motor Control
```python
WHEEL_KP = 1500.0      # Proportional gain (higher = stiffer)
WHEEL_KV = 30.0        # Derivative gain (higher = less overshoot)
WHEEL_TORQUE_MAX = 20.0 N·m  # Saturation (paper 4.41 N·m)
```

**Effect:**
- High KP: Fast response but oscillation risk
- High KV: Damping, reduces overshoot

#### Cable Dynamics
```python
TENDON_K = 2000.0 N/m     # Spring stiffness
TENDON_DAMP = 80.0 N·s/m  # Damping coefficient
```

**Effect on Gap Scenario:**
- Higher K → faster compression, stiffer response
- Higher DAMP → smoother deformation, slower

**Tuning Strategy:**
1. Start with paper values
2. If deformation too fast (oscillates): increase TENDON_DAMP
3. If deformation too slow: increase TENDON_K
4. If robot overshoots gap: decrease WHEEL_KP or increase WHEEL_KV

#### Frame Compliance
```python
PIVOT_KP = 1.5 N·m/rad     # Spring (was 3.0 in v1)
PIVOT_KV = 0.3 N·m·s/rad   # Damping
K_PIVOT = 500.0 N/m        # Segment-connector spring
```

**Effect:**
- Lower PIVOT_KP → frame bends more (good for gap)
- Higher PIVOT_KP → frame stays rigid (good for straight line)

### Configuration Pressets

```python
# CIRCLE (best turning)
L1_CIRCLE = 0.6366 m
L2_CIRCLE = 0.6366 m
# Aspect ratio r = L2/L1 = 1.0
# Turning radius = L2/g ≈ 0.065m (tightest)

# ROVER (fastest forward)
L1_ROVER = 1.2732 m
L2_ROVER = 0.5787 m
# Aspect ratio r = 2.2
# Paper: "optimal speed 1.4 m/s at r=2.0-3.0"

# SPHERE (compact)
L1_SPHERE = 0.05 m
L2_SPHERE = 0.05 m
# Highly compressed, minimal footprint

# FOLDING (out-of-plane)
L1_FOLDING = 0.10 m
L2_FOLDING = 0.10 m
# Intermediate, shows 3D deformation
```

---

## Common Modifications

### Modify Wheel Speed Limits

**File:** `goat_rover_sim.py`, line ~150

```python
# BEFORE
self.entity.set_dofs_force_range(
    lower=np.full(4, -20.0),  
    upper=np.full(4,  20.0),
    dofs_idx_local=self.dofs_wheels)

# AFTER (increase to 30 N·m)
    lower=np.full(4, -30.0),  
    upper=np.full(4,  30.0),
```

### Add New Scenario

**Template:**

```python
def scenario_custom(scene, goat):
    """Custom scenario: [describe here]"""
    n_steps = int(10.0 / dt)  # 10 seconds
    logs = []
    
    for i in range(n_steps):
        # 1. Compute desired morphology
        t = i * dt
        l1 = 1.2732 + 0.5 * np.sin(t)  # sinusoidal morphing
        l2 = 0.5787
        
        # 2. Compute desired motion
        if t < 5:
            v_lin = 0.1
            v_ang = 0
        else:
            v_lin = 0
            v_ang = 0.2
        
        # 3. Control robot
        goat.skid_steer(v_lin, v_ang)
        goat.apply_tendon_forces(l1, l2)
        
        # 4. Physics step
        scene.step()
        
        # 5. Log data
        logs.append({...})
    
    return logs

# Add to main:
if scenario_name == "custom":
    logs = scenario_custom(scene, goat)
```

### Change URDF Geometry

**File:** `goat_rover_2.urdf`

**Example: Increase Wheel Radius**

```xml
<!-- BEFORE: radius="0.05" -->
<link name="wheel_X_pos">
    <visual>
        <origin xyz="0 0 0" rpy="0 1.5708 0"/>
        <geometry><cylinder radius="0.05" length="0.06"/></geometry>

<!-- AFTER: radius="0.08" (80mm wheels) -->
        <geometry><cylinder radius="0.08" length="0.06"/></geometry>
```

**Also update Python:**
```python
# goat_rover_sim.py, line ~45
WHEEL_R = 0.08  # Changed from 0.25
```

### Adjust Motor Torque / Speed Trade-off

**Paper Motor Spec:** 45 kg·cm = 4.41 N·m, ~300 RPM

**To make faster (lower torque limit):**
```python
WHEEL_TORQUE_MAX = 10.0  # N·m → faster wheels, less torque
WHEEL_KP = 500.0         # Lower stiffness
```

**To make stronger (higher torque limit):**
```python
WHEEL_TORQUE_MAX = 50.0  # N·m → slower wheels, more torque
WHEEL_KP = 2000.0        # Higher stiffness
```

---

## Troubleshooting

### Issue: "RigidSolver not found"
**Cause:** Genesis scene not initialized properly  
**Fix:** Ensure `goat.set_solver(scene)` is called after adding robot to scene

```python
# Correct order:
scene = gs.Scene()
scene.add_entity(...)  # Load robot
goat = GOATRover(robot_entity)
goat.set_solver(scene)  # MUST be after add_entity
```

### Issue: Robot "explodes" or vibrates wildly
**Cause:** Time step too large or gains too high  
**Fix:**
1. Reduce time step: `dt = 0.001` (from 0.002)
2. Reduce motor gain: `WHEEL_KP = 800` (from 1500)
3. Increase damping: `WHEEL_KV = 50` (from 30)

### Issue: Robot refuses to compress in gap
**Cause:** Cables not stiff enough or wrong contact friction  
**Fix:**
1. Increase cable stiffness: `TENDON_K = 3000` (from 2000)
2. Ensure wall collision enabled: `scene.add_entity(walls, collision=True)`
3. Check gap width: must be < robot native width (0.5787m)

### Issue: Cables snap / extreme tension
**Cause:** Reference length `l_ref` unreachable, or huge velocities  
**Fix:**
1. Check `apply_tendon_forces` parameters: `l1`, `l2` within valid range
2. Add velocity saturation: `max_vel = 2.0 m/s`
3. Reduce motor KP to slow acceleration

### Issue: "Connector link not found"
**Cause:** URDF link name mismatch  
**Fix:** Check URDF spelling:
```python
# Code expects:
"connector_X_pos", "connector_X_neg", "connector_Y_pos", "connector_Y_neg"
"wheel_joint_X_pos", "wheel_joint_X_neg", etc.

# Verify in URDF:
<link name="connector_X_pos"> ... </link>
<joint name="wheel_joint_X_pos" ...> ... </joint>
```

### Issue: Simulation slow / runs at 5% speed
**Cause:** High accuracy physics or CPU bottleneck  
**Fix:**
1. Enable GPU: `python goat_rover_sim.py --backend cuda`
2. Reduce solver accuracy: `scene.set_timestep(0.005)` (larger dt)
3. Run headless: `python goat_rover_sim.py --no-viewer`

### Issue: Viewer black/doesn't show
**Cause:** Graphics not initialized or running headless  
**Fix:**
```python
# In code:
if not no_viewer:
    scene.render()

# Or from command line:
python goat_rover_sim.py  # Default includes viewer
```

---

## Advanced Topics

### Custom Contact Callbacks

**If you need to respond to specific collision events:**

```python
def on_contact_callback(contact_info):
    """Called when collision detected"""
    link_a, link_b = contact_info
    # Example: reduce robot L2 if front region touches something
    if "seg_XpYp" in [link_a, link_b]:
        goat.apply_tendon_forces(l1=1.2, l2=0.30)  # Compress

scene.set_contact_callback(on_contact_callback)
```

### Logging & Analysis

**Default output:** Pandas DataFrame with columns:
```
[time, x, y, theta, vx, vy, omega, l1, l2, wheel_R_vel, ...]
```

**Save to CSV:**
```python
import pandas as pd
df = pd.DataFrame(logs)
df.to_csv("rover_gap_test.csv", index=False)
```

**Plot results:**
```python
import matplotlib.pyplot as plt
plt.plot(df['time'], df['x'], label="X position")
plt.plot(df['time'], df['y'], label="Y position")
plt.legend()
plt.show()
```

### Multi-Robot Simulation

**Add multiple GOAT instances:**

```python
scene = gs.Scene()
goat1 = scene.add_entity(GOATRover(urdf="goat_rover_2.urdf"))
goat2 = scene.add_entity(GOATRover(urdf="goat_rover_2.urdf"))
goat2.set_pos([2.0, 0, 0.25])  # Offset initial position

# Control independently
for step in range(n_steps):
    goat1.skid_steer(v_lin=0.1, v_ang=0)
    goat2.skid_steer(v_lin=0.05, v_ang=0.2)
    # ... apply forces, step, etc.
```

---

## Summary Table: Files & Functions

| File | Key Components | Purpose |
|------|---|---|
| `goat_rover_sim.py` | `GOATRover` class, scenarios, main | Control & physics simulation |
| `goat_rover_2.urdf` | Links, joints, geometry | Model definition |
| `goat_sim.py` | Earlier version (deprecated) | Reference only |
| `test_urdf.py` | URDF diagnostic | Debug link loading issues |

| Function | Input | Output | Purpose |
|----------|-------|--------|---------|
| `apply_tendon_forces(l1, l2)` | Cable lengths (m) | Force vectors | Deformation via cables |
| `skid_steer(v_lin, v_ang)` | Velocities (m/s, rad/s) | Wheel speeds | Locomotive control |
| `configure_gains()` | None | DOF limits set | Motor parameter setup |
| `set_solver(scene)` | Genesis scene | Force API linked | Enable force application |

---

