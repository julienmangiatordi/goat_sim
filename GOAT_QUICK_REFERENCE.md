# GOAT Simulation — Quick Reference Card

**For rapid prompting of Claude AI | Print-friendly**

---

## Files Quick Map

```
/home/julien/Documents/Goat/Genesis/goat/
├── goat_rover_sim.py          ← MAIN: Robot control + scenarios
├── goat_rover_2.urdf          ← MAIN: Physics model definition
├── goat_rover.urdf            ← Alt version
├── goat_v5_soft_joints.urdf   ← Earlier (cables visible)
└── test_urdf.py               ← Debug URDF loading
```

---

## Launch Commands

```bash
# Standard (all scenarios, with viewer)
python goat_rover_sim.py

# One scenario only
python goat_rover_sim.py --scenario gap
python goat_rover_sim.py --scenario straight
python goat_rover_sim.py --scenario turn
python goat_rover_sim.py --scenario aspect_ratio

# Headless (faster)
python goat_rover_sim.py --scenario gap --no-viewer

# GPU acceleration
python goat_rover_sim.py --backend cuda

# Combine all
python goat_rover_sim.py --scenario gap --no-viewer --backend cuda
```

---

## Key Parameters (goat_rover_sim.py)

### Geometry
```python
Line ~35:   WHEEL_R = 0.25          # Wheel radius (m)
Line ~30:   L1_ROVER = 1.2732       # Cable length X-axis (m)
Line ~31:   L2_ROVER = 0.5787       # Cable length Y-axis (m)
Line ~32:   TRACK_WIDTH = 0.5787    # Wheelbase (m)
```

### Cable Dynamics (Tendon Physics)
```python
Line ~40:   TENDON_K = 2000.0       # Cable spring (N/m)
Line ~41:   TENDON_DAMP = 80.0      # Cable viscous (N·s/m)
```

### Frame Compliance (Pivot Flexing)
```python
Line ~44:   PIVOT_KP = 1.5          # Pivot spring (N·m/rad)
Line ~45:   PIVOT_KV = 0.3          # Pivot damping (N·m·s/rad)
```

### Motor Control
```python
Line ~125:  WHEEL_KP = 1500.0       # Motor stiffness
Line ~126:  WHEEL_KV = 30.0         # Motor damping
Line ~127:  WHEEL_TORQUE_MAX = 20.0 # Torque limit (N·m)
```

---

## Tuning Guide

### Robot Too Slow?
1. Increase `WHEEL_KP` from 1500 → 2000
2. Increase `WHEEL_TORQUE_MAX` from 20 → 30 N·m
3. Check wheel radius: `WHEEL_R` (was 0.25m)

### Robot Vibrating/Oscillating?
1. Increase `WHEEL_KV` from 30 → 50
2. Increase `TENDON_DAMP` from 80 → 120
3. Reduce `WHEEL_KP` from 1500 → 800

### Gap Deformation Too Slow?
1. Increase `TENDON_K` from 2000 → 3000 N/m
2. Reduce `PIVOT_KP` from 1.5 → 0.8 (allow more bending)

### Gap Deformation Too Fast/Jerky?
1. Increase `TENDON_DAMP` from 80 → 150
2. Increase `PIVOT_KV` from 0.3 → 0.7
3. Reduce `TENDON_K` from 2000 → 1200

---

## Morphology Presets

```python
# CIRCLE (best turning)
L1_CIRCLE = 0.6366      # r = 1.0
L2_CIRCLE = 0.6366

# ROVER (fastest forward) ← DEFAULT
L1_ROVER = 1.2732       # r = 2.2
L2_ROVER = 0.5787

# SPHERE (compact)
L1_SPHERE = 0.05        # r = 0.05
L2_SPHERE = 0.05

# CUSTOM
L1 = 0.8
L2 = 0.4
# Then call: goat.apply_tendon_forces(L1, L2)
```

---

## Main Code Structure

### Initialization
```python
scene = gs.Scene()                          # Create physics world
entity = scene.add_entity(...)              # Load URDF robot
goat = GOATRover(entity)                    # Wrap in control class
goat.configure_gains()                      # Set motor params
goat.set_solver(scene)                      # Link force API
```

### Control Loop (each timestep)
```python
l1 = 1.2732                                 # Target cable lengths
l2 = 0.5787
v_lin = 0.1                                 # Linear velocity (m/s)
v_ang = 0.0                                 # Angular velocity (rad/s)

goat.skid_steer(v_lin, v_ang)              # Set wheel speeds
goat.apply_tendon_forces(l1, l2)           # Apply cable forces
scene.step()                                # Physics step (dt=0.002s)
```

---

## URDF Structure (goat_rover_2.urdf)

### Main Components
```
base_link (PAYLOAD 1.8 kg)
├── connector_X_pos (0.12 kg) → wheel_X_pos
├── connector_X_neg (0.12 kg) → wheel_X_neg
├── connector_Y_pos (0.12 kg) → wheel_Y_pos
├── connector_Y_neg (0.12 kg) → wheel_Y_neg
├── pivot_XpYp (0.02 kg)     → segments
├── pivot_XpYn (0.02 kg)     → segments
├── pivot_XnYp (0.02 kg)     → segments
└── pivot_XnYn (0.02 kg)     → segments
```

### Link Names (for Python access)
```python
e.get_link("base_link")              # Payload/hub
e.get_link("connector_X_pos")        # Right chevron
e.get_link("connector_X_neg")        # Left chevron
e.get_link("connector_Y_pos")        # Front chevron
e.get_link("connector_Y_neg")        # Back chevron
e.get_link("wheel_X_pos")            # Right wheel
e.get_link("wheel_X_neg")            # Left wheel
e.get_link("wheel_Y_pos")            # Front wheel
e.get_link("wheel_Y_neg")            # Back wheel
e.get_link("pivot_XpYp")             # Corner 1 pivot
# ... (4 pivots total)
```

### Joint Names (for DOF access)
```python
e.get_joint("wheel_joint_X_pos")     # Right motor
e.get_joint("wheel_joint_X_neg")     # Left motor
e.get_joint("wheel_joint_Y_pos")     # Front motor
e.get_joint("wheel_joint_Y_neg")     # Back motor
e.get_joint("joint_pivot_XpYp")      # Corner 1 flex
# ... (4 pivots total)
```

---

## Scenario Parameters

### STRAIGHT
```python
Duration: 10 s
Morphology: ROVER (1.2732, 0.5787)
Command: v_lin=0.15, v_ang=0
Expected: Forward motion, minimal deviation
```

### TURN
```python
Duration: 10 s
Morphology: ROVER
Command: v_lin=0.1, v_ang=0.5 rad/s
Expected: Circular path, radius ≈ 0.2 m
```

### GAP
```python
Duration: 15 s
Morphology: ROVER → auto-compresses on contact
Obstacle: 50 cm corridor (robot 57.8 cm wide)
Command: v_lin=0.08 m/s, v_ang=0
Wheels: X-only (Y passive)
Expected: Compress, pass through, re-expand
Paper ref: Fig. 3B
```

### ASPECT_RATIO
```python
Duration: 20 s
Morphology: Sweep L1 from 0.3 → 1.2 → 0.5 m
Command: v_lin=0.02 or 0
Expected: Shape deformation visualization
```

---

## Debugging Commands

### Check URDF Loads
```bash
python test_urdf.py
# Expected output: Lists all 17 links with positions
```

### Print Available Links
```python
from genesis import *
import numpy as np

scene = Scene()
entity = scene.add_entity(...)
goat = GOATRover(entity)

for i, link in enumerate(entity.get_links()):
    print(f"[{i}] {link.name} pos={link.get_pos()}")
```

### Log Tendon State
```python
# Inside control loop:
print(f"L1={l1:.4f}, L2={l2:.4f}, "
      f"pos_payload={goat_entity.get_link('base_link').get_pos()}")
```

---

## Common Edits for Claude AI

### Change Wheel Size
**File:** `goat_rover_sim.py` line 35  
**Also:** `goat_rover_2.urdf` line ~280 (in `<wheel_*>` links)
```python
WHEEL_R = 0.25          # Change this
```

### Change Cable Stiffness
**File:** `goat_rover_sim.py` line 40
```python
TENDON_K = 2000.0       # Increase for stiffer cables
```

### Change Motor Strength
**File:** `goat_rover_sim.py` line 127
```python
WHEEL_TORQUE_MAX = 20.0 # Increase for stronger motors
```

### Add New Scenario
**File:** `goat_rover_sim.py` (add function ~line 200)
```python
def scenario_custom(scene, goat):
    # Your code here
    # Must return logs list
```
Then add to main scenario dispatcher.

### Modify Gap Width
**File:** `goat_rover_sim.py` (in `scenario_gap` function)
```python
p_wall_L = (-0.25, ...)  # Move walls
p_wall_R = (+0.25, ...)
```

---

## Error Messages & Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| "RigidSolver not found" | set_solver() called too early | Call after `add_entity()` |
| "Link '...' not found" | Wrong link name in code | Check URDF: `<link name="...">` |
| "Module genesis not found" | Genesis not installed | `conda activate genesis` |
| "Robot explodes" | Time step too large | Reduce `dt` from 0.002 → 0.001 |
| "Too slow" | Motor gains too low | Increase `WHEEL_KP`, `TENDON_K` |
| "Won't compress" | Contact disabled or K too low | Increase `TENDON_K`, check collision |

---

## Physics Constants (From Paper)

```
Motor torque:       4.41 N·m (45 kg·cm)
Winch torque:       3.92 N·m (40 kg·cm)
Wheel radius:       0.25 m (FG spokes 8mm long)
Cable material:     PE 0.6 mm diameter
Frame material:     Fiberglass tubes 5mm ø
Total mass:         2.80 kg
  - Payload:        1.80 kg
  - 4 Connectors:   0.48 kg (0.12 each)
  - 4 Wheels:       0.44 kg (0.11 each)
  - 4 Pivots:       0.08 kg (0.02 each)

Optimal speed:      1.4 m/s (at aspect ratio r=2.0-3.0)
Optimal turning:    r=1.0 (circle mode)

Paper scenario:
  Fig. 2A: Gap passing (compression)
  Fig. 3B: Gap transit (shows deformation)
  Fig. 4C: Turning vs aspect ratio curves
```

---

## Data Analysis Template

```python
import pandas as pd
import matplotlib.pyplot as plt

# After simulation:
df = pd.DataFrame(logs)
df.to_csv("robot_data.csv")

# Plot trajectories
fig, axes = plt.subplots(2, 2)

axes[0, 0].plot(df['x'], df['y'])
axes[0, 0].set_xlabel("X (m)")
axes[0, 0].set_ylabel("Y (m)")
axes[0, 0].set_title("Trajectory")

axes[0, 1].plot(df['time'], df['vx'])
axes[0, 1].set_title("Velocity (X)")

axes[1, 0].plot(df['time'], df['l1'], label="L1")
axes[1, 0].plot(df['time'], df['l2'], label="L2")
axes[1, 0].set_title("Morphology")
axes[1, 0].legend()

axes[1, 1].plot(df['time'], df['theta'])
axes[1, 1].set_title("Heading")

plt.tight_layout()
plt.show()
```

---

## Tip: Asking Claude AI

**Good prompt:**
> "In goat_rover_sim.py line 40, change TENDON_K from 2000 to 3000 because the gap deformation is too slow. After this change, the robot should compress faster when hitting walls. I'll test with `--scenario gap`"

**Bad prompt:**
> "Make the simulation go faster"

---

**Version:** 1.0 | Quick Reference for GOAT | April 2026
