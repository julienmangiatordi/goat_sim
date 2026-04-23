# GOAT Simulation Documentation Index

**Navigation Guide for All GOAT Documentation**  
**Created:** April 2026  
**For:** Claude AI Navigator Assistant

---

## Quick Navigation

### 🚀 **Just Starting?**
1. Read: [GOAT_QUICK_REFERENCE.md](GOAT_QUICK_REFERENCE.md) — 5 min overview
2. Try: `python goat_rover_sim.py --scenario straight`
3. Modify: Pick any parameter from "Key Parameters" section

### 🔧 **Want to Modify Code?**
1. Check: [GOAT_QUICK_REFERENCE.md](GOAT_QUICK_REFERENCE.md) → "Common Edits for Claude AI"
2. Reference: [GOAT_SIMULATION_GUIDE.md](GOAT_SIMULATION_GUIDE.md) section 6 (Common Modifications)
3. Context: Use exact file names and line numbers when asking Claude

### 📐 **Need to Understand Physics?**
1. Start: [GOAT_SIMULATION_GUIDE.md](GOAT_SIMULATION_GUIDE.md) section 4 (Physics Model)
2. Deep dive: [GOAT_URDF_TECHNICAL_REFERENCE.md](GOAT_URDF_TECHNICAL_REFERENCE.md) (inertia, forces)
3. Paper reference: Polzin et al., Sci. Robot. 10, eadp6419 (2025)

### 🎯 **Debugging a Problem?**
1. [GOAT_SIMULATION_GUIDE.md](GOAT_SIMULATION_GUIDE.md) section 8 (Troubleshooting)
2. [GOAT_QUICK_REFERENCE.md](GOAT_QUICK_REFERENCE.md) bottom table (Error Messages & Fixes)

### 🏗️ **Understanding URDF?**
→ [GOAT_URDF_TECHNICAL_REFERENCE.md](GOAT_URDF_TECHNICAL_REFERENCE.md) (comprehensive breakdown)

---

## Document Map

```
├─ THIS FILE (Index)
│
├─ GOAT_QUICK_REFERENCE.md
│  ├─ File locations & launch commands
│  ├─ Parameter line numbers & tuning
│  ├─ Scenario specs
│  ├─ Code snippets (copy-paste ready)
│  ├─ Error lookup table
│  └─ "Perfect for: Quick lookups, copy-paste modifications"
│
├─ GOAT_SIMULATION_GUIDE.md (MAIN)
│  ├─ Complete architecture overview
│  ├─ URDF components & hierarchy
│  ├─ Python classes & methods
│  ├─ Physics equations & models
│  ├─ All 4 scenarios detailed
│  ├─ Parameter tuning guide
│  ├─ Common modifications (templates)
│  ├─ Troubleshooting & debugging
│  └─ "Perfect for: Learning the system, understanding physics"
│
├─ GOAT_URDF_TECHNICAL_REFERENCE.md
│  ├─ Line-by-line XML breakdown
│  ├─ Inertia calculations
│  ├─ Joint limits & forces
│  ├─ Coordinate system
│  ├─ Mass verification
│  ├─ Material definitions
│  ├─ Typical URDF load output
│  └─ "Perfect for: Modifying robot geometry, deep physics understanding"
│
└─ Associated source files:
   ├─ goat_rover_sim.py (main simulator)
   ├─ goat_rover_2.urdf (current model)
   ├─ goat_v5_soft_joints.urdf (alt version)
   ├─ goat_sim.py (deprecated)
   └─ test_urdf.py (diagnostics)
```

---

## Document Selection by Task

| What You Want | Read This | Lines | Time |
|---|---|---|---|
| **Quick parameter lookup** | Quick Reference → "Key Parameters" | — | 2 min |
| **Launch simulation** | Quick Reference → "Launch Commands" | — | 1 min |
| **Tune cable stiffness** | Quick Reference → "Tuning Guide" → SIMULATION_GUIDE §6 | 40, 80-110 | 5 min |
| **Add new scenario** | SIMULATION_GUIDE §3 + §7 | 200-300 | 15 min |
| **Understand gap scenario** | SIMULATION_GUIDE §3 → "GAP" | — | 10 min |
| **Fix robot oscillating** | Quick Reference → Error table | — | 3 min |
| **Understand cable physics** | SIMULATION_GUIDE §4.2 | — | 10 min |
| **Modify wheel size** | Quick Reference → "Common Edits" | — | 5 min |
| **Modify URDF geometry** | URDF_REFERENCE §1-5 + Example | 400+ | 20 min |
| **Calculate inertia** | URDF_REFERENCE §4.4 formula | — | 5 min |
| **Debug link not found error** | SIMULATION_GUIDE §8 + URDF_REFERENCE | — | 10 min |
| **Understand skid-steer physics** | SIMULATION_GUIDE §4.1 (equations) | — | 10 min |
| **Learn entire system** | All 3 documents in order | — | 1-2 hours |

---

## Key Sections by Topic

### Cable/Tendon Dynamics
- **Overview:** SIMULATION_GUIDE §4.2 (conceptual)
- **Equations:** SIMULATION_GUIDE §4.2 with code snippet
- **Parameters:** Quick Reference "Cable Dynamics" + SIMULATION_GUIDE §6
- **Implementation:** See `apply_tendon_forces()` in goat_rover_sim.py lines 155-200

### Wheel Locomotion
- **Theory:** SIMULATION_GUIDE §4.1 (skid-steer equations)
- **Code:** SIMULATION_GUIDE §3 classes `skid_steer()` and `skid_steer_x_only()`
- **Tuning:** Quick Reference "Tuning Guide"

### Robot Geometry
- **Overview:** SIMULATION_GUIDE §2 (component hierarchy)
- **Details:** URDF_REFERENCE §1-5 (every link/joint)
- **Coordinates:** URDF_REFERENCE §7
- **Mass budget:** URDF_REFERENCE §6

### Scenarios
- **All scenarios:** SIMULATION_GUIDE §3
- **Details per scenario:** QUICK_REFERENCE bottom table
- **Paper references:** SIMULATION_GUIDE §5

### Debugging
- **Step-by-step:** SIMULATION_GUIDE §8
- **Quick lookup:** Quick Reference bottom table

---

## File Locations

```
/home/julien/Documents/Goat/
├─ GOAT_SIMULATION_GUIDE.md          ← Full technical reference
├─ GOAT_QUICK_REFERENCE.md           ← Copy-paste quick lookup
├─ GOAT_URDF_TECHNICAL_REFERENCE.md  ← URDF deep dive
├─ GOAT_SIMULATION_DOCUMENTATION_INDEX.md (THIS FILE)
│
└─ Genesis/goat/
   ├─ goat_rover_sim.py               ← Main simulator
   ├─ goat_rover_2.urdf               ← Active URDF model
   ├─ goat_rover.urdf                 ← Alt (older)
   ├─ goat_v5_soft_joints.urdf        ← Alt (cable-visible)
   ├─ goat_sim.py                     ← Deprecated
   └─ test_urdf.py                    ← Diagnostic tool
```

---

## How to Use These Docs with Claude AI

### Template: Initial Context Setup
```
I'm working on a GOAT robot simulation in Genesis physics engine.

Key files:
- Main simulator: goat_rover_sim.py
- Model: goat_rover_2.urdf
- Docs: GOAT_SIMULATION_GUIDE.md (read this for context)

Task: [YOUR REQUEST]

Question/Request: [DETAILED DESCRIPTION]
```

### Template: Parameter Modification
```
File to modify: goat_rover_sim.py, line 40
Current code: TENDON_K = 2000.0
Change reason: Cable deformation too slow in gap scenario
Desired change: TENDON_K = 3000.0
Expected behavior: Robot compresses faster when hitting walls

Please make this change and explain what will happen.
```

### Template: Complex Feature Addition
```
I want to add: [FEATURE]
Reference: See GOAT_SIMULATION_GUIDE.md §6 "Common Modifications" - "Add New Scenario"
Use as template: The scenario_straight() function (lines 280-300)
Desired behavior: [DESCRIPTION]

Based on existing code patterns, here's what I want to do: [YOUR APPROACH]
```

### Template: Debugging
```
Problem: Robot oscillates wildly
Last action: Changed WHEEL_KP from 1500 to 2500
Error output: [IF ANY]

Reference: GOAT_QUICK_REFERENCE.md "Robot Vibrating/Oscillating?"
Troubleshooting steps I've tried: [LIST]

What should I try next?
```

---

## Key Concepts Reference

**Terminology used in code/docs:**

| Term | Meaning | Context |
|------|---------|---------|
| **Tendon/Cable** | Force-based deformation mechanism (not real cables) | Python: `apply_tendon_forces()` |
| **Morphology** | Shape configuration (circle/rover/sphere) | Parameters: `l1`, `l2` |
| **Aspect ratio (r)** | r = L2 / L1 (short/long side ratio) | r=2.2 (rover), r=1.0 (circle) |
| **Connector** | Chevron link connecting payload to wheels | URDF: `connector_X_pos/neg/Y_pos/neg` |
| **Pivot** | Flexible corner joint (compliance point) | URDF: `pivot_XpYp/XpYn/XnYp/XnYn` |
| **FG Rod** | Fiberglass segment (visual/structural) | URDF: `seg_*` links |
| **Rimless Wheel** | Wheel model (8 spokes, no rim) | Paper design feature |
| **Skid-steer** | Differential drive (left/right wheel pair) | Kinematics: eq. 3-4 (paper) |
| **DOF** | Degree of freedom (wheel speeds, pivot angles) | Python: `dofs_idx_local` |
| **Inertia** | Rotational resistance (I_xx, I_yy, I_zz) | URDF: per link mass property |

---

## Physics Parameters Summary

### Core Constants (Immutable)
```python
R_FRAME = 0.6366 m              # Frame radius from paper
WHEEL_R = 0.25 m                # Wheel spoke length
ASPECT_RATIO = 2.2              # Rover configuration
L1_ROVER = 1.2732 m             # Long cable
L2_ROVER = 0.5787 m             # Short cable
```

### Tunable Dynamics (Solver-specific)
```python
TENDON_K = 2000 N/m             # Cable stiffness
TENDON_DAMP = 80 N·s/m          # Cable viscous damping
PIVOT_KP = 1.5 N·m/rad          # Pivot spring constant
PIVOT_KV = 0.3 N·m·s/rad        # Pivot damping
WHEEL_KP = 1500                 # Motor proportional gain
WHEEL_KV = 30                   # Motor derivative gain
WHEEL_TORQUE_MAX = 20 N·m       # Motor saturation
```

### Physics Engine Settings
```python
dt = 0.002 s                    # Timestep (500 Hz)
gravity = 9.81 m/s²            # Standard Earth gravity
friction μ ≈ 0.8                # Wheel-ground friction
```

---

## Emergency Quick Links

💥 **Robot not loading?**
→ SIMULATION_GUIDE §8 "RigidSolver not found"

💥 **Can't find cable forces?**
→ SIMULATION_GUIDE §4.2 + see `apply_tendon_forces()` line 155

💥 **URDF syntax error?**
→ Run `python test_urdf.py` then check URDF_REFERENCE §1-5

💥 **Parameter line number not in doc?**
→ Check QUICK_REFERENCE "Key Parameters" for actual line numbers

💥 **Scenario doesn't exist?**
→ Add one using template in SIMULATION_GUIDE §6 "Add New Scenario"

---

## Asking Claude AI Effectively

**Good practice:**
1. Quote the exact error message or show output
2. Reference specific document section: "SIMULATION_GUIDE §4.2"
3. Provide file name and line number
4. Explain expected vs actual behavior
5. Show current code (3 lines before/after edit location)

**Example good question:**
```
File: goat_rover_sim.py, line 125
Error: Robot vibrates in circles instead of going straight
Current: WHEEL_KP = 1500, WHEEL_KV = 30
Tried: Increasing WHEEL_KV to 50 → still vibrates
Reference: GOAT_QUICK_REFERENCE.md "Robot Vibrating" section

Should I reduce WHEEL_KP or increase damping elsewhere?
Show me the exact change to make.
```

**Example unclear question:**
```
Robot goes too slow how fix
```

---

## Document Maintenance

**Last Updated:** April 16, 2026  
**Current Simulation Version:** goat_rover_sim.py (latest)  
**Current URDF Version:** goat_rover_2.urdf (VERSION 4)  
**Paper Reference:** Polzin et al., Sci. Robot. 10, eadp6419 (2025)

**Known Limitations:**
1. Cable forces are Python-based (not URDF-native)
2. Pivots are passive (no active control in current version)
3. Wheels are simplified rimless model (visual only)
4. Gap scenario is idealized obstacle (parallel walls)

**Future Improvements (if any):**
- [ ] Tendon spool/winch simulation (motor rotation → cable length)
- [ ] Multi-body cable physics (intermediate pulley points)
- [ ] Terrain generation (not just flat + gap)
- [ ] Real-time force visualization

---

## Support & Next Steps

**If you get confused:**
1. Check this index for relevant section
2. Read the recommended document section
3. Provide full context to Claude AI (doc reference + code snippet)
4. Ask for step-by-step explanation

**To improve simulations:**
1. Read SIMULATION_GUIDE §6 (Parameter Tuning)
2. Try QUICK_REFERENCE "Tuning Guide" lookup
3. Run scenarios with different parameters: `python goat_rover_sim.py --scenario gap`
4. Compare CSV outputs (if logging enabled)

---

**Documentation Set Version:** 1.0  
**For Claude AI Navigator Assistant**  
**Questions? Refer to relevant document sections above.**
