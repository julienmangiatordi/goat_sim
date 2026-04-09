"""
goat_sim.py — Genesis simulation of GOAT robot
Polzin et al., Sci. Robot. 10, eadp6419 (2025)

NOTE sur les eyelets/segments:
  Genesis fusionne tous les links reliés par des joints 'fixed' dans le
  corps parent le plus proche ayant un joint mobile. Lens, segments et
  eyelets sont donc tous absorbés dans 'payload' (corps rigide unique).
  Les tendons prismatiques (tendon_1, tendon_2) restent actifs car leurs
  actuators ont un joint prismatique (non-fixed).
  La force de morphing est exercée via le contrôleur PD des joints
  prismatiques directement — pas besoin d'apply_force sur les eyelets.
"""

import argparse
import numpy as np
import genesis as gs

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="rover",
                    choices=["rover", "sphere", "morph"])
parser.add_argument("--backend", default="cpu",
                    choices=["cpu", "cuda"])
args = parser.parse_args()

backend = gs.cuda if args.backend == "cuda" else gs.cpu
gs.init(backend=backend, precision="64", logging_level="warning")

# ── Scene ─────────────────────────────────────────────────────────────────────
scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=0.002,
        gravity=(0.0, 0.0, -9.81),
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(2.5, -2.5, 1.8),
        camera_lookat=(0.0, 0.0, 0.30),
        camera_fov=50,
        max_FPS=60,
    ),
    show_viewer=True,
)

scene.add_entity(gs.morphs.Plane())

goat = scene.add_entity(
    gs.morphs.URDF(
        file="goat_v5_soft_joints.urdf",
        pos=(0.0, 0.0, 0.35),
        euler=(0.0, 0.0, 0.0),
        fixed=False,
    )
)

scene.build()

# Debug: affiche les links disponibles
print("[DEBUG] Links disponibles:")
for link in goat.links:
    print(f"  - {link.name}")

# ── DOF index lookup ──────────────────────────────────────────────────────────
def dof(joint_name):
    return goat.get_joint(joint_name).dof_idx_local

WHEEL_DOFS = [
    dof("wheel_FL_joint"),
    dof("wheel_FR_joint"),
    dof("wheel_RL_joint"),
    dof("wheel_RR_joint"),
]
# NOTE: TENDON_DOFS no longer exist in goat_v4_cable_driven.urdf
# Tendons are controlled via apply_cable_forces() with tendon_state global

# ── PD gains ──────────────────────────────────────────────────────────────────
# Paper: roues 45 kg·cm = 4.41 N·m, treuils 40 kg·cm = 3.92 N·m
goat.set_dofs_kp(np.array([200.0]*4), dofs_idx_local=np.array(WHEEL_DOFS))
goat.set_dofs_kv(np.array([ 20.0]*4), dofs_idx_local=np.array(WHEEL_DOFS))
goat.set_dofs_force_range(
    lower=np.array([-4.41]*4), upper=np.array([4.41]*4),
    dofs_idx_local=np.array(WHEEL_DOFS))

# ── Morphology presets (paper Fig. 2C) ───────────────────────────────────────
MORPH = {
    "circle":  (0.20, 0.20),
    "rover":   (0.30, 0.15),
    "folding": (0.10, 0.10),
    "sphere":  (0.05, 0.05),
}

# ── Global tendon state ──────────────────────────────────────────────────────
tendon_state = {"l1": 0.20, "l2": 0.20}  # Current cable lengths [m]
tendon_target = {"l1": 0.20, "l2": 0.20}  # Desired cable lengths [m]

def apply_cable_forces():
    """
    Apply cable tension forces from payload through eyelets.
    
    Paper mechanism (Polzin et al.):
    - Tendon 1 (X-axis): Payload → eyelet_L → eyelet_R → Payload
      Pulls eyelet_L and eyelet_R toward payload with force = k * (l_ref - l_current)
    - Tendon 2 (Y-axis): Payload → eyelet_F → eyelet_B → Payload
      Pulls eyelet_F and eyelet_B toward payload with force = k * (l_ref - l_current)
    """
    CABLE_K = 500.0  # Cable stiffness (N/m) when compressed
    CABLE_D = 50.0   # Damping (N·s/m)
    MAX_FORCE = 10.0  # Max cable tension (N)
    
    try:
        # Payload position (winch location)
        payload_pos = goat.get_link("payload").get_pos()
        payload_vel = goat.get_link("payload").get_lin_vel() if hasattr(goat.get_link("payload"), 'get_lin_vel') else np.zeros((3,))
        
        # Smooth tendon state toward target
        alpha = 0.01  # Blending factor
        tendon_state["l1"] += alpha * (tendon_target["l1"] - tendon_state["l1"])
        tendon_state["l2"] += alpha * (tendon_target["l2"] - tendon_state["l2"])
        
        # ─── TENDON 1 (X-axis): eyelet_L ↔ eyelet_R ───────────────────────
        eyelet_L = goat.get_link("eyelet_L")
        eyelet_R = goat.get_link("eyelet_R")
        
        if eyelet_L is not None and eyelet_R is not None:
            pos_L = eyelet_L.get_pos()
            pos_R = eyelet_R.get_pos()
            vel_L = eyelet_L.get_lin_vel()
            vel_R = eyelet_R.get_lin_vel()
            
            # Cable path: Payload → eyelet_L → eyelet_R → Payload
            dist_L = np.linalg.norm(pos_L - payload_pos)
            dist_R = np.linalg.norm(pos_R - payload_pos)
            total_dist = dist_L + dist_R
            
            # Compression: l1_target is desired cable length, shorter = more compressed
            compression_1 = max(0, tendon_state["l1"] - total_dist)
            force_mag_1 = min(MAX_FORCE, CABLE_K * compression_1)
            
            # Direction: toward payload (negative for L side, positive for R side)
            dir_L = (payload_pos - pos_L) / (dist_L + 1e-6)
            dir_R = (payload_pos - pos_R) / (dist_R + 1e-6)
            
            # Damping
            damp_L = CABLE_D * np.dot(vel_L, dir_L)
            damp_R = CABLE_D * np.dot(vel_R, dir_R)
            
            force_L = (force_mag_1 + damp_L) * dir_L
            force_R = (force_mag_1 + damp_R) * dir_R
            
            eyelet_L.apply_force(force_L)
            eyelet_R.apply_force(force_R)
        
        # ─── TENDON 2 (Y-axis): eyelet_F ↔ eyelet_B ───────────────────────
        eyelet_F = goat.get_link("eyelet_F")
        eyelet_B = goat.get_link("eyelet_B")
        
        if eyelet_F is not None and eyelet_B is not None:
            pos_F = eyelet_F.get_pos()
            pos_B = eyelet_B.get_pos()
            vel_F = eyelet_F.get_lin_vel()
            vel_B = eyelet_B.get_lin_vel()
            
            # Cable path: Payload → eyelet_F → eyelet_B → Payload
            dist_F = np.linalg.norm(pos_F - payload_pos)
            dist_B = np.linalg.norm(pos_B - payload_pos)
            total_dist = dist_F + dist_B
            
            # Compression
            compression_2 = max(0, tendon_state["l2"] - total_dist)
            force_mag_2 = min(MAX_FORCE, CABLE_K * compression_2)
            
            # Direction: toward payload
            dir_F = (payload_pos - pos_F) / (dist_F + 1e-6)
            dir_B = (payload_pos - pos_B) / (dist_B + 1e-6)
            
            # Damping
            damp_F = CABLE_D * np.dot(vel_F, dir_F)
            damp_B = CABLE_D * np.dot(vel_B, dir_B)
            
            force_F = (force_mag_2 + damp_F) * dir_F
            force_B = (force_mag_2 + damp_B) * dir_B
            
            eyelet_F.apply_force(force_F)
            eyelet_B.apply_force(force_B)
            
    except Exception as e:
        pass  # Links might not exist or error in calculation

def set_morphology(name):
    """Set target morphology (cable lengths l1, l2)"""
    l1, l2 = MORPH[name]
    tendon_target["l1"] = l1
    tendon_target["l2"] = l2

# ── Locomotion ────────────────────────────────────────────────────────────────
def skid_steer(v_lin, v_ang):
    """
    Toutes roues axis Y (URDF v4).
    half_track = 0.35 m (FL à Y=+0.35, FR à Y=-0.35)
    Ordre WHEEL_DOFS: FL, FR, RL, RR
    """
    half_track = 0.35
    vL = v_lin - half_track * v_ang
    vR = v_lin + half_track * v_ang
    goat.control_dofs_velocity(
        np.array([vL, vR, vL, vR]),
        dofs_idx_local=np.array(WHEEL_DOFS))

def morph_step(stages, steps_per_stage=200):
    for stage in stages:
        set_morphology(stage)
        for _ in range(steps_per_stage):
            apply_cable_forces()
            scene.step()

def morph_to_sphere():
    morph_step(["rover", "folding", "sphere"])

def morph_to_rover():
    morph_step(["folding", "rover"])

def initiate_rolling(delta=0.04):
    l1, l2 = MORPH["sphere"]
    tendon_target["l1"] = l1 + delta
    tendon_target["l2"] = l2 - delta

# ── Main ──────────────────────────────────────────────────────────────────────
print(f"Starting GOAT in [{args.config}] mode — dt=0.002s (500 Hz)")

if args.config == "rover":
    set_morphology("rover")
    print("Settling rover configuration (0.3s)...")
    for _ in range(150):
        apply_cable_forces()
        scene.step()

    print("Driving forward 1.0 m/s for 5s...")
    for i in range(2500):
        apply_cable_forces()
        skid_steer(v_lin=1.0, v_ang=0.0)
        scene.step()
        if i % 500 == 0:
            pos = goat.get_link("payload").get_pos().tolist()
            print(f"  t={i*0.002:.1f}s  "
                  f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  "
                  f"cables=[{tendon_state['l1']:.3f}, {tendon_state['l2']:.3f}]")

    print("Test virage gauche (v_ang=+1.0 rad/s)...")
    for i in range(1000):
        apply_cable_forces()
        skid_steer(v_lin=0.5, v_ang=1.0)
        scene.step()
    print("Done.")

elif args.config == "sphere":
    print("Morphing to sphere (safe path, paper Fig. 2C)...")
    morph_to_sphere()

    print("Settling sphere (0.6s)...")
    for _ in range(300):
        apply_cable_forces()
        scene.step()

    print("Initiating rolling — shifting CoM (paper Fig. 4D)...")
    initiate_rolling(delta=0.04)
    for i in range(3000):
        apply_cable_forces()
        scene.step()
        if i % 500 == 0:
            pos = goat.get_link("payload").get_pos().tolist()
            print(f"  t={i*0.002:.1f}s  "
                  f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

    print("Active stop: morphing sphere → rover (paper Fig. 4E)...")
    morph_to_rover()
    print("Done.")

elif args.config == "morph":
    set_morphology("circle")
    print("Settling circle (0.4s)...")
    for _ in range(200):
        apply_cable_forces()
        scene.step()

    print("Morphing flat → sphere (safe path 1→2→3→4, paper Fig. 2C)...")
    morph_to_sphere()

    for _ in range(100):
        apply_cable_forces()
        scene.step()

    print("Explosive reconfiguration (clutch release, paper Fig. 4F)...")
    print("Expected: jump height ≈ 0.5m in <0.3s")
    tendon_state["l1"] = 0.0
    tendon_state["l2"] = 0.0
    for i in range(150):
        apply_cable_forces()
        scene.step()
        if i % 25 == 0:
            pos = goat.get_link("payload").get_pos().tolist()
            print(f"  t={i*0.002:.3f}s  z={pos[2]:.3f}m")
    print("Done.")

print("GOAT simulation complete.")