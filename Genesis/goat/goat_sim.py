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
        file="goat_v4.urdf",
        pos=(0.0, 0.0, 0.20),
        euler=(0.0, 0.0, 0.0),
        fixed=False,
    )
)

scene.build()

# Debug: affiche les links disponibles
print("[DEBUG] Links disponibles:", [j.name for j in goat.links])

# ── DOF index lookup ──────────────────────────────────────────────────────────
def dof(joint_name):
    return goat.get_joint(joint_name).dof_idx_local

WHEEL_DOFS = [
    dof("wheel_FL_joint"),
    dof("wheel_FR_joint"),
    dof("wheel_RL_joint"),
    dof("wheel_RR_joint"),
]
TENDON_DOFS = [dof("tendon_1"), dof("tendon_2")]

# ── PD gains ──────────────────────────────────────────────────────────────────
# Paper: roues 45 kg·cm = 4.41 N·m, treuils 40 kg·cm = 3.92 N·m
goat.set_dofs_kp(np.array([200.0]*4), dofs_idx_local=np.array(WHEEL_DOFS))
goat.set_dofs_kv(np.array([ 20.0]*4), dofs_idx_local=np.array(WHEEL_DOFS))
goat.set_dofs_force_range(
    lower=np.array([-4.41]*4), upper=np.array([4.41]*4),
    dofs_idx_local=np.array(WHEEL_DOFS))

goat.set_dofs_kp(np.array([500.0]*2), dofs_idx_local=np.array(TENDON_DOFS))
goat.set_dofs_kv(np.array([ 50.0]*2), dofs_idx_local=np.array(TENDON_DOFS))
goat.set_dofs_force_range(
    lower=np.array([-3.92]*2), upper=np.array([3.92]*2),
    dofs_idx_local=np.array(TENDON_DOFS))

# ── Morphology presets (paper Fig. 2C) ───────────────────────────────────────
MORPH = {
    "circle":  (0.20, 0.20),
    "rover":   (0.30, 0.15),
    "folding": (0.10, 0.10),
    "sphere":  (0.05, 0.05),
}

def set_morphology(name):
    l1, l2 = MORPH[name]
    goat.control_dofs_position(
        np.array([l1, l2]),
        dofs_idx_local=np.array(TENDON_DOFS))

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
            scene.step()

def morph_to_sphere():
    morph_step(["rover", "folding", "sphere"])

def morph_to_rover():
    morph_step(["folding", "rover"])

def initiate_rolling(delta=0.04):
    l1, l2 = MORPH["sphere"]
    goat.control_dofs_position(
        np.array([l1 + delta, l2 - delta]),
        dofs_idx_local=np.array(TENDON_DOFS))

# ── Main ──────────────────────────────────────────────────────────────────────
print(f"Starting GOAT in [{args.config}] mode — dt=0.002s (500 Hz)")

if args.config == "rover":
    set_morphology("rover")
    print("Settling rover configuration (0.3s)...")
    for _ in range(150):
        scene.step()

    print("Driving forward 1.0 m/s for 5s...")
    for i in range(2500):
        skid_steer(v_lin=1.0, v_ang=0.0)
        scene.step()
        if i % 500 == 0:
            pos = goat.get_link("payload").get_pos().tolist()
            t = goat.get_dofs_position(
                dofs_idx_local=np.array(TENDON_DOFS)).tolist()
            print(f"  t={i*0.002:.1f}s  "
                  f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  "
                  f"tendons=[{t[0]:.3f}, {t[1]:.3f}]")

    print("Test virage gauche (v_ang=+1.0 rad/s)...")
    for i in range(1000):
        skid_steer(v_lin=0.5, v_ang=1.0)
        scene.step()
    print("Done.")

elif args.config == "sphere":
    print("Morphing to sphere (safe path, paper Fig. 2C)...")
    morph_to_sphere()

    print("Settling sphere (0.6s)...")
    for _ in range(300):
        scene.step()

    print("Initiating rolling — shifting CoM (paper Fig. 4D)...")
    initiate_rolling(delta=0.04)
    for i in range(3000):
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
        scene.step()

    print("Morphing flat → sphere (safe path 1→2→3→4, paper Fig. 2C)...")
    morph_to_sphere()

    for _ in range(100):
        scene.step()

    print("Explosive reconfiguration (clutch release, paper Fig. 4F)...")
    print("Expected: jump height ≈ 0.5m in <0.3s")
    goat.control_dofs_force(
        np.array([0.0, 0.0]),
        dofs_idx_local=np.array(TENDON_DOFS))
    for i in range(150):
        scene.step()
        if i % 25 == 0:
            pos = goat.get_link("payload").get_pos().tolist()
            print(f"  t={i*0.002:.3f}s  z={pos[2]:.3f}m")
    print("Done.")

print("GOAT simulation complete.")