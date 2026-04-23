"""
goat_rover_sim.py — Simulation Genesis du robot GOAT en configuration ROVER
Polzin et al., Sci. Robot. 10, eadp6419 (2025) — v5

Corrections v5 par rapport à v4 :
  1. Robot avance en +Y (roues Y± = motrices, axe rot X)
     → le gap est traversé dans la direction de la largeur L2 (axe Y)
  2. Couloir aligné en Y : murs à x = ±(gap/2), longueur dans Y
  3. URDF v5 ajoute des bras rigides (arm_X±, arm_Y±) entre payload
     et connecteurs pour représenter les tubes FG visuellement
  4. Roues X± passives (free-spin), Y± motrices
"""

import math
import argparse
import numpy as np
import genesis as gs
import pandas as pd


# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES
# ─────────────────────────────────────────────────────────────────────────────

DT           = 0.002
R_FRAME      = 4.0 / (2 * math.pi)
ASPECT_RATIO = 2.2
L1_ROVER     = 2.0 * R_FRAME          # 1.2732 m  (axe X, long)
L2_ROVER     = L1_ROVER / ASPECT_RATIO # 0.5787 m  (axe Y, court, direction avance)
WHEEL_R      = 0.25
TRACK_WIDTH  = L1_ROVER                # écartement des roues motrices Y± = L2, mais
                                       # skid-steer latéral utilise L1 pour les X±
Z_INIT       = 0.50

WHEEL_KP         = 800.0
WHEEL_KV         = 40.0
WHEEL_TORQUE_MAX = 25.0

SPRING_KP = 2000.0
SPRING_KV = 120.0


# ─────────────────────────────────────────────────────────────────────────────
# CLASSE ROBOT
# ─────────────────────────────────────────────────────────────────────────────

class GOATRover:
    def __init__(self, entity):
        self.entity = entity
        self._init_dofs()

    def _init_dofs(self):
        e = self.entity

        # Roues Y± = motrices (avance en Y, axe rot X)
        self.dof_Yp = e.get_joint("wheel_joint_Y_pos").dof_idx_local
        self.dof_Yn = e.get_joint("wheel_joint_Y_neg").dof_idx_local
        # Roues X± = passives (roulent librement en X, axe rot Y)
        self.dof_Xp = e.get_joint("wheel_joint_X_pos").dof_idx_local
        self.dof_Xn = e.get_joint("wheel_joint_X_neg").dof_idx_local
        self.dofs_wheels_drive   = np.array([self.dof_Yp, self.dof_Yn], dtype=int)
        self.dofs_wheels_passive = np.array([self.dof_Xp, self.dof_Xn], dtype=int)
        self.dofs_wheels_all     = np.array(
            [self.dof_Yp, self.dof_Yn, self.dof_Xp, self.dof_Xn], dtype=int)

        # Joints prismatiques (compliance)
        self.dof_pXp = e.get_joint("joint_connector_X_pos").dof_idx_local
        self.dof_pXn = e.get_joint("joint_connector_X_neg").dof_idx_local
        self.dof_pYp = e.get_joint("joint_connector_Y_pos").dof_idx_local
        self.dof_pYn = e.get_joint("joint_connector_Y_neg").dof_idx_local
        self.dofs_springs = np.array(
            [self.dof_pXp, self.dof_pXn, self.dof_pYp, self.dof_pYn], dtype=int)

    def configure_gains(self):
        e = self.entity

        # Roues motrices
        e.set_dofs_kp(np.full(2, WHEEL_KP), dofs_idx_local=self.dofs_wheels_drive)
        e.set_dofs_kv(np.full(2, WHEEL_KV), dofs_idx_local=self.dofs_wheels_drive)
        e.set_dofs_force_range(
            lower=np.full(2, -WHEEL_TORQUE_MAX),
            upper=np.full(2,  WHEEL_TORQUE_MAX),
            dofs_idx_local=self.dofs_wheels_drive,
        )

        # Roues passives : kp=0, kv faible (juste friction)
        e.set_dofs_kp(np.zeros(2),          dofs_idx_local=self.dofs_wheels_passive)
        e.set_dofs_kv(np.full(2, 2.0),      dofs_idx_local=self.dofs_wheels_passive)
        e.set_dofs_force_range(
            lower=np.full(2, -1.0),
            upper=np.full(2,  1.0),
            dofs_idx_local=self.dofs_wheels_passive,
        )

        # Springs prismatiques : PD centré sur 0
        e.set_dofs_kp(np.full(4, SPRING_KP), dofs_idx_local=self.dofs_springs)
        e.set_dofs_kv(np.full(4, SPRING_KV), dofs_idx_local=self.dofs_springs)
        e.set_dofs_force_range(
            lower=np.full(4, -400.0),
            upper=np.full(4,  400.0),
            dofs_idx_local=self.dofs_springs,
        )
        e.control_dofs_position(np.zeros(4), dofs_idx_local=self.dofs_springs)

    def drive_forward(self, v_lin: float, v_ang: float = 0.0):
        """Avance en +Y. v_ang tourne (skid-steer sur roues Y± avec écart L2)."""
        d2 = L2_ROVER / 2.0
        # wheel_Y_pos et wheel_Y_neg sont côte-à-côte en Y
        # Rotation autour Z : une roue accélère, l'autre ralentit
        vYp = (v_lin + d2 * v_ang) / WHEEL_R
        vYn = (v_lin - d2 * v_ang) / WHEEL_R
        self.entity.control_dofs_velocity(
            np.array([vYp, vYn]),
            dofs_idx_local=self.dofs_wheels_drive,
        )

    def stop(self):
        self.entity.control_dofs_velocity(
            np.zeros(2), dofs_idx_local=self.dofs_wheels_drive)

    def get_l2_actual(self):
        """Largeur réelle (Y) : distance connector_Y_pos ↔ connector_Y_neg."""
        cp = np.array(self.entity.get_link("connector_Y_pos").get_pos().tolist())
        cn = np.array(self.entity.get_link("connector_Y_neg").get_pos().tolist())
        return float(np.linalg.norm(cp - cn))

    def log(self, t: float, tag: str = ""):
        pos  = np.array(self.entity.get_link("base_link").get_pos().tolist())
        vel  = np.array(self.entity.get_link("base_link").get_vel().tolist())
        spd  = float(np.linalg.norm(vel[:2]))
        wv   = self.entity.get_dofs_velocity(
            dofs_idx_local=self.dofs_wheels_drive).tolist()
        v_wh = abs(float(sum(wv) / 2.0)) * WHEEL_R
        sp   = self.entity.get_dofs_position(
            dofs_idx_local=self.dofs_springs).tolist()
        sp_s = ",".join(f"{x*100:+.1f}" for x in sp)
        print(
            f"  [{tag}] t={t:.1f}s | "
            f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) | "
            f"v={spd:.3f} m/s (wheel={v_wh:.3f}) | "
            f"springs=[{sp_s}] cm"
        )


# ─────────────────────────────────────────────────────────────────────────────
# SCÉNARIOS
# ─────────────────────────────────────────────────────────────────────────────

def settle(goat, scene, duration=2.0):
    print("\n[SETTLE] Stabilisation...")
    steps = int(duration / DT)
    for i in range(steps):
        scene.step()
        if i % int(1.0 / DT) == 0:
            goat.log(i * DT, "SETTLE")


def scenario_straight(goat, scene, duration=8.0):
    print("\n[STRAIGHT] v=1.4 m/s en +Y (paper Fig. 4B)")
    pos0 = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    t_log = 0.0
    for i in range(int(duration / DT)):
        t = i * DT
        goat.drive_forward(v_lin=1.4)
        scene.step()
        if t >= t_log:
            goat.log(t, "STRAIGHT")
            t_log += 2.0
    pos1 = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    dist = float(np.linalg.norm(pos1[:2] - pos0[:2]))
    print(f"  → dist={dist:.2f} m | v_avg={dist/duration:.3f} m/s (paper: ~1.4 m/s)")
    goat.stop()


def scenario_turn(goat, scene):
    print("\n[TURN] Virage skid-steer (paper Fig. 4C)")
    phases = [(3.0, 1.4, 0.0, "AVANCE"), (4.0, 0.5, 1.0, "VIRE"), (3.0, 1.4, 0.0, "AVANCE")]
    for dur, v, w, tag in phases:
        t_log = 0.0
        for i in range(int(dur / DT)):
            t = i * DT
            goat.drive_forward(v_lin=v, v_ang=w)
            scene.step()
            if t >= t_log:
                goat.log(t, tag)
                t_log += 1.5
    goat.stop()


def scenario_gap(goat, scene, gap_width=0.50, duration=15.0):
    print(f"\n[GAP] gap={gap_width*100:.0f} cm | robot_width={L2_ROVER*100:.1f} cm")
    print(f"  Compression requise : {(L2_ROVER - gap_width)*100:.1f} cm")

    pos0  = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    logs  = []
    t_log = 0.0

    for i in range(int(duration / DT)):
        t   = i * DT
        pos = np.array(goat.entity.get_link("base_link").get_pos().tolist())
        vel = np.array(goat.entity.get_link("base_link").get_vel().tolist())
        l2  = goat.get_l2_actual()

        # PD centrage sur y=0 (axe Y = direction avance, axe X = centrage latéral)
        y_err = pos[0]   # dérive latérale en X
        y_dot = vel[0]
        v_ang = float(np.clip(-3.0 * y_err - 2.0 * y_dot, -2.0, 2.0))

        goat.drive_forward(v_lin=0.30, v_ang=v_ang)
        scene.step()

        logs.append({"t": t, "x": pos[0], "y": pos[1], "z": pos[2],
                     "vx": vel[0], "vy": vel[1], "l2_actual": l2})

        if t >= t_log:
            goat.log(t, "GAP")
            print(f"   → l2={l2*100:.1f} cm (gap={gap_width*100:.0f} cm)")
            t_log += 1.0

    pd.DataFrame(logs).to_csv("gap_debug.csv", index=False)
    pos1 = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    dist = float(np.linalg.norm(pos1[:2] - pos0[:2]))
    print(f"\n  → dist totale={dist:.2f} m")
    print("  ✓ Traversée réussie !" if dist > 1.0 else "  ✗ Robot bloqué")
    goat.stop()


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", default="all",
                   choices=["straight", "turn", "gap", "all"])
    p.add_argument("--gap-width", type=float, default=0.50)
    p.add_argument("--backend",   default="cpu", choices=["cpu", "cuda"])
    p.add_argument("--no-viewer", action="store_true")
    args = p.parse_args()

    print("=" * 62)
    print("GOAT ROVER — Genesis Simulation v5")
    print("Polzin et al., Sci. Robot. 10, eadp6419 (2025)")
    print("=" * 62)
    print(f"  L1={L1_ROVER:.4f} m | L2={L2_ROVER:.4f} m | Z_INIT={Z_INIT} m")
    print(f"  Avance en +Y | Roues motrices : Y± (hub vert)")

    backend = gs.cuda if args.backend == "cuda" else gs.cpu
    gs.init(backend=backend, precision="64", logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0.0, 0.0, -9.81)),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.0, -1.5, 2.0),
            camera_lookat=(0.0, 1.5, 0.25),
            camera_fov=55,
            max_FPS=60,
        ),
        show_viewer=not args.no_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Rough())

    # ── Couloir gap : murs parallèles à Y, robot traverse en +Y ──────────
    if args.scenario in ("gap", "all"):
        gap        = args.gap_width
        wall_len   = 2.5           # longueur du mur dans Y
        wall_thick = 0.05
        wall_h     = 0.45
        # Entrée du couloir à y=1.0, sortie à y=3.5
        wall_cy    = 1.0 + wall_len / 2.0   # centre en Y = 2.25
        x_plus     =  gap / 2.0 + wall_thick / 2.0
        x_minus    = -(gap / 2.0 + wall_thick / 2.0)

        print(f"\n  [GAP] Couloir x=[{-gap/2:.2f},{gap/2:.2f}] m | "
              f"y=[1.0,{1.0+wall_len:.1f}] m | gap={gap*100:.0f} cm")

        scene.add_entity(
            gs.morphs.Box(pos=(x_plus,  wall_cy, wall_h/2.0),
                          size=(wall_thick, wall_len, wall_h)),
            surface=gs.surfaces.Rough())
        scene.add_entity(
            gs.morphs.Box(pos=(x_minus, wall_cy, wall_h/2.0),
                          size=(wall_thick, wall_len, wall_h)),
            surface=gs.surfaces.Rough())

    robot = scene.add_entity(
        gs.morphs.URDF(
            file="goat_rover_prism.urdf",
            pos=(0.0, 0.0, Z_INIT),
            euler=(0.0, 0.0, 0.0),
            fixed=False,
        )
    )

    scene.build()

    goat = GOATRover(robot)
    goat.configure_gains()
    print(f"[Genesis] {robot.n_links} links | {robot.n_dofs} dofs")

    settle(goat, scene)

    if args.scenario in ("straight", "all"):
        scenario_straight(goat, scene)
    if args.scenario in ("turn", "all"):
        scenario_turn(goat, scene)
    if args.scenario in ("gap", "all"):
        scenario_gap(goat, scene, gap_width=args.gap_width)

    print("\n[DONE] Simulation terminée.")


if __name__ == "__main__":
    main()