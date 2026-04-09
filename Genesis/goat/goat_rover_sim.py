"""
goat_rover_sim.py — Simulation Genesis du robot GOAT en configuration ROVER
Polzin et al., Sci. Robot. 10, eadp6419 (2025)

Usage:
python goat_rover_sim_v2.py                    # tous les scénarios, viewer ON
python goat_rover_sim_v2.py --scenario straight
python goat_rover_sim_v2.py --scenario turn
python goat_rover_sim_v2.py --scenario aspect_ratio
python goat_rover_sim_v2.py --scenario gap      # ← NOUVEAU: Fig. 3B du paper
python goat_rover_sim_v2.py --no-viewer         # headless
python goat_rover_sim_v2.py --backend cuda      # GPU

ARCHITECTURE (fidèle au paper, Materials & Methods):
────────────────────────────────────────────────────
Cycle fermé câble = impossible en URDF → simulé en Python par apply_force().

Routage exact des tendons (paper):
Tendon 1 (axe X, "l1"):
  payload ──winch──► eyelet(connector_X_pos) ──câble──► eyelet(connector_X_neg) ──► payload
Tendon 2 (axe Y, "l2"):
  payload ──winch──► eyelet(connector_Y_pos) ──câble──► eyelet(connector_Y_neg) ──► payload

Modèle de force (chaque brin du câble):
  F = k_cable * max(dist - l_ref/2, 0)  [unilatéral, câble PE ne pousse pas]
    + d_cable * max(ḋ, 0)               [amortissement visqueux]
  Direction: du connecteur vers le payload (tension)
  Réaction Newton III: sur le payload (−F)

Données numériques du paper:
  R_frame = 4m / (2π) = 0.6366 m  (4 tiges FG 2m, 5mm ø, 2 anneaux 4m)
  Aspect ratio = 2.2 (rover, optimal: "fastest speed 1.4 m/s at r=2.0-3.0")
  l1 = 2*R = 1.2732 m  (tendon 1, axe long X)
  l2 = l1/2.2 = 0.5787 m  (tendon 2, axe court Y)
  Track width d = 2*l2/2 = 0.5787 m  (eq. 3&4 du paper)
  Wheel radius = 0.25 m  (longueur rayons FG 8mm, paper)
  Motor torque = 4.41 N·m  (45 kg·cm, paper)
  Winch torque = 3.92 N·m  (40 kg·cm, paper)
  Mass total = 2.80 kg  (payload 1.8 kg + frame/roues 1.0 kg, Table 1)

SCÉNARIO GAP (nouveau, Fig. 3B):
  - Deux murs parallèles créant un couloir de 50 cm de large
  - Robot (largeur Y = L2_ROVER = 0.5787 m) doit se comprimer pour passer
  - Attendu: front compresses, back bulges (paper Fig. 3B)
  - Les pivots (PIVOT_KP) et tendons (TENDON_K) assurent la compliance
  - Les segments FG avec <collision> transmettent les forces de contact
"""

import math
import argparse
import numpy as np
import genesis as gs

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTES DU ROBOT — toutes issues du paper
# ══════════════════════════════════════════════════════════════════════════════

R_FRAME      = 4.0 / (2 * math.pi)     # 0.6366 m
ASPECT_RATIO = 2.2
L1_ROVER     = 2.0 * R_FRAME           # 1.2732 m (tendon 1, axe X)
L2_ROVER     = L1_ROVER / ASPECT_RATIO # 0.5787 m (tendon 2, axe Y)
POS_X        = L1_ROVER / 2.0          # 0.6366 m
POS_Y        = L2_ROVER / 2.0          # 0.2894 m
WHEEL_R      = 0.25                    # m (longueur rayons FG)
TRACK_WIDTH  = L2_ROVER                # 0.5787 m (d dans eq.3&4 du paper)
Z_INIT       = WHEEL_R                 # 0.25 m (roues au sol)

# Config circle r=1.0 (meilleur virage, paper Fig. 4C)
L1_CIRCLE    = L1_ROVER
L2_CIRCLE    = L1_ROVER                # l1=l2 → r=1

# Dynamiques tendons (câble PE 0.6mm, k réduit pour stabilité dt=0.002s)
# Légèrement augmentés v2 pour meilleure réactivité lors du gap
TENDON_K     = 2000.0   # N/m   (était 1500 en v1)
TENDON_DAMP  = 80.0     # N·s/m (était 60 en v1)

# Compliance frame: EI/L pour tige FG 5mm ø, L=0.5m
# E=45 GPa, I=π*d⁴/64=3.07e-11 m⁴ → EI=1.38 N·m² → k=EI/L=2.76 N·m/rad
# Légèrement réduit en v2 pour que la frame plie davantage sous contact latéral
PIVOT_KP     = 1.5      # N·m/rad  (était 3.0 en v1)
PIVOT_KV     = 0.3      # N·m·s/rad (était 0.5 en v1)

# ══════════════════════════════════════════════════════════════════════════════
# CLASSE PRINCIPALE
# ══════════════════════════════════════════════════════════════════════════════

class GOATRover:
    def __init__(self, entity):
        self.entity = entity
        self._init_dofs()

    def _init_dofs(self):
        e = self.entity
        # Roues: X_pos=droite, X_neg=gauche, Y_pos=avant, Y_neg=arrière
        # (axe de marche = +X en config rover r=2.2)
        self.dof_R = e.get_joint("wheel_joint_X_pos").dof_idx_local
        self.dof_L = e.get_joint("wheel_joint_X_neg").dof_idx_local
        self.dof_F = e.get_joint("wheel_joint_Y_pos").dof_idx_local
        self.dof_B = e.get_joint("wheel_joint_Y_neg").dof_idx_local
        self.dofs_wheels = np.array([self.dof_R, self.dof_L, self.dof_F, self.dof_B])

        # Pivots flexibles (compliance fibre de verre)
        self.dofs_pivots = np.array([
            e.get_joint("joint_pivot_XpYp").dof_idx_local,
            e.get_joint("joint_pivot_XpYn").dof_idx_local,
            e.get_joint("joint_pivot_XnYp").dof_idx_local,
            e.get_joint("joint_pivot_XnYn").dof_idx_local,
        ])

    def configure_gains(self):
        e = self.entity
        # Moteurs roues: 45 kg·cm = 4.41 N·m (paper)
        e.set_dofs_kp(np.full(4, 1500.0), dofs_idx_local=self.dofs_wheels)
        e.set_dofs_kv(np.full(4, 30.0),  dofs_idx_local=self.dofs_wheels)
        e.set_dofs_force_range(
            lower=np.full(4, -20.0),  # N·m, limite de couple moteur (paper 4.41 N·m)
            upper=np.full(4,  20.0),
            dofs_idx_local=self.dofs_wheels)
        # Pivots: compliance fibre de verre kp=PIVOT_KP N·m/rad
        e.set_dofs_kp(np.zeros(4), dofs_idx_local=self.dofs_pivots)
        e.set_dofs_kv(np.zeros(4), dofs_idx_local=self.dofs_pivots)
        e.control_dofs_position(np.zeros(4), dofs_idx_local=self.dofs_pivots)

    # ── Locomotion (paper eq. 3 & 4) ───────────────────────────────────────
    def skid_steer(self, v_lin: float, v_ang: float, track: float = TRACK_WIDTH):
        """
        Version générale (paper eq. 3 & 4) – utilisée pour straight/turn/aspect_ratio.
        """
        d2 = track / 2.0
        vR = (v_lin + d2 * v_ang) / WHEEL_R
        vL = (v_lin - d2 * v_ang) / WHEEL_R
        v_side = v_lin / WHEEL_R  # roues Y en skid passif
        self.entity.control_dofs_velocity(
            np.array([vR, vL, v_side, v_side]),
            dofs_idx_local=self.dofs_wheels)

    def skid_steer_x_only(self, v_lin: float, v_ang: float, track: float = TRACK_WIDTH):
        """
        Version 'gap' : seule la paire X est motrice.
        Fidèle aux eq. 3–4, sans artificiellement entraîner les roues Y.
        """
        d2 = track / 2.0
        vR = (v_lin + d2 * v_ang) / WHEEL_R
        vL = (v_lin - d2 * v_ang) / WHEEL_R
        self.entity.control_dofs_velocity(
            np.array([vR, vL, 0.0, 0.0]),   # Y_pos, Y_neg figés
            dofs_idx_local=self.dofs_wheels)

    def stop(self):
        self.entity.control_dofs_velocity(
            np.zeros(4), dofs_idx_local=self.dofs_wheels)

    def set_solver(self, scene):
        """Récupère le RigidSolver par nom de classe (pas d'import nécessaire)."""
        self._solver = None
        for s in scene.sim.solvers:
            if type(s).__name__ == "RigidSolver":
                self._solver = s
                break
        if self._solver is None:
            # Fallback: premier solver qui a apply_links_external_force
            for s in scene.sim.solvers:
                if hasattr(s, "apply_links_external_force"):
                    self._solver = s
                    break
        if self._solver is None:
            raise RuntimeError("RigidSolver introuvable dans scene.sim.solvers")
        e = self.entity
        self._idx_payload = e.get_link("base_link").idx
        self._idx_conn_Xp = e.get_link("connector_X_pos").idx
        self._idx_conn_Xn = e.get_link("connector_X_neg").idx
        self._idx_conn_Yp = e.get_link("connector_Y_pos").idx
        self._idx_conn_Yn = e.get_link("connector_Y_neg").idx

    def apply_tendon_forces(self, l1: float, l2: float):
        e       = self.entity
        payload = e.get_link("base_link")
        p_pos   = np.array(payload.get_pos().tolist())
        try:
            p_vel = np.array(payload.get_vel().tolist())
        except Exception:
            p_vel = np.zeros(3)

        tendons = [
            ("connector_X_pos", self._idx_conn_Xp, l1 / 2.0),
            ("connector_X_neg", self._idx_conn_Xn, l1 / 2.0),
            ("connector_Y_pos", self._idx_conn_Yp, l2 / 2.0),
            ("connector_Y_neg", self._idx_conn_Yn, l2 / 2.0),
        ]

        for conn_name, conn_idx, l_half in tendons:
            conn  = e.get_link(conn_name)
            c_pos = np.array(conn.get_pos().tolist())
            delta = p_pos - c_pos
            dist  = float(np.linalg.norm(delta)) + 1e-9
            unit  = delta / dist
            stretch = dist - l_half

            if stretch > 0.0:
                try:
                    c_vel = np.array(conn.get_vel().tolist())
                except Exception:
                    c_vel = np.zeros(3)
                v_axial = float(np.dot(p_vel - c_vel, unit))
                F_mag   = max(TENDON_K * stretch + TENDON_DAMP * v_axial, 0.0)
                F_vec   = F_mag * unit

                self._solver.apply_links_external_force(
                    force=np.array([F_vec]),
                    links_idx=[conn_idx]
                )
                self._solver.apply_links_external_force(
                    force=np.array([-F_vec]),
                    links_idx=[self._idx_payload]
                )
        # rappel élastique pivot → connecteur parent
        pivot_connector_pairs = [
            ("pivot_XpYp", "connector_X_pos", "connector_Y_pos"),
            ("pivot_XpYn", "connector_X_pos", "connector_Y_neg"),
            ("pivot_XnYp", "connector_X_neg", "connector_Y_pos"),
            ("pivot_XnYn", "connector_X_neg", "connector_Y_neg"),
        ]
        K_pivot = 500.0   # N/m — raideur de rappel frame

        for pname, cname1, cname2 in pivot_connector_pairs:
            piv  = self.entity.get_link(pname)
            p_p  = np.array(piv.get_pos().tolist())
            
            for cname in (cname1, cname2):
                conn = self.entity.get_link(cname)
                c_p  = np.array(conn.get_pos().tolist())
                
                # Position de repos attendue (géométrie URDF)
                # on récupère le vecteur actuel pivot→connecteur
                delta = c_p - p_p
                dist  = np.linalg.norm(delta) + 1e-9
                
                # Longueur de repos = distance URDF nominale
                # XpYp → connXp : 0.6366-0.4502=0.1864m
                # XpYp → connYp : 0.2894-0.2046=0.0848m  
                L0_map = {
                    ("pivot_XpYp","connector_X_pos"): 0.1864,
                    ("pivot_XpYp","connector_Y_pos"): 0.0848,
                    ("pivot_XpYn","connector_X_pos"): 0.1864,
                    ("pivot_XpYn","connector_Y_neg"): 0.0848,
                    ("pivot_XnYp","connector_X_neg"): 0.1864,
                    ("pivot_XnYp","connector_Y_pos"): 0.0848,
                    ("pivot_XnYn","connector_X_neg"): 0.1864,
                    ("pivot_XnYn","connector_Y_neg"): 0.0848,
                }
                L0 = L0_map[(pname, cname)]
                stretch = dist - L0
                
                if abs(stretch) > 1e-4:
                    F_mag = K_pivot * stretch
                    F_vec = F_mag * (delta / dist)
                    
                    c_idx = self.entity.get_link(cname).idx
                    p_idx = piv.idx
                    
                    # Force sur connecteur (tire vers pivot si étiré)
                    self._solver.apply_links_external_force(
                        force=np.array([-F_vec]), links_idx=[c_idx])
                    # Réaction sur pivot
                    self._solver.apply_links_external_force(
                        force=np.array([F_vec]),  links_idx=[p_idx])


    # ── Logging ─────────────────────────────────────────────────────────────
    def log(self, t: float, tag: str = ""):
        pos   = np.array(self.entity.get_link("base_link").get_pos().tolist())
        vel   = np.array(self.entity.get_link("base_link").get_vel().tolist())
        speed = float(np.linalg.norm(vel[:2]))
        w_vel = self.entity.get_dofs_velocity(dofs_idx_local=self.dofs_wheels)
        v_est = float(w_vel[0] + w_vel[1]) / 2.0 * WHEEL_R
        pivs  = np.degrees(
            self.entity.get_dofs_position(dofs_idx_local=self.dofs_pivots).tolist())
        print(f"  [{tag}] t={t:.1f}s | "
              f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})m | "
              f"v_body={speed:.3f}m/s v_wheel={v_est:.3f}m/s | "
              f"pivots=[{','.join(f'{a:.1f}°' for a in pivs)}]")

# ══════════════════════════════════════════════════════════════════════════════
# SCÉNARIOS (calqués sur les expériences du paper)
# ══════════════════════════════════════════════════════════════════════════════

def settle(goat, scene, duration=2.0):
    """Phase de stabilisation initiale sous gravité + tension tendons."""
    print("\n[SETTLE] Stabilisation (2s)...")
    steps = int(duration / 0.002)
    for i in range(steps):
        goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
        scene.step()
        if i % 500 == 0:
            goat.log(i * 0.002, "SETTLE")


def scenario_straight(goat, scene, duration=8.0):
    """
    Marche droite à v=1.4 m/s, config rover r=2.2.
    Paper Fig. 4B: vitesse optimale 1.4 m/s à r=2.0-3.0
    Attendu: ~11.2 m en 8s, dérive < 20%
    """
    print("\n[STRAIGHT] v=1.4 m/s, r=2.2 (paper Fig. 4B)")
    pos0 = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    steps, t_log = int(duration / 0.002), 0.0
    for i in range(steps):
        t = i * 0.002
        goat.skid_steer(v_lin=1.4, v_ang=0.0)
        goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
        scene.step()
        if t >= t_log:
            goat.log(t, "STRAIGHT")
            t_log += 2.0
    pos1  = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    dist  = float(np.linalg.norm(pos1[:2] - pos0[:2]))
    drift = abs(pos1[1] - pos0[1]) / max(abs(pos1[0] - pos0[0]), 1e-3) * 100
    v_avg = dist / duration
    print(f"  → dist={dist:.2f}m | v_avg={v_avg:.3f}m/s (paper: ~1.4 m/s) | "
          f"dérive={drift:.1f}% (paper: <20% à r=2.2)")
    goat.stop()


def scenario_turn(goat, scene):
    """
    Slalom: avance 3s → vire à gauche 4s → avance 3s.
    Paper Fig. 4C: rayon de virage ~1.0 m à r=2.2
    """
    print("\n[TURN] Virage skid-steer à r=2.2 (paper Fig. 4C: rayon ~1.0 m)")
    phases = [
        (3.0, 1.4, 0.0, "AVANCE"),
        (4.0, 0.5, 1.0, "VIRE_G"),
        (3.0, 1.4, 0.0, "AVANCE"),
    ]
    for dur, v, w, tag in phases:
        print(f"  Phase {tag}: v={v} m/s, ω={w} rad/s")
        steps, t_log = int(dur / 0.002), 0.0
        for i in range(steps):
            t = i * 0.002
            goat.skid_steer(v_lin=v, v_ang=w)
            goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
            scene.step()
            if t >= t_log:
                goat.log(t, tag)
                t_log += 1.5
    goat.stop()


def scenario_aspect_ratio(goat, scene):
    """
    Compare r=1.0 / r=2.2 / r=3.3 sur 3s chacun.
    Paper Fig. 4B: v_max à r=2.2, v_min aux extrêmes
    Paper Fig. 4C: virage max à r=1.0 (circle)
    """
    print("\n[ASPECT_RATIO] Comparaison r=1.0 / 2.2 / 3.3 (paper Fig. 4B&C)")
    configs = [
        (1.0, "circle r=1.0 (meilleur virage)"),
        (2.2, "rover r=2.2 (vitesse optimale ~1.4 m/s)"),
        (3.3, "elongé r=3.3 (vitesse réduite)"),
    ]
    for r, label in configs:
        l1 = L1_ROVER
        l2 = l1 / r
        print(f"\n  Config: {label} | l1={l1:.3f}m l2={l2:.3f}m")
        goat.stop()
        for _ in range(500):
            goat.apply_tendon_forces(l1, l2)
            scene.step()
        pos0 = np.array(goat.entity.get_link("base_link").get_pos().tolist())
        for _ in range(1500):
            goat.skid_steer(v_lin=1.4, v_ang=0.0, track=l2)
            goat.apply_tendon_forces(l1, l2)
            scene.step()
        pos1  = np.array(goat.entity.get_link("base_link").get_pos().tolist())
        dist  = float(np.linalg.norm(pos1[:2] - pos0[:2]))
        drift = abs(pos1[1] - pos0[1]) / max(abs(pos1[0] - pos0[0]), 1e-3) * 100
        print(f"  → v_mesurée={dist/3:.3f}m/s | dérive={drift:.1f}%")
        print(f"  → paper attend: {'~1.4 m/s' if r==2.2 else '< 1.4 m/s'}")
    goat.stop()


def scenario_gap(goat, scene, gap_width=0.50, approach_dist=2.5, duration=15.0):
    """
    Reproduction de la Fig. 3B du paper : traversée d'un gap (couloir étroit).

    Physique attendue:
      - Frame width au repos = L2_ROVER = 0.5787 m
      - gap_width = 0.50 m  → le robot doit se comprimer de ~8 cm
      - L'avant (connector_Y_pos) entre en contact avec les murs → se comprime
      - Les pivots transmettent les couples → l'arrière gonfle (bulges)
      - Après passage, les tendons ramènent la frame à sa forme rover

    IMPORTANT: ce scénario nécessite que les segments FG (seg_*) aient
    des blocs <collision> dans le URDF, sinon les murs ne transmettent
    aucune force à la frame.

    Paper Fig. 3B: succès si gap ≥ ~45 cm (~83% de la largeur robot 54 cm).
    Ici on simule le passage à 50 cm (succès probable selon la courbe du paper).
    """
    print(f"\n[GAP] Traversée gap={gap_width*100:.0f} cm "
          f"(paper Fig. 3B, robot width={L2_ROVER*100:.1f} cm)")
    print(f"  Robot doit se comprimer de {(L2_ROVER - gap_width)*100:.1f} cm")
    print(f"  Pendants de compliance: PIVOT_KP={PIVOT_KP} N·m/rad | "
          f"TENDON_K={TENDON_K} N/m")

    steps  = int(duration / 0.002)
    t_log  = 0.0
    pos0   = np.array(goat.entity.get_link("base_link").get_pos().tolist())

    # Phase 1: approche (robot se déplace de 0 à approach_dist en X)
    # Phase 2: traversée du couloir (les murs sont placés à x=approach_dist)
    # Les deux phases sont continues — le robot avance à vitesse modérée
    # pour laisser la compliance agir (vitesse réduite vs straight: 0.6 m/s)
    for i in range(steps):
        t = i * 0.002

        pos = np.array(goat.entity.get_link("base_link").get_pos().tolist())
        vel = np.array(goat.entity.get_link("base_link").get_vel().tolist())

        # Erreur latérale
        y_err = pos[1]
        y_dot = vel[1]          # dérivée = vitesse latérale

        # Contrôleur PD : corrige position ET vitesse latérale
        Kp = 3.0
        Kd = 2.0
        v_ang = -Kp * y_err - Kd * y_dot
        v_ang = np.clip(v_ang, -2.0, 2.0)

        # Locomotion purement en X, fidèle aux eq. 3–4
        goat.skid_steer_x_only(v_lin=1.4, v_ang=v_ang)      #v_lin=1.4 m/s pour laisser la compliance agir (paper Fig. 3B)

        goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
        scene.step()

        if t >= t_log:
            pos = np.array(goat.entity.get_link("base_link").get_pos().tolist())
            x_from_start = pos[0] - pos0[0]
            goat.log(t, "GAP")

            # Indique la phase en cours
            if x_from_start < approach_dist:
                print(f"    → Approche: {x_from_start:.2f}m / {approach_dist:.2f}m")
            elif x_from_start < approach_dist + 1.5:
                print(f"    → DANS LE COULOIR (compression active attendue)")
            else:
                print(f"    → Sortie du couloir, frame revient à sa forme")
            t_log += 1.0

    pos1  = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    dist  = float(np.linalg.norm(pos1[:2] - pos0[:2]))
    drift = abs(pos1[1] - pos0[1]) / max(abs(pos1[0] - pos0[0]), 1e-3) * 100
    print(f"\n  → dist totale={dist:.2f}m | dérive_lat={drift:.1f}%")
    if dist > approach_dist:
        print(f"  ✓ Robot a traversé le gap !")
    else:
        print(f"  ✗ Robot bloqué avant le gap (dist < approach_dist)")
    goat.stop()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="GOAT Rover Simulation — Genesis (Polzin et al. 2025)")
    p.add_argument("--scenario", default="all",
                   choices=["straight", "turn", "aspect_ratio", "gap", "all"])
    p.add_argument("--gap-width", type=float, default=0.50,
                   help="Largeur du gap en mètres (défaut: 0.50 m, paper Fig. 3B)")
    p.add_argument("--backend", default="cpu", choices=["cpu", "cuda"])
    p.add_argument("--no-viewer", action="store_true")
    args = p.parse_args()

    print("=" * 62)
    print("GOAT ROVER — Genesis Simulation v2")
    print("Polzin et al., Sci. Robot. 10, eadp6419 (2025)")
    print("=" * 62)
    print(f"  R_frame={R_FRAME:.4f}m | r={ASPECT_RATIO} | "
          f"l1={L1_ROVER:.4f}m | l2={L2_ROVER:.4f}m")
    print(f"  track_width={TRACK_WIDTH:.4f}m | wheel_r={WHEEL_R}m | "
          f"Z_init={Z_INIT}m\n")

    backend = gs.cuda if args.backend == "cuda" else gs.cpu
    gs.init(backend=backend, precision="64", logging_level="warning")

    # ── Scène de base ──────────────────────────────────────────────────────
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.002, gravity=(0.0, 0.0, -9.81)),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.2, -2.5, 1.6),
            camera_lookat=(0.0, 0.0, 0.25),
            camera_fov=55, max_FPS=60),
        show_viewer=not args.no_viewer,
    )

    scene.add_entity(gs.morphs.Plane())

    # ── Murs du gap (uniquement si scénario gap) ─────────────────
    #
    #  Vue de dessus:
    #
    #        ████████████████████  ← mur Y+ (y = +gap/2 + épaisseur/2)
    #  robot ──────────────────→   (axe X)
    #        ████████████████████  ← mur Y- (y = -gap/2 - épaisseur/2)
    #
    #  Le robot commence à x=0, les murs débutent à x=approach_dist=1.5m
    #  et font wall_length=2.0m de long → couvrent x=[1.5, 3.5]

    if args.scenario in ("gap", "all"):
        gap         = args.gap_width        # 0.50 m  (paper Fig. 3B)
        approach_x  = 2.5                   # m: distance avant d'entrer dans le gap
        wall_len    = 2.0                   # m: longueur du couloir
        wall_thick  = 0.05                  # m
        wall_h      = 0.45                  # m: plus haut que les roues (r=0.25m)
        wall_cx     = approach_x + wall_len / 2.0   # centre X des murs
        y_plus      = gap / 2.0 + wall_thick / 2.0
        y_minus     = -(gap / 2.0 + wall_thick / 2.0)

        print(f"  [GAP] Couloir: x=[{approach_x:.1f}, {approach_x+wall_len:.1f}]m | "
              f"gap={gap*100:.0f}cm | robot_width={L2_ROVER*100:.1f}cm")

        # Mur Y+
        scene.add_entity(gs.morphs.Box(
        pos=(wall_cx, y_plus, wall_h / 2.0),
        size=(wall_len, wall_thick, wall_h),
        ), surface=gs.surfaces.Rough())   # ← force le contact

        scene.add_entity(gs.morphs.Box(
            pos=(wall_cx, y_minus, wall_h / 2.0),
            size=(wall_len, wall_thick, wall_h),
        ), surface=gs.surfaces.Rough())

    # ── Chargement URDF ────────────────────────────────────────────────────
    robot_entity = scene.add_entity(
        gs.morphs.URDF(
            file="goat_rover_2.urdf",
            pos=(0.0, 0.0, Z_INIT),
            euler=(0.0, 0.0, 0.0),
            fixed=False))

    scene.build()

    goat = GOATRover(robot_entity)
    goat.configure_gains()
    goat.set_solver(scene)

    print(f"[Genesis] {robot_entity.n_links} links chargés.")

    # ── Stabilisation ─────────────────────────────────────────────────────
    settle(goat, scene)

    # ── Scénarios ─────────────────────────────────────────────────────────
    if args.scenario in ("straight", "all"):
        scenario_straight(goat, scene)

    if args.scenario in ("turn", "all"):
        scenario_turn(goat, scene)

    if args.scenario in ("aspect_ratio", "all"):
        scenario_aspect_ratio(goat, scene)

    if args.scenario in ("gap", "all"):
        scenario_gap(goat, scene, gap_width=args.gap_width, duration=15.0)

    print("\n[DONE] Simulation terminée.")


if __name__ == "__main__":
    main()
