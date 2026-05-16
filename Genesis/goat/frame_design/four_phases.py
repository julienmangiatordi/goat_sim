"""
four_phases_tendon.py  : tendons physiques XPBD
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Tendons T1 (axe X) et T2 (axe Y) modélisés comme entités PBD
Cloth quasi-inextensibles. Le winch est simulé par patch direct
de solver.edges_info[i].len_rest à chaque step (XPBD natif).

Fixes vs version précédente :
  - n_support_neighbors=4 dans VisOptions  (fix kth out of bounds)
  - cylinder tendon n=20 sections           (garantit ≥20 particules)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import trimesh
import numpy as np
import genesis as gs
import csv
import time

# ══════════════════════════════════════════════════════════════
# PARTIE 1 — Génération des OBJ
# ══════════════════════════════════════════════════════════════

def make_wavy_ring_mesh(R, wave_amp, phase, tube_r, n_major=80, n_tube=12):
    u = np.linspace(0, 2*np.pi, n_major, endpoint=False)
    v = np.linspace(0, 2*np.pi, n_tube, endpoint=False)
    U, V = np.meshgrid(u, v)
    cx = R * np.cos(U);  cy = R * np.sin(U)
    cz = wave_amp * np.sin(2*U + phase)
    tx = -R*np.sin(U);   ty = R*np.cos(U)
    tz = 2*wave_amp*np.cos(2*U + phase)
    t_norm = np.sqrt(tx**2+ty**2+tz**2)+1e-12
    tx, ty, tz = tx/t_norm, ty/t_norm, tz/t_norm
    bx=ty; by=-tx; bz=np.zeros_like(tx)
    b_norm = np.sqrt(bx**2+by**2)+1e-12
    bx, by = bx/b_norm, by/b_norm
    nx=by*tz-bz*ty; ny=bz*tx-bx*tz; nz=bx*ty-by*tx
    px = cx + tube_r*(np.cos(V)*nx + np.sin(V)*bx)
    py = cy + tube_r*(np.cos(V)*ny + np.sin(V)*by)
    pz = cz + tube_r*(np.cos(V)*nz + np.sin(V)*bz)
    vertices = np.stack([px.flatten(), py.flatten(), pz.flatten()], axis=1)
    nU, nV = n_major, n_tube
    faces = []
    for i in range(nV):
        for j in range(nU):
            v0=i*nU+j; v1=i*nU+(j+1)%nU
            v2=((i+1)%nV)*nU+(j+1)%nU; v3=((i+1)%nV)*nU+j
            faces += [[v0,v1,v2],[v0,v2,v3]]
    return trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=True)


def cylinder_between(p1, p2, radius, n=12):
    p1, p2 = np.array(p1, float), np.array(p2, float)
    vec = p2-p1; length=np.linalg.norm(vec); mid=(p1+p2)/2.
    cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=n)
    z_axis = np.array([0.,0.,1.]); vec_n=vec/length
    if np.abs(np.dot(z_axis, vec_n)) < 0.9999:
        axis = np.cross(z_axis, vec_n); axis /= np.linalg.norm(axis)
        angle = np.arccos(np.clip(np.dot(z_axis, vec_n), -1, 1))
        cyl.apply_transform(trimesh.transformations.rotation_matrix(angle, axis))
    cyl.apply_translation(mid)
    return cyl


def make_tendon_mesh(mesh_A, mesh_B, u_targets, tol_rad, tube_r=0.006):
    """
    Génère un mesh tendon (tube) reliant les centroïdes des zones
    d'ancrage angulaires de ring_A et ring_B.
    n=20 sections garantit suffisamment de particules (> n_support_neighbors=4).
    """
    vA = mesh_A.vertices;  uA = np.arctan2(vA[:,1], vA[:,0])
    vB = mesh_B.vertices;  uB = np.arctan2(vB[:,1], vB[:,0])

    pts_A, pts_B = [], []
    for ut in u_targets:
        mA = np.abs((uA - ut + np.pi) % (2*np.pi) - np.pi) < tol_rad
        mB = np.abs((uB - ut + np.pi) % (2*np.pi) - np.pi) < tol_rad
        if mA.any(): pts_A.append(vA[mA].mean(axis=0))
        if mB.any(): pts_B.append(vB[mB].mean(axis=0))

    cA = np.mean(pts_A, axis=0) if pts_A else np.array([0.25, 0., 0.05])
    cB = np.mean(pts_B, axis=0) if pts_B else np.array([0.25, 0., -0.05])

    # n=20 → ~20 vertices en section × nb_segments en hauteur >> 4 voisins requis
    return cylinder_between(cA, cB, radius=tube_r, n=20)


# ── Paramètres géométriques ───────────────────────────────────
R         = 0.25
wave_amp  = 0.05
tube_r    = 0.025
strut_r   = 0.013
tendon_r  = 0.006    # tube tendon visible mais fin
phase_A   = 0.0
phase_B   = np.pi
tol_anc   = np.deg2rad(30)

print("Génération des OBJ...")
mesh_A = make_wavy_ring_mesh(R, wave_amp, phase_A, tube_r)
mesh_B = make_wavy_ring_mesh(R, wave_amp, phase_B, tube_r)
mesh_A.export("ring_A.obj");  print("ring_A.obj OK")
mesh_B.export("ring_B.obj");  print("ring_B.obj OK")

mid_u = [np.pi/4 + k*np.pi/2 for k in range(4)]
for i, u_m in enumerate(mid_u):
    pA = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m+phase_A)])
    pB = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m+phase_B)])
    cylinder_between(pA, pB, strut_r).export(f"strut_{i}.obj")
    print(f"strut_{i}.obj OK  pA={np.round(pA,3)}  pB={np.round(pB,3)}")

# T1 : axe X (0° et 180°) — rouge
t1_mesh = make_tendon_mesh(mesh_A, mesh_B,
    u_targets=[0.0, np.pi], tol_rad=tol_anc, tube_r=tendon_r)
t1_mesh.export("tendon_T1.obj")
print(f"tendon_T1.obj OK  ({len(t1_mesh.vertices)} vertices)")

# T2 : axe Y (90° et 270°) — orange
t2_mesh = make_tendon_mesh(mesh_A, mesh_B,
    u_targets=[np.pi/2, 3*np.pi/2], tol_rad=tol_anc, tube_r=tendon_r)
t2_mesh.export("tendon_T2.obj")
print(f"tendon_T2.obj OK  ({len(t2_mesh.vertices)} vertices)")


# ══════════════════════════════════════════════════════════════
# PARTIE 2 — Simulation Genesis
# ══════════════════════════════════════════════════════════════

gs.init(seed=0, precision='32', logging_level='INFO')

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=2e-3,
        substeps=20,
        gravity=(0, 0, 0),
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(0.8, 0.8, 0.6),
        camera_lookat=(0.0, 0.0, 0.0),
        camera_fov=35,
        max_FPS=60,
    ),
    vis_options=gs.options.VisOptions(
        show_world_frame=True,
        n_support_neighbors=4,   # FIX : évite kth out of bounds sur petits meshes
    ),
    show_viewer=True,
)

# ── Matériaux ─────────────────────────────────────────────────
mat_ring  = gs.materials.PBD.Cloth(rho=800.,   stretch_compliance=1e-9,  bending_compliance=1e-6)
mat_strut = gs.materials.PBD.Cloth(rho=1200.,  stretch_compliance=1e-12, bending_compliance=1e-9)
# Tendon : quasi-inextensible (câble dur)
mat_tendon = gs.materials.PBD.Cloth(rho=500.,  stretch_compliance=1e-13, bending_compliance=1e-5)

blue   = (0.15, 0.35, 0.95, 1.0)
dark   = (0.18, 0.18, 0.18, 1.0)
red    = (0.90, 0.15, 0.15, 1.0)
orange = (0.95, 0.55, 0.10, 1.0)

# ── Entités ───────────────────────────────────────────────────
ent_ring_A = scene.add_entity(
    material=mat_ring,
    morph=gs.morphs.Mesh(file="ring_A.obj"),
    surface=gs.surfaces.Default(color=blue, vis_mode="visual"))

ent_ring_B = scene.add_entity(
    material=mat_ring,
    morph=gs.morphs.Mesh(file="ring_B.obj"),
    surface=gs.surfaces.Default(color=blue, vis_mode="visual"))

ent_struts = []
for i in range(4):
    ent = scene.add_entity(
        material=mat_strut,
        morph=gs.morphs.Mesh(file=f"strut_{i}.obj"),
        surface=gs.surfaces.Default(color=dark, vis_mode="visual"))
    ent_struts.append(ent)

# Tendons physiques : ajoutés APRÈS les struts pour avoir leurs edge_start propres
ent_T1 = scene.add_entity(
    material=mat_tendon,
    morph=gs.morphs.Mesh(file="tendon_T1.obj"),
    surface=gs.surfaces.Default(color=red,    vis_mode="visual"))

ent_T2 = scene.add_entity(
    material=mat_tendon,
    morph=gs.morphs.Mesh(file="tendon_T2.obj"),
    surface=gs.surfaces.Default(color=orange, vis_mode="visual"))

scene.build()
scene.reset()

# ── Accès au solver PBD et aux edge ranges des tendons ────────
solver = scene.sim.pbd_solver

e_start_T1 = ent_T1._edge_start
n_edges_T1  = ent_T1.n_edges
e_start_T2  = ent_T2._edge_start
n_edges_T2  = ent_T2.n_edges

print(f"\nTendon T1 : {n_edges_T1} edges (idx {e_start_T1}..{e_start_T1+n_edges_T1-1})")
print(f"Tendon T2 : {n_edges_T2} edges (idx {e_start_T2}..{e_start_T2+n_edges_T2-1})")

# Longueurs de repos initiales (mesurées sur le mesh généré)
len_rest_T1 = np.array([solver.edges_info[e_start_T1+i].len_rest
                         for i in range(n_edges_T1)], dtype=np.float32)
len_rest_T2 = np.array([solver.edges_info[e_start_T2+i].len_rest
                         for i in range(n_edges_T2)], dtype=np.float32)

print(f"len_rest T1 moyen = {len_rest_T1.mean()*1e3:.2f} mm")
print(f"len_rest T2 moyen = {len_rest_T2.mean()*1e3:.2f} mm")

# Positions de référence des struts (pin cinématique)
strut_refs = [e.get_particles_pos().cpu().numpy().copy() for e in ent_struts]

# ── Contrôleur winch : patch len_rest ─────────────────────────
CONTRACTION_MAX = 0.35   # raccourcissement max du tendon (35%)

def set_tendon_lengths(u1: float, u2: float):
    """
    Winch physique : réduit la rest_length de chaque edge du tendon
    proportionnellement à u_k ∈ [0,1].
    u=0 → longueur initiale (tendon relâché)
    u=1 → longueur réduite de CONTRACTION_MAX (tendon pleinement tiré)
    """
    f1 = 1.0 - CONTRACTION_MAX * u1
    f2 = 1.0 - CONTRACTION_MAX * u2
    for i in range(n_edges_T1):
        solver.edges_info[e_start_T1 + i].len_rest = len_rest_T1[i] * f1
    for i in range(n_edges_T2):
        solver.edges_info[e_start_T2 + i].len_rest = len_rest_T2[i] * f2

def pin_struts():
    for ent, ref in zip(ent_struts, strut_refs):
        ent.set_particles_pos(ref.copy())

def measure_tendon_length(ent):
    pos = ent.get_particles_pos().cpu().numpy()
    return np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)) * 1e3  # mm


# ══════════════════════════════════════════════════════════════
# SÉQUENCE 4 PHASES — Fig. 2B(i) du papier GOAT
# ══════════════════════════════════════════════════════════════
#
#  ① Circulaire      u1=1.0, u2=0.0  T1 raccourci,  T2 libre
#  ② Rover           u1=1.0, u2=1.0  T1+T2 raccourcis → ellipse allongée
#  ③ Reconfiguration u1=0.5, u2=0.5  tendons mi-course → forme intermédiaire
#  ④ Sphère          u1=0.0, u2=0.0  tous relâchés → forme repos sphérique

N_TRANS = 50    # steps de transition (easing cosinus)
N_HOLD  = 80    # steps de palier stable (visible dans le viewer)
N_PHASE = N_TRANS + N_HOLD

STATES = [
    (1.0, 0.0),   # ① Circulaire
    (1.0, 1.0),   # ② Rover
    (0.5, 0.5),   # ③ Reconfiguration
    (0.0, 0.0),   # ④ Sphère
]
NAMES   = ["Circulaire", "Rover", "Reconfiguration", "Sphere"]
SYMBOLS = ["①", "②", "③", "④"]

n_total = len(STATES) * N_PHASE
print("\n" + "="*70)
print(f"  Simulation 4 phases — {n_total} steps")
print(f"  {N_TRANS} trans + {N_HOLD} palier / phase   (~{N_PHASE*2e-3*1e3:.0f} ms sim / phase)")
print(f"  Tendons physiques XPBD — patch edges_info.len_rest")
print("="*70)

log = open("four_phases_log.csv", "w", newline="")
writer = csv.writer(log)
writer.writerow(["step","phase","u1","u2","L_T1_mm","L_T2_mm"])

SLEEP = 0.012   # ~1.5 s par phase visible dans le viewer

for step in range(n_total):
    phase_idx     = step // N_PHASE
    step_in_phase = step %  N_PHASE

    u1_tgt, u2_tgt = STATES[phase_idx]

    # Transition lissée (easing cosinus)
    if step_in_phase < N_TRANS:
        u1_prev, u2_prev = STATES[phase_idx-1] if phase_idx > 0 else STATES[0]
        alpha = step_in_phase / N_TRANS
        s = 0.5 * (1 - np.cos(np.pi * alpha))
        u1 = u1_prev + (u1_tgt - u1_prev) * s
        u2 = u2_prev + (u2_tgt - u2_prev) * s
    else:
        u1, u2 = u1_tgt, u2_tgt

    # ★ Solution C : patch des rest_lengths (winch physique)
    set_tendon_lengths(u1, u2)
    pin_struts()
    scene.step()
    time.sleep(SLEEP)

    L1 = measure_tendon_length(ent_T1)
    L2 = measure_tendon_length(ent_T2)
    writer.writerow([step, NAMES[phase_idx],
                     round(u1,4), round(u2,4),
                     round(L1,3), round(L2,3)])

    if step_in_phase == 0:
        print(f"\n{'─'*70}")
        print(f"  {SYMBOLS[phase_idx]} PHASE {phase_idx+1} : "
              f"{NAMES[phase_idx].upper()}  (u1={u1_tgt:.1f}, u2={u2_tgt:.1f})")
        print(f"{'─'*70}")

    if step % 20 == 0:
        b1 = "█" * int(u1 * 20)
        b2 = "█" * int(u2 * 20)
        print(f"  [{step:3d}] u1={u1:.3f} {b1:<20}  "
              f"u2={u2:.3f} {b2:<20}  "
              f"L_T1={L1:6.2f}mm  L_T2={L2:6.2f}mm")

log.close()
print("\nSimulation terminée. Log : four_phases_log.csv")