import trimesh
import numpy as np
import genesis as gs
import csv


# ══════════════════════════════logs════════════════════════════
log_file = open("tendon_log.csv", "w", newline="")
writer   = csv.writer(log_file)
writer.writerow(["step", "u1", "u2", "f1_residual", "f2_residual", "frame_z"])
# ══════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════
# PARTIE 1 — Génération des OBJ
# ══════════════════════════════════════════════════════════════

def make_wavy_ring_mesh(R, wave_amp, phase, tube_r, n_major=80, n_tube=12):
    u = np.linspace(0, 2*np.pi, n_major, endpoint=False)
    v = np.linspace(0, 2*np.pi, n_tube, endpoint=False)
    U, V = np.meshgrid(u, v)

    cx = R * np.cos(U)
    cy = R * np.sin(U)
    cz = wave_amp * np.sin(2*U + phase)

    tx = -R * np.sin(U)
    ty =  R * np.cos(U)
    tz =  2 * wave_amp * np.cos(2*U + phase)
    t_norm = np.sqrt(tx**2 + ty**2 + tz**2) + 1e-12
    tx, ty, tz = tx/t_norm, ty/t_norm, tz/t_norm

    bx =  ty
    by = -tx
    bz = np.zeros_like(tx)
    b_norm = np.sqrt(bx**2 + by**2) + 1e-12
    bx, by = bx/b_norm, by/b_norm

    nx = by*tz - bz*ty
    ny = bz*tx - bx*tz
    nz = bx*ty - by*tx

    px = cx + tube_r*(np.cos(V)*nx + np.sin(V)*bx)
    py = cy + tube_r*(np.cos(V)*ny + np.sin(V)*by)
    pz = cz + tube_r*(np.cos(V)*nz + np.sin(V)*bz)

    vertices = np.stack([px.flatten(), py.flatten(), pz.flatten()], axis=1)
    nU, nV = n_major, n_tube
    faces = []
    for i in range(nV):
        for j in range(nU):
            v0 = i*nU + j
            v1 = i*nU + (j+1)%nU
            v2 = ((i+1)%nV)*nU + (j+1)%nU
            v3 = ((i+1)%nV)*nU + j
            faces += [[v0,v1,v2],[v0,v2,v3]]
    return trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=True)


def cylinder_between(p1, p2, radius, n=12):
    p1, p2 = np.array(p1, dtype=float), np.array(p2, dtype=float)
    vec = p2 - p1
    length = np.linalg.norm(vec)
    mid = (p1 + p2) / 2.0
    cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=n)
    z_axis = np.array([0.0, 0.0, 1.0])
    vec_n = vec / length
    if np.abs(np.dot(z_axis, vec_n)) < 0.9999:
        axis = np.cross(z_axis, vec_n)
        axis /= np.linalg.norm(axis)
        angle = np.arccos(np.clip(np.dot(z_axis, vec_n), -1, 1))
        R_mat = trimesh.transformations.rotation_matrix(angle, axis)
        cyl.apply_transform(R_mat)
    cyl.apply_translation(mid)
    return cyl


# ── Paramètres géométriques ──────────────────────────────────
R        = 0.25
wave_amp = 0.05
tube_r   = 0.025
strut_r  = 0.013
phase_A  = 0.0
phase_B  = np.pi

print("Génération des OBJ...")

ring_A_mesh = make_wavy_ring_mesh(R, wave_amp, phase_A, tube_r)
ring_A_mesh.export("ring_A.obj")
print("ring_A.obj OK")

ring_B_mesh = make_wavy_ring_mesh(R, wave_amp, phase_B, tube_r)
ring_B_mesh.export("ring_B.obj")
print("ring_B.obj OK")

mid_u = [np.pi/4 + k*np.pi/2 for k in range(4)]
for i, u_m in enumerate(mid_u):
    pA = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m + phase_A)])
    pB = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m + phase_B)])
    s = cylinder_between(pA, pB, strut_r)
    s.export(f"strut_{i}.obj")
    print(f"strut_{i}.obj OK  pA={np.round(pA,3)}  pB={np.round(pB,3)}")


# ══════════════════════════════════════════════════════════════
# PARTIE 2 — Simulation Genesis
# ══════════════════════════════════════════════════════════════

gs.init(seed=0, precision='32', logging_level='debug')

spawn_z = 0.6

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=4e-3,
        substeps=10,
        gravity=(0, 0, -9.81),
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.5, 1.0, 0.8),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=40,
    ),
    vis_options=gs.options.VisOptions(show_world_frame=True),
    show_viewer=True,
)

scene.add_entity(morph=gs.morphs.Plane())

blue = (0.15, 0.35, 0.95, 1.0)
dark = (0.18, 0.18, 0.18, 1.0)

mat_ring = gs.materials.PBD.Cloth(
    rho=800.0,
    stretch_compliance=1e-9,
    bending_compliance=1e-6,
)
mat_strut = gs.materials.PBD.Cloth(
    rho=1200.0,
    stretch_compliance=1e-12,
    bending_compliance=1e-9,
)

ent_ring_A = scene.add_entity(
    material=mat_ring,
    morph=gs.morphs.Mesh(file='ring_A.obj', pos=(0.0, 0.0, spawn_z),
                         euler=(90.0, 0.0, 0.0), scale=1.0),
    surface=gs.surfaces.Default(color=blue, vis_mode='visual'),
)
ent_ring_B = scene.add_entity(
    material=mat_ring,
    morph=gs.morphs.Mesh(file='ring_B.obj', pos=(0.0, 0.0, spawn_z),
                         euler=(90.0, 0.0, 0.0), scale=1.0),
    surface=gs.surfaces.Default(color=blue, vis_mode='visual'),
)
ent_struts = []
for i in range(4):
    ent = scene.add_entity(
        material=mat_strut,
        morph=gs.morphs.Mesh(file=f'strut_{i}.obj', pos=(0.0, 0.0, spawn_z),
                             euler=(90.0, 0.0, 0.0), scale=1.0),
        surface=gs.surfaces.Default(color=dark, vis_mode='visual'),
    )
    ent_struts.append(ent)

all_entities = [ent_ring_A, ent_ring_B] + ent_struts

scene.build()
scene.reset()

# ── Positions de référence (forme neutre après reset) ─────────
posA_ref = ent_ring_A.get_particles_pos().cpu().numpy().copy()
posB_ref = ent_ring_B.get_particles_pos().cpu().numpy().copy()

centerA = posA_ref.mean(0)
centerB = posB_ref.mean(0)

# ── Indices d'ancrage des tendons ────────────────────────────
u_A = np.arctan2(posA_ref[:, 1], posA_ref[:, 0])
u_B = np.arctan2(posB_ref[:, 1], posB_ref[:, 0])

u_targets = [np.pi/4 + k*np.pi/2 for k in range(4)]
tol = 0.10

anchor_idx_A = [[] for _ in range(2)]
anchor_idx_B = [[] for _ in range(2)]

for i, ut in enumerate(u_targets):
    tendon_id = 0 if i in (0, 2) else 1
    mask_A = np.abs((u_A - ut + np.pi) % (2*np.pi) - np.pi) < tol
    mask_B = np.abs((u_B - ut + np.pi) % (2*np.pi) - np.pi) < tol
    anchor_idx_A[tendon_id].extend(np.where(mask_A)[0].tolist())
    anchor_idx_B[tendon_id].extend(np.where(mask_B)[0].tolist())

anchor_idx_A = [np.array(idx, dtype=int) for idx in anchor_idx_A]
anchor_idx_B = [np.array(idx, dtype=int) for idx in anchor_idx_B]

print(f"Ancres tendon0 : {len(anchor_idx_A[0])} particules sur A, "
      f"{len(anchor_idx_B[0])} sur B")
print(f"Ancres tendon1 : {len(anchor_idx_A[1])} particules sur A, "
      f"{len(anchor_idx_B[1])} sur B")


# ── Fonction de déplacement — part TOUJOURS de posA/B_ref ────
def apply_tendon_displacement(u1, u2):
    posA_np = posA_ref.copy()   # ← clé : on repart de la géométrie de référence
    posB_np = posB_ref.copy()

    for k, u in enumerate((u1, u2)):
        if u <= 0:
            continue

        idxA = anchor_idx_A[k]
        idxB = anchor_idx_B[k]

        dirA = centerA - posA_np[idxA]
        dirB = centerB - posB_np[idxB]
        dirA /= (np.linalg.norm(dirA, axis=1, keepdims=True) + 1e-8)
        dirB /= (np.linalg.norm(dirB, axis=1, keepdims=True) + 1e-8)

        amp = 0.03 * u   # 3 cm max

        posA_np[idxA] += amp * dirA
        posB_np[idxB] += amp * dirB

    ent_ring_A.set_particles_pos(posA_np)
    ent_ring_B.set_particles_pos(posB_np)

def measure_tendon_forces():
    """
    Mesure la résistance du frame aux tendons :
    force ≈ ||pos_imposée - pos_effective|| / compliance
    Renvoie (f1, f2) en unités arbitraires.
    """
    posA_actual = ent_ring_A.get_particles_pos().cpu().numpy()
    posB_actual = ent_ring_B.get_particles_pos().cpu().numpy()

    forces = []
    for k in range(2):
        idxA = anchor_idx_A[k]
        idxB = anchor_idx_B[k]

        # déplacement résiduel = écart entre où on a poussé et où le solver a mis les particules
        residA = posA_actual[idxA] - posA_ref[idxA]
        residB = posB_actual[idxB] - posB_ref[idxB]

        # projection sur la direction de traction (vers le centre)
        dirA = centerA - posA_ref[idxA]
        dirB = centerB - posB_ref[idxB]
        dirA /= (np.linalg.norm(dirA, axis=1, keepdims=True) + 1e-8)
        dirB /= (np.linalg.norm(dirB, axis=1, keepdims=True) + 1e-8)

        projA = (residA * dirA).sum(axis=1).mean()   # déplacement moyen projeté [m]
        projB = (residB * dirB).sum(axis=1).mean()

        forces.append((projA + projB) / 2.0)

    return forces[0], forces[1]


# ── Mesure du rebond ──────────────────────────────────────────
def get_frame_z(entities):
    zs = []
    for ent in entities:
        pos_np = ent.get_particles_pos().cpu().numpy()
        if pos_np.ndim == 3:
            pos_np = pos_np[0]
        zs.append(pos_np[:, 2].mean())
    return np.mean(zs)


# ══════════════════════════════════════════════════════════════
# BOUCLE PRINCIPALE — Phase 0 : reconfig | Phase 1 : chute
# ══════════════════════════════════════════════════════════════

n_conf  = 200    # steps de reconfiguration (morphing quasi-statique)
n_total = 4000   # steps totaux (reste = chute + rebond)

z_min_prev  = spawn_z
z_max_after = 0.0
impact_done = False
rebond_done = False
falling     = False   # False pendant reconfig, True dès la chute
u1_final = u2_final = 0.0

print("=" * 55)
print(f"Phase 0 : reconfiguration sur {n_conf} steps")
print(f"Phase 1 : chute libre sur {n_total - n_conf} steps")
print("=" * 55)

for i in range(n_total):

    # ── PHASE 0 : reconfiguration ─────────────────────────────
    if i < n_conf:
        t = i / (n_conf - 1)

        # Phase 1→2 : Circular → Rover (t in [0, 0.4])
        # Tendon 1 reste constant (~1.0 normalisé), Tendon 2 monte jusqu'à 1.0+
        if t < 0.4:
            alpha = t / 0.4
            u1 = 1.0                        # T1 stable (long)
            u2 = 0.0 + 1.0 * alpha         # T2 monte de 0 → 1.0

        # Phase 2→3 : Rover → Reconfiguring (t in [0.4, 0.6])
        # Les deux commencent à descendre, T2 descend plus vite
        elif t < 0.6:
            beta = (t - 0.4) / 0.2
            u1 = 1.0 - 0.4 * beta          # T1 descend légèrement : 1.0 → 0.6
            u2 = 1.0 - 0.5 * beta          # T2 descend : 1.0 → 0.5

        # Phase 3→4 : Reconfiguring → Sphere (t in [0.6, 1.0])
        # Les deux descendent vers 0 (out-of-plane folding)
        else:
            gamma = (t - 0.6) / 0.4
            u1 = 0.6 - 0.6 * gamma         # T1 : 0.6 → 0.0
            u2 = 0.5 - 0.5 * gamma         # T2 : 0.5 → 0.0

        apply_tendon_displacement(u1, u2)
        scene.step()

        # mesure après step
        f1, f2 = measure_tendon_forces()
        z      = get_frame_z(all_entities)

        writer.writerow([i, round(u1, 4), round(u2, 4),
                         round(f1*1000, 4), round(f2*1000, 4), round(z, 4)])

        # affichage console toutes les 20 steps
        if i % 20 == 0:
            bar1 = "█" * int(abs(f1) * 5000)
            bar2 = "█" * int(abs(f2) * 5000)
            print(f"[{i:4d}] u1={u1:.3f} u2={u2:.3f} | "
                  f"T1: {bar1:<20} {f1*1000:+6.2f} mm | "
                  f"T2: {bar2:<20} {f2*1000:+6.2f} mm")

    # ── PHASE 1 : chute libre (tendons figés) ─────────────────
    else:
        if i == n_conf:
            print(f"[step {i}] Début de la chute libre")
            z_min_prev = get_frame_z(all_entities)   # hauteur réelle au début de la chute

        z = get_frame_z(all_entities)

        # Détection de l'impact
        if falling and z < z_min_prev:
            z_min_prev = z
        elif falling and not impact_done and z > z_min_prev + 0.005:
            impact_done = True
            falling     = False
            z_impact    = z_min_prev
            print(f"[step {i}] Impact — z_min = {z_impact:.4f} m")

        # Détection du sommet du rebond
        if impact_done and not rebond_done:
            if z > z_max_after:
                z_max_after = z
            elif z < z_max_after - 0.005:
                rebond_done = True
                h0 = z_min_prev - z_impact      # hauteur de chute effective
                h1 = z_max_after - z_impact     # hauteur de rebond
                e  = np.sqrt(max(h1, 0) / h0) if h0 > 0 else 0
                print(f"[step {i}] Rebond terminé")
                print(f"  h0 (chute)  = {h0:.4f} m")
                print(f"  h1 (rebond) = {h1:.4f} m")
                print(f"  e  (coeff. restitution)  = {e:.3f}")
                print(f"  Énergie conservée        = {(h1/h0)*100:.1f}%")
                print(f"  Référence paper (sphère) : ~50% → e ≈ 0.71")

    scene.step()





    """ z = get_frame_z(all_entities)

    if falling and z < z_min_prev:
        z_min_prev = z
    elif falling and z > z_min_prev + 0.005:
        impact_done = True
        falling     = False
        z_impact    = z_min_prev
        print(f"[step {i:5d}] Impact — z_min = {z_impact:.4f} m")

    if impact_done and not rebond_done:
        if z > z_max_after:
            z_max_after = z
        elif z < z_max_after - 0.005:
            rebond_done = True
            h0 = spawn_z  - z_impact
            h1 = z_max_after - z_impact
            e  = np.sqrt(h1 / h0) if h0 > 0 else 0
            print(f"[step {i:5d}] Rebond terminé")
            print(f"  h0 (chute)  = {h0:.4f} m")
            print(f"  h1 (rebond) = {h1:.4f} m")
            print(f"  e  (coeff. restitution)  = {e:.3f}")
            print(f"  Énergie conservée        = {(h1/h0)*100:.1f}%")
            print(f"  Référence paper (sphère) : ~50% → e ≈ 0.71") """