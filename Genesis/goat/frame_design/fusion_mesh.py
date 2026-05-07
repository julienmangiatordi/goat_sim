import trimesh
import numpy as np

def make_wavy_ring_mesh(R, wave_amp, phase, tube_r, n_major=200, n_tube=20):
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

    # Repère stable : binormale = t × Z
    bx =  ty   # t × (0,0,1) = (ty, -tx, 0)
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


# ── Paramètres ──────────────────────────────────────────────
R        = 0.25
wave_amp = 0.05
tube_r   = 0.025
strut_r  = 0.013

phase_A = 0.0
phase_B = np.pi   # ← CLÉ : opposé → croisements à z=0 pour les 4 points

# ── Anneaux ─────────────────────────────────────────────────
ring_A = make_wavy_ring_mesh(R, wave_amp, phase_A, tube_r)
ring_A.export("ring_A.obj")
print("ring_A.obj OK")

ring_B = make_wavy_ring_mesh(R, wave_amp, phase_B, tube_r)
ring_B.export("ring_B.obj")
print("ring_B.obj OK")

# ── 4 struts : milieu entre croisements, reliant ring_A à ring_B ──
# u_mid = pi/4, 3pi/4, 5pi/4, 7pi/4
# pA et pB ont même x,y mais z opposés (+amp et -amp)
mid_u = [np.pi/4 + k*np.pi/2 for k in range(4)]

struts = []
for i, u_m in enumerate(mid_u):
    pA = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m + phase_A)])
    pB = np.array([R*np.cos(u_m), R*np.sin(u_m), wave_amp*np.sin(2*u_m + phase_B)])
    s = cylinder_between(pA, pB, strut_r)
    name = f"strut_{i}.obj"
    s.export(name)
    struts.append(s)
    print(f"strut_{i}: pA={np.round(pA,3)} pB={np.round(pB,3)}")

combined = trimesh.util.concatenate([ring_A, ring_B] + struts)
combined.export("goat_lens.obj")
print("goat_lens.obj OK — vertices:", len(combined.vertices))