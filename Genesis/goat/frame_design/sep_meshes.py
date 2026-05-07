import numpy as np
import trimesh

major_a = 0.27
major_b = 0.14
tube_r  = 0.025   # assez épais pour PBD (5x le vrai rayon, ok pour sim)
n_major = 80
n_tube  = 12

def make_torus_oval(major_a, major_b, tube_r, n_major=80, n_tube=12):
    u = np.linspace(0, 2*np.pi, n_major, endpoint=False)
    v = np.linspace(0, 2*np.pi, n_tube, endpoint=False)
    U, V = np.meshgrid(u, v)

    cx = major_a * np.cos(U)
    cy = np.zeros_like(U)
    cz = major_b * np.sin(U)

    tx = -major_a * np.sin(U)
    tz =  major_b * np.cos(U)
    t_norm = np.sqrt(tx**2 + tz**2) + 1e-12
    tx, tz = tx/t_norm, tz/t_norm
    ty = np.zeros_like(U)

    bx =  tz; by = np.zeros_like(U); bz = -tx
    nx = by*tz - bz*ty
    ny = bz*tx - bx*tz
    nz = bx*ty - by*tx

    px = cx + tube_r*(np.cos(V)*nx + np.sin(V)*bx)
    py = cy + tube_r*(np.cos(V)*ny + np.sin(V)*by)
    pz = cz + tube_r*(np.cos(V)*nz + np.sin(V)*bz)

    verts = np.stack([px.flatten(), py.flatten(), pz.flatten()], axis=1)
    nU, nV = n_major, n_tube
    faces = []
    for i in range(nV):
        for j in range(nU):
            v0 = i*nU + j
            v1 = i*nU + (j+1)%nU
            v2 = ((i+1)%nV)*nU + (j+1)%nU
            v3 = ((i+1)%nV)*nU + j
            faces += [[v0,v1,v2],[v0,v2,v3]]
    return trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=True)

def make_strut(p_start, p_end, radius=0.020, n=12):
    """Cylindre entre deux points 3D."""
    vec = p_end - p_start
    length = np.linalg.norm(vec)
    cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=n)
    direction = vec / length
    z_axis = np.array([0, 0, 1.0])
    axis = np.cross(z_axis, direction)
    axis_norm = np.linalg.norm(axis)
    if axis_norm > 1e-6:
        axis /= axis_norm
        angle = np.arccos(np.clip(np.dot(z_axis, direction), -1, 1))
        T = trimesh.transformations.rotation_matrix(angle, axis)
    else:
        T = np.eye(4) if np.dot(z_axis, direction) > 0 else \
            trimesh.transformations.rotation_matrix(np.pi, [1,0,0])
    T[:3, 3] = 0.5 * (p_start + p_end)
    cyl.apply_transform(T)
    return cyl

# ── ring_A : plan XZ ──────────────────────────────────────────────────────────
ring_A = make_torus_oval(major_a, major_b, tube_r)
ring_A.export("ring_A.obj")
print(f"ring_A.obj : {len(ring_A.vertices)} vertices")

# ── ring_B : ring_A tourné 90° autour Z → plan YZ ────────────────────────────
ring_B = ring_A.copy()
ring_B.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0,0,1]))
ring_B.export("ring_B.obj")
print(f"ring_B.obj : {len(ring_B.vertices)} vertices")

# ── 4 struts correctement positionnés ─────────────────────────────────────────
# Chaque strut relie le milieu d'arc de ring_A au milieu d'arc correspondant de ring_B
# Angles : 45°, 135°, 225°, 315° (entre chaque croisement)
angles = [45, 135, 225, 315]
for idx, t_deg in enumerate(angles):
    t = np.radians(t_deg)
    # Point sur ring_A (plan XZ) à cet angle
    pA = np.array([major_a*np.cos(t), 0.0,            major_b*np.sin(t)])
    # Point correspondant sur ring_B (plan YZ) : swap x↔y
    pB = np.array([0.0,               major_a*np.cos(t), major_b*np.sin(t)])
    strut = make_strut(pA, pB, radius=0.020)
    fname = f"strut_{idx}.obj"
    strut.export(fname)
    L = np.linalg.norm(pB - pA)
    print(f"{fname} : centre={0.5*(pA+pB).round(3)}, L={L:.3f} m")

print("\nTous les OBJ générés.")