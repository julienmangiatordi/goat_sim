import numpy as np
import trimesh

# ===========================
# Paramètres — Fig. 2B(ii)
# ===========================
R      = 0.32    # rayon anneau [m]
r_rod  = 0.006   # rayon rod (fiberglass ~5 mm)
r_strut = 0.008  # rayon des struts (liaisons)
n_pts  = 120     # points par anneau
n_sec  = 14      # sections du tube
n_struts = 8     # nombre de struts

thetas = np.linspace(0, 2*np.pi, n_pts, endpoint=False)

# ===== Ring A : plan X-Z =====
ringA = np.stack([
    R * np.cos(thetas),
    np.zeros(n_pts),
    R * np.sin(thetas),
], axis=-1)

# ===== Ring B : plan Y-Z (perpendiculaire) =====
ringB = np.stack([
    np.zeros(n_pts),
    R * np.cos(thetas),
    R * np.sin(thetas),
], axis=-1)

def make_tube(pts, radius):
    """Crée un tube en concaténant des cylindres entre points consécutifs."""
    segs = []
    for i in range(len(pts)):
        p0 = pts[i]
        p1 = pts[(i+1) % len(pts)]
        segs.append(trimesh.creation.cylinder(
            radius=radius, segment=[p0, p1], sections=n_sec))
    return segs

# Tubes pour les deux anneaux
meshes = make_tube(ringA, r_rod) + make_tube(ringB, r_rod)

# ===== Struts : liaisons entre les deux rings =====
strut_angles = np.linspace(0, 2*np.pi, n_struts, endpoint=False)
for angle in strut_angles:
    # Point sur Ring A
    pA = np.array([R*np.cos(angle), 0.0, R*np.sin(angle)])
    # Point correspondant sur Ring B
    pB = np.array([0.0, R*np.cos(angle), R*np.sin(angle)])
    # Strut reliant les deux
    strut = trimesh.creation.cylinder(
        radius=r_strut, segment=[pA, pB], sections=16)
    meshes.append(strut)

# Assemblage final
frame = trimesh.util.concatenate(meshes)
frame.export("goat_lens.obj")
print(f"Wrote goat_lens.obj — {len(frame.vertices)} v, {len(frame.faces)} f")
