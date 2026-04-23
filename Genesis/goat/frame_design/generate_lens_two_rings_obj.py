import numpy as np
import trimesh

# ===========================
# Paramètres géométriques
# ===========================
a = 0.40      # demi-longueur de l’ellipse (axe x) [m]
b = 0.20      # demi-largeur de l’ellipse (axe y) [m]
r_rod = 0.006 # rayon des rods (fiberglass ~5 mm) [file:2]
r_strut = 0.008  # rayon des segments noirs (un peu plus rigides visuellement)
z_amp = 0.04  # amplitude de tressage en z (écart max entre les 2 rods) [m]

n_points = 128      # discrétisation le long du tour
n_struts = 8        # nombre de segments noirs

# ===========================
# Génération des deux anneaux
# ===========================
ts = np.linspace(0.0, 2.0 * np.pi, n_points, endpoint=False)

# Anneau A : z(t) = +z_amp * sin(2t)
# Anneau B : z(t) = -z_amp * sin(2t)
# → les deux rods passent alternativement au-dessus / au-dessous, comme “entrelacés”.
ringA_pts = []
ringB_pts = []
for t in ts:
    x = a * np.cos(t)
    y = b * np.sin(t)
    zA = 0.5 * z_amp * np.sin(2.0 * t)
    zB = -0.5 * z_amp * np.sin(2.0 * t)
    ringA_pts.append([x, y, zA])
    ringB_pts.append([x, y, zB])

ringA_pts = np.asarray(ringA_pts)
ringB_pts = np.asarray(ringB_pts)

segments = []
# Cylindres le long de chaque anneau
for pts in (ringA_pts, ringB_pts):
    for i in range(n_points):
        p0 = pts[i]
        p1 = pts[(i + 1) % n_points]
        cyl = trimesh.creation.cylinder(
            radius=r_rod,
            segment=[p0, p1],
            sections=12,
        )
        segments.append(cyl)

# ===========================
# Segments “rigides” noirs (liaisons entre anneaux)
# ===========================
indices_struts = np.linspace(0, n_points, n_struts, endpoint=False, dtype=int)

for idx in indices_struts:
    pA = ringA_pts[idx]
    pB = ringB_pts[idx]
    strut = trimesh.creation.cylinder(
        radius=r_strut,
        segment=[pA, pB],
        sections=12,
    )
    segments.append(strut)

# ===========================
# Assemblage du mesh
# ===========================
frame_mesh = trimesh.util.concatenate(segments)

# Nettoyage léger
mask = frame_mesh.nondegenerate_faces()
frame_mesh.update_faces(mask)
frame_mesh.remove_unreferenced_vertices()

frame_mesh.export("lens_weaved_frame.obj")
print(
    f"Wrote lens_weaved_frame.obj — "
    f"{len(frame_mesh.vertices)} vertices, {len(frame_mesh.faces)} faces."
)
print("Is watertight:", frame_mesh.is_watertight)