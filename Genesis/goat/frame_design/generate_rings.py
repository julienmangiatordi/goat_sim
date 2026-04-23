# generate_rings.py
# Génère 4 fichiers OBJ : ring_A, ring_B, strut_top, strut_bot

import numpy as np
import trimesh

R = 0.32
r_tube = 0.015   # épaisseur des rings
r_strut = 0.020  # rayon des struts
angle_deg = 15
n_maj = 32
n_min = 10


def make_ring(angle_deg_val):
    # Torus de base
    torus = trimesh.creation.torus(
        major_radius=R,
        minor_radius=r_tube,
        major_sections=n_maj,
        minor_sections=n_min,
    )

    # Inclinée de +/- angle_deg autour de Y
    Ry = trimesh.transformations.rotation_matrix(
        np.radians(angle_deg_val), [0, 1, 0]
    )

    torus.apply_transform(Ry)
    torus.fill_holes()
    assert torus.is_watertight
    return torus


def export_mesh(mesh, filename):
    mesh.export(filename)
    print(f"{filename}: OK, watertight={mesh.is_watertight}")


def vertical_span_between_rings(ringA, ringB, y_target, eps=0.02):
    """
    Cherche tous les sommets des deux rings dont la coordonnée y est
    proche de y_target, et renvoie l'étendue verticale (z_min, z_max).
    """
    vertsA = ringA.vertices
    vertsB = ringB.vertices

    maskA = np.abs(vertsA[:, 1] - y_target) < eps
    maskB = np.abs(vertsB[:, 1] - y_target) < eps

    ptsA = vertsA[maskA]
    ptsB = vertsB[maskB]

    if ptsA.shape[0] == 0 or ptsB.shape[0] == 0:
        raise RuntimeError(f"Aucun point de ring trouvé près de y={y_target:.3f}")

    z_min = min(ptsA[:, 2].min(), ptsB[:, 2].min())
    z_max = max(ptsA[:, 2].max(), ptsB[:, 2].max())

    return z_min, z_max


def make_vertical_strut(ringA, ringB, y_target, filename):
    z_min, z_max = vertical_span_between_rings(ringA, ringB, y_target)
    length = z_max - z_min
    center_z = 0.5 * (z_min + z_max)

    # Strut vertical (axe Z) de longueur = plus grand écart vertical
    p0 = np.array([y_target, 0.0, center_z - 1 * length])
    p1 = np.array([y_target, 0.0, center_z + 1 * length])

    cyl = trimesh.creation.cylinder(
        radius=r_strut / 2,
        segment=[p0, p1],
        sections=16,
    )
    cyl.fill_holes()
    cyl.export(filename)
    print(f"{filename}: watertight={cyl.is_watertight}, length={length:.3f} m")


# --- génération des 2 rings ---
ring_A = make_ring(+angle_deg)
ring_B = make_ring(-angle_deg)

export_mesh(ring_A, "ring_A.obj")
export_mesh(ring_B, "ring_B.obj")

# --- struts aux extrémités les plus éloignées des zones de croisement ---
# Les croisement sont autour de y≈0,  struts à y=+R et y=-R
make_vertical_strut(ring_A, ring_B, +R, "strut_top.obj")
make_vertical_strut(ring_A, ring_B, -R, "strut_bot.obj")
