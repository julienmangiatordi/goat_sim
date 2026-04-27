"""
Generate GOAT Lens Frame OBJ File

Crée le fichier mesh .obj du frame en fibre de verre pour le GOAT.
Ce fichier peut être utilisé indépendamment dans Genesis ou d'autres simulateurs.

Usage:
    python generate_lens_frame_genesis.py                    # Génère avec paramètres par défaut
    python generate_lens_frame_genesis.py --radius 0.35      # Rayon custom
    python generate_lens_frame_genesis.py --rod-radius 0.006 # Rayon tube custom
"""

import argparse
import math
import numpy as np


def generate_frame_mesh(R, r_rod, n_pts, n_sections, n_struts, output_file):
    """
    Génère le mesh du frame lens et l'exporte en OBJ.
    
    Utilise uniquement NumPy pour éviter les dépendances externes.
    Crée des cylindres en combinant des triangles.
    
    Args:
        R: rayon des anneaux
        r_rod: rayon des tubes
        n_pts: points de discrétisation par anneau
        n_sections: sections des cylindres
        n_struts: nombre de struts
        output_file: chemin du fichier OBJ à créer
    """
    
    vertices = []
    faces = []
    vertex_count = 0
    
    def add_cylinder(p_start, p_end, radius, sections):
        """Ajoute un cylindre au mesh."""
        nonlocal vertex_count
        
        # Direction et longueur
        direction = p_end - p_start
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return
        
        direction = direction / length
        
        # Axes perpendiculaires
        if abs(direction[2]) < 0.9:
            up = np.array([0, 0, 1])
        else:
            up = np.array([1, 0, 0])
        
        right = np.cross(direction, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, direction)
        
        # Générer points du cylindre
        angles = np.linspace(0, 2*np.pi, sections, endpoint=False)
        
        for z in [0, 1]:
            p_center = p_start + z * direction * length
            for angle in angles:
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                point = p_center + x * right + y * up
                vertices.append(point)
        
        # Triangles
        for i in range(sections):
            i1 = vertex_count + i
            i2 = vertex_count + (i + 1) % sections
            i3 = vertex_count + sections + i
            i4 = vertex_count + sections + (i + 1) % sections
            
            faces.append([i1 + 1, i2 + 1, i3 + 1])
            faces.append([i2 + 1, i4 + 1, i3 + 1])
        
        vertex_count += 2 * sections
    
    print(f"Generating frame mesh...")
    print(f"  Ring radius: {R:.4f} m")
    print(f"  Rod radius: {r_rod:.4f} m")
    print(f"  Points per ring: {n_pts}")
    print(f"  Cylinder sections: {n_sections}")
    print(f"  Struts: {n_struts}")
    
    # Discrétiser les anneaux
    theta = np.linspace(0, 2*np.pi, n_pts, endpoint=False)
    
    # Ring A (X-Z plane)
    ptA_ring = np.array([
        [R * np.cos(t), 0.0, R * np.sin(t)] for t in theta
    ])
    
    # Ring B (Y-Z plane)
    ptB_ring = np.array([
        [0.0, R * np.cos(t), R * np.sin(t)] for t in theta
    ])
    
    print(f"Adding ring A (X-Z plane) tubes...")
    for i in range(n_pts):
        p0 = ptA_ring[i]
        p1 = ptA_ring[(i + 1) % n_pts]
        add_cylinder(p0, p1, r_rod, n_sections)
    
    print(f"Adding ring B (Y-Z plane) tubes...")
    for i in range(n_pts):
        p0 = ptB_ring[i]
        p1 = ptB_ring[(i + 1) % n_pts]
        add_cylinder(p0, p1, r_rod, n_sections)
    
    # Struts
    print(f"Adding {n_struts} struts...")
    r_strut = r_rod * 1.33
    strut_angles = np.linspace(0, 2*np.pi, n_struts, endpoint=False)
    
    for angle in strut_angles:
        pA = np.array([R * np.cos(angle), 0.0, R * np.sin(angle)])
        pB = np.array([0.0, R * np.cos(angle), R * np.sin(angle)])
        add_cylinder(pA, pB, r_strut, 16)
    
    # Exporter OBJ
    print(f"Writing {output_file}...")
    with open(output_file, 'w') as f:
        f.write(f"# GOAT Lens Frame Mesh\n")
        f.write(f"# Ring radius: {R}\n")
        f.write(f"# Rod radius: {r_rod}\n")
        f.write(f"# Total vertices: {len(vertices)}\n")
        f.write(f"# Total faces: {len(faces)}\n")
        f.write(f"\n")
        
        for v in vertices:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")
    
    print(f"✓ Done!")
    print(f"  Vertices: {len(vertices)}")
    print(f"  Faces: {len(faces)}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate GOAT lens frame OBJ mesh for Genesis simulation"
    )
    parser.add_argument('--radius', type=float, default=0.32,
                        help='Frame ring radius [m] (default: 0.32)')
    parser.add_argument('--rod-radius', type=float, default=0.005,
                        help='Tube radius [m] (default: 0.005 = 5mm)')
    parser.add_argument('--points', type=int, default=60,
                        help='Points per ring for discretization (default: 60)')
    parser.add_argument('--sections', type=int, default=8,
                        help='Cylinder section count (default: 8)')
    parser.add_argument('--struts', type=int, default=6,
                        help='Number of connecting struts (default: 6)')
    parser.add_argument('--output', type=str, default='goat_lens_genesis.obj',
                        help='Output OBJ file path (default: goat_lens_genesis.obj)')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("GOAT Lens Frame Mesh Generator for Genesis")
    print("=" * 70)
    print()
    
    # Paramètres par défaut du GOAT
    R_FRAME = 4.0 / (2 * math.pi)  # 0.6366 m selon le papier
    
    # Utiliser les arguments ou les défauts
    R = args.radius if args.radius != 0.32 else R_FRAME
    r_rod = args.rod_radius
    n_pts = args.points
    n_sections = args.sections
    n_struts = args.struts
    output = args.output
    
    generate_frame_mesh(R, r_rod, n_pts, n_sections, n_struts, output)
    
    print()
    print("Frame mesh ready for Genesis simulation!")
    print(f"Load it with: gs.morphs.Mesh(file='{output}', ...)")


if __name__ == "__main__":
    main()
