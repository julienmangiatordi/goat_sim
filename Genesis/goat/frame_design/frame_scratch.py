"""
Frame GOAT Fiberglass Lens - Genesis Deformable Beams Simulation

Objectif: Simuler le frame morphe du GOAT comme deux anneaux croisés en fibre de verre
en utilisant les deformable beams (FEM solver) de Genesis.

Architecture:
- Anneau A : Plan X-Z (formant un cercle vertical sur cet axe)
- Anneau B : Plan Y-Z (formant un cercle vertical perpendiculaire)
- Les deux anneaux se croisent à 4 points pour former une structure "lens"
- Struts: Liaisons entre les anneaux pour rigidité
- Contacts: La structure peut se déformer au contact avec des obstacles

Papier: Figure 2.B - structure lens formée par deux anneaux croisés
"""

import math
import numpy as np
import genesis as gs


def create_lens_frame_mesh(R=0.32, r_rod=0.006, n_pts=120, n_sections=14, n_struts=8,
                            export_obj=None):
    """
    Crée la géométrie mesh pour le frame lens du GOAT.
    
    Args:
        R: rayon des anneaux [m]
        r_rod: rayon des tubes en fibre de verre [m]
        n_pts: points de discrétisation par anneau
        n_sections: sections du cylindre pour les tubes
        n_struts: nombre de struts reliant les deux anneaux
        export_obj: chemin pour exporter le mesh en OBJ (optionnel)
    
    Returns:
        Tuple (vertices, faces) du mesh
    """
    try:
        import trimesh
    except ImportError:
        print("Warning: trimesh not available. Using simplified geometry.")
        return None

    theta = np.linspace(0, 2*np.pi, n_pts, endpoint=False)
    
    # Ring A : plan X-Z
    ringA = np.stack([
        R * np.cos(theta),
        np.zeros(n_pts),
        R * np.sin(theta),
    ], axis=-1)
    
    # Ring B : plan Y-Z (perpendiculaire)
    ringB = np.stack([
        np.zeros(n_pts),
        R * np.cos(theta),
        R * np.sin(theta),
    ], axis=-1)
    
    meshes = []
    
    # Tubes pour les anneaux
    for i in range(len(ringA)-1):
        p0, p1 = ringA[i], ringA[i+1]
        tube_a = trimesh.creation.cylinder(
            radius=r_rod, segment=[p0, p1], sections=n_sections
        )
        meshes.append(tube_a)
        
        p0, p1 = ringB[i], ringB[i+1]
        tube_b = trimesh.creation.cylinder(
            radius=r_rod, segment=[p0, p1], sections=n_sections
        )
        meshes.append(tube_b)
    
    # Fermer les anneaux
    p0, p1 = ringA[-1], ringA[0]
    tube_a = trimesh.creation.cylinder(
        radius=r_rod, segment=[p0, p1], sections=n_sections
    )
    meshes.append(tube_a)
    
    p0, p1 = ringB[-1], ringB[0]
    tube_b = trimesh.creation.cylinder(
        radius=r_rod, segment=[p0, p1], sections=n_sections
    )
    meshes.append(tube_b)
    
    # Struts reliant les anneaux
    strut_angles = np.linspace(0, 2*np.pi, n_struts, endpoint=False)
    r_strut = r_rod * 1.33  # Légèrement plus gros pour la rigidité
    
    for angle in strut_angles:
        pA = np.array([R*np.cos(angle), 0.0, R*np.sin(angle)])
        pB = np.array([0.0, R*np.cos(angle), R*np.sin(angle)])
        strut = trimesh.creation.cylinder(
            radius=r_strut, segment=[pA, pB], sections=16
        )
        meshes.append(strut)
    
    # Assemblage
    frame = trimesh.util.concatenate(meshes)
    
    if export_obj:
        frame.export(export_obj)
        print(f"✓ Exported mesh: {export_obj}")
        print(f"  Vertices: {len(frame.vertices)}, Faces: {len(frame.faces)}")
    
    return frame


# ═══════════════════════════════════════════════════════════════════════════════
# SIMULATION AVEC GENESIS
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    # Initialisation Genesis
    print("Initializing Genesis...")
    gs.init(seed=0, precision="32", logging_level="info")
    
    # Paramètres géométriques
    R_FRAME = 4.0 / (2 * math.pi)  # 0.6366 m
    
    # Créer le mesh du frame
    print("Creating frame mesh...")
    obj_path = "goat_lens_simulation.obj"
    frame_mesh = create_lens_frame_mesh(
        R=R_FRAME,
        r_rod=0.005,  # 5mm rayon
        n_pts=60,     # Points par anneau (réduit pour perfs)
        n_sections=8, # Sections cylindre
        n_struts=6,   # Struts de liaison
        export_obj=obj_path
    )
    
    # Créer la scène Genesis
    print("Creating Genesis scene...")
    dt_fem = 1e-4  # Timestep FEM
    
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            substeps=20,
            gravity=(0.0, 0.0, -9.81),
        ),
        fem_options=gs.options.FEMOptions(
            dt=dt_fem,
            gravity=(0.0, 0.0, -9.81),
            use_implicit_solver=True,
            n_newton_iterations=10,
            n_pcg_iterations=50,
            damping_alpha=0.02,
            damping_beta=0.002,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(1.5, 1.5, 0.8),
            camera_lookat=(0.0, 0.0, 0.4),
            camera_fov=45.0,
        ),
        vis_options=gs.options.VisOptions(
            show_world_frame=True,
        ),
        show_viewer=True,
    )
    
    # Sol rigide
    scene.add_entity(
        morph=gs.morphs.Plane(),
        material=gs.materials.Rigid(),
    )
    
    # Paramètres matériaux - Fibre de verre
    # Réduire E légèrement pour stabilité numérique tout en restant réaliste
    E = 5e6        # Young's modulus [Pa] (réduit pour stabilité FEM)
    nu = 0.30      # Poisson's ratio
    rho = 400.0    # Density [kg/m³] (réduit pour démo)
    
    # Frame GOAT lens - deformable beam (FEM Elastic)
    print("Adding frame to scene...")
    frame = scene.add_entity(
        morph=gs.morphs.Mesh(
            file=obj_path,
            pos=(0.0, 0.0, 0.5),
            euler=(0.0, 0.0, 0.0),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=gs.materials.FEM.Elastic(
            E=E,
            nu=nu,
            rho=rho,
            model="linear_corotated",
        ),
        surface=gs.surfaces.Default(
            color=(0.25, 0.50, 1.0),
            vis_mode="visual",  # Skinning: visualiser le mesh déformé
        ),
    )
    
    # Obstacles de test (scénario gap)
    # Deux murs parallèles créant un couloir
    wall_width = 0.04
    wall_height = 0.8
    corridor_width = 0.45  # Plus étroit que le frame (L2_ROVER = 0.5787)
    
    wall_left = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(-corridor_width/2 - wall_width/2, 0.0, wall_height/2),
            size=(wall_width, 1.0, wall_height),
        ),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.8, 0.8, 0.8)),
    )
    
    wall_right = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(corridor_width/2 + wall_width/2, 0.0, wall_height/2),
            size=(wall_width, 1.0, wall_height),
        ),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.8, 0.8, 0.8)),
    )
    
    # Build scene
    print("Building scene...")
    scene.build()
    scene.reset()
    
    # Simulation
    print("Starting simulation — press Ctrl+C to stop")
    print(f"Frame Young's Modulus: {E:.2e} Pa")
    print(f"Frame Density: {rho} kg/m³")
    print(f"Corridor width: {corridor_width:.3f} m (frame Y-width: 0.579 m)")
    print()
    
    try:
        for i in range(10000):
            # Afficher état tous les 1000 steps
            if i % 1000 == 0:
                print(f"Step {i:5d} / 10000")
            
            scene.step()
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")


if __name__ == "__main__":
    main()
