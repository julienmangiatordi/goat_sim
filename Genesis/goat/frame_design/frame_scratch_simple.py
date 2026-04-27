"""
Frame GOAT Lens - Genesis Simple Working Version

USAGE:
    python frame_scratch_simple.py
"""

import genesis as gs
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_mesh_path(filename):
    return os.path.join(SCRIPT_DIR, filename)


def main():
    print("=" * 70)
    print("GOAT Lens Frame - Genesis Deformable Beams Simulation")
    print("=" * 70)

    gs.init(seed=0, precision="32", logging_level="info", backend=gs.cpu)

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=4e-3,
            substeps=10,
            gravity=(0.0, 0.0, -9.81),
        ),
        fem_options=gs.options.FEMOptions(
            damping=0.1,
        ),
        rigid_options=gs.options.RigidOptions(
            gravity=(0.0, 0.0, -9.81),
            enable_collision=True,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.0, 2.0, 1.5),
            camera_lookat=(0.0, 0.0, 0.6),
            camera_fov=60.0,
        ),
        vis_options=gs.options.VisOptions(show_world_frame=True),
        show_viewer=True,
    )

    # Sol rigide
    scene.add_entity(
        morph=gs.morphs.Plane(),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.8, 0.8, 0.8, 1.0)),
    )

    # Matériau fibre de verre
    mat = gs.materials.FEM.Elastic(
        E=5e6,
        nu=0.25,
        rho=1800.0,
        model="stable_neohookean",
    )

    strut_mat = gs.materials.FEM.Elastic(
        E=5e6,
        nu=0.30,
        rho=1800.0,
        model="stable_neohookean",
    )

    # Ring A (axe X-Z)
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=get_mesh_path("ring_A.obj"),
            pos=(0.0, 0.0, 0.6),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=mat,
        surface=gs.surfaces.Default(
            color=(0.25, 0.50, 1.0, 1.0),
            vis_mode="particle",
        ),
    )

    # Ring B (axe Y-Z)
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=get_mesh_path("ring_B.obj"),
            pos=(0.0, 0.0, 0.6),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=mat,
        surface=gs.surfaces.Default(
            color=(0.25, 0.50, 1.0, 1.0),
            vis_mode="particle",
        ),
    )

    # Strut top
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=get_mesh_path("strut_top.obj"),
            pos=(0.0, 0.0, 0.6),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=strut_mat,
        surface=gs.surfaces.Default(
            color=(0.6, 0.6, 0.6, 1.0),
            vis_mode="particle",
        ),
    )

    # Strut bottom
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=get_mesh_path("strut_bot.obj"),
            pos=(0.0, 0.0, 0.6),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=strut_mat,
        surface=gs.surfaces.Default(
            color=(0.6, 0.6, 0.6, 1.0),
            vis_mode="particle",
        ),
    )

    print("Building scene...")
    scene.build()
    scene.reset()

    print()
    print("=" * 70)
    print("SIMULATION RUNNING - FREE FALL TEST")
    print("=" * 70)
    print("vis_mode=particle : les noeuds FEM s'affichent comme des points")
    print("Gravity: 9.81 m/s² | Model: stable_neohookean | Backend: CPU")
    print("Press Ctrl+C to stop")
    print()

    try:
        for i in range(10000):
            if i % 500 == 0 and i > 0:
                print(f"Step {i:5d} / 10000")
            scene.step()

    except KeyboardInterrupt:
        print("\n✓ Simulation stopped by user")

    print("Done!")


if __name__ == "__main__":
    main()