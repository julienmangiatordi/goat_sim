import numpy as np
import genesis as gs

gs.init(seed=0, precision="32", logging_level="info")

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        substeps=20,
        gravity=(0.0, 0.0, -9.81),
    ),
    fem_options=gs.options.FEMOptions(
        dt=1e-4,
        gravity=(0.0, 0.0, -9.81),   # <--- important
        use_implicit_solver=True,
        n_newton_iterations=10,
        n_pcg_iterations=50,
        damping_alpha=0.02,
        damping_beta=0.002,
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.5, 1.0, 1.0),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=45.0,
    ),
    vis_options=gs.options.VisOptions(show_world_frame=True),
    show_viewer=True,
)

# Sol rigide
scene.add_entity(
    morph=gs.morphs.Plane(),
    material=gs.materials.Rigid(),
)

# Matériau FEM
mat = gs.materials.FEM.Elastic(
    E=5e6,
    nu=0.30,
    rho=400.0,
    model="linear_corotated",
)

spawn_z = 0.6
blue = (0.15, 0.35, 0.95, 1.0)
dark = (0.15, 0.15, 0.15, 1.0)

for fname, color in [
    ("ring_A.obj", blue),
    ("ring_B.obj", blue),
    ("strut_top.obj", dark),
    ("strut_bot.obj", dark),
]:
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=fname,
            pos=(0.0, 0.0, spawn_z),
            euler=(0.0, 0.0, 0.0),
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=mat,
        surface=gs.surfaces.Default(color=color),
    )

scene.build()
scene.reset()

print("Running — Ctrl+C to stop")
for i in range(8000):
    scene.step()
