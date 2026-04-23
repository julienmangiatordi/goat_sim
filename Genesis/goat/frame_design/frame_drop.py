import numpy as np
import genesis as gs

dt  = 2e-4   # timestep plus petit pour stabilité à E élevé

gs.init(seed=0, precision="32", logging_level="info")

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        substeps=20,
        gravity=(0.0, 0.0, -9.81),
    ),
    mpm_options=gs.options.MPMOptions(
        dt=dt,
        lower_bound=(-1.0, -1.0, -0.1),
        upper_bound=( 1.0,  1.0,  1.5),
        grid_density=128,          # résolution suffisante pour des tubes fins
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.5, 1.5, 0.8),
        camera_lookat=(0.0, 0.0, 0.3),
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

# Frame GOAT lens
# Fiberglass: E ~ 40 GPa réalité, réduit à 5e8 pour stabilité numérique MPM
E   = 5e8
nu  = 0.28
rho = 2000.0

frame = scene.add_entity(
    morph=gs.morphs.Mesh(
        file="goat_lens.obj",
        pos=(0.0, 0.0, 0.5),          # Position spawn
        euler=(0.0, 0.0, 0.0),        # Rings debout (X-Z et Y-Z verticaux)
        scale=1.0,
        decimate=False,
        convexify=False,
    ),
    material=gs.materials.MPM.Elastic(
        E=E,
        nu=nu,
        rho=rho,
        sampler="random",
    ),
    # Visualisation: spheres propres (pas d'artefacts de déformation du mesh)
    surface=gs.surfaces.Default(
        color=(0.25, 0.50, 1.0),
        vis_mode="particle",
    ),
)

scene.build()
scene.reset()

print("Simulation running — press Ctrl+C to stop")
for i in range(6000):
    scene.step()