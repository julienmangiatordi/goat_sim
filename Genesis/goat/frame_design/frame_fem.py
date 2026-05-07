import numpy as np
import genesis as gs

gs.init(seed=0, precision='32', logging_level='debug')

dt = 1e-3

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=dt,
        substeps=10,
        gravity=(0, 0, -9.81),
    ),
    fem_options=gs.options.FEMOptions(
        dt=dt,
        damping=0.8,               # un peu d'amortissement pour stabilité
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.8, 0.0, 0.9),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=40,
    ),
    vis_options=gs.options.VisOptions(show_world_frame=True),
    show_viewer=True,
)

scene.add_entity(morph=gs.morphs.Plane())

# Matériau FEM calibré sur le GOAT
# E effectif de structure (compliance du treillis de rods fibre de verre)
mat = gs.materials.FEM.Elastic(
    E=2e5,       # Pa — plus rigide qu'en MPM, FEM est plus précis
    nu=0.40,
    rho=800.0,
    model='stable_neohooken',
)

blue = (0.15, 0.35, 0.95, 1.0)
dark = (0.15, 0.15, 0.15, 1.0)

spawn_z = 0.9

# Les 4 fichiers OBJ générés par ton script (tubes fermés, pas creux)
for fname, color in [
    ('ring_A.obj', blue),
    ('ring_B.obj', blue),
    ('strut_top.obj', dark),
    ('strut_bot.obj', dark),
]:
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=fname,
            pos=(0.0, 0.0, spawn_z),
            euler=(90.0, 0.0, 0.0),   # frame vertical
            scale=1.0,
            decimate=False,
            convexify=False,
        ),
        material=mat,
        surface=gs.surfaces.Default(color=color, vis_mode='visual'),
    )

scene.build()
scene.reset()

for i in range(6000):
    scene.step()