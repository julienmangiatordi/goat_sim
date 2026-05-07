import numpy as np
import genesis as gs

E   = 5e4
nu  = 0.40
rho = 800.0

gs.init(seed=0, precision='32', logging_level='debug')

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=4e-3,          # ← valeur de la doc officielle, PAS 5e-4
        substeps=10,
        gravity=(0, 0, -9.81),  # ← gravité ICI seulement
    ),
    mpm_options=gs.options.MPMOptions(
        # ← PAS de dt ici, PAS de gravity ici (non supportés selon la doc)
        lower_bound=(-1.0, -1.0, -0.5),
        upper_bound=( 1.0,  1.0,  1.5),
    ),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.8, 0.0, 0.9),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=40,
    ),
    vis_options=gs.options.VisOptions(
        show_world_frame=True,
        visualize_mpm_boundary=True,   # ← utile pour debug
    ),
    show_viewer=True,
)

scene.add_entity(morph=gs.morphs.Plane())

# ── TEST CUBE (doit tomber) ──────────────────────────────────────────────────
cube = scene.add_entity(
    material=gs.materials.MPM.Elastic(E=5e4, nu=0.4, rho=800.0),
    morph=gs.morphs.Box(
        pos=(0.0, 0.0, 0.5),
        size=(0.15, 0.15, 0.15),
    ),
    surface=gs.surfaces.Default(
        color=(1.0, 0.4, 0.4, 1.0),
        vis_mode='particle',   # ← 'particle' indispensable pour voir quelque chose
    ),
)

# ── FRAME GOAT ───────────────────────────────────────────────────────────────
frame_mpm = scene.add_entity(
    morph=gs.morphs.Mesh(
        file='goat_lens.obj',
        pos=(0.0, 0.5, 0.9),    # décalé en Y pour ne pas superposer au cube
        euler=(90.0, 0.0, 0.0),
        scale=1.0,
        decimate=False,
        convexify=False,
    ),
    material=gs.materials.MPM.Elastic(   # ← Elastic d'abord, plus simple que Muscle
        E=E,
        nu=nu,
        rho=rho,
    ),
    surface=gs.surfaces.Default(
        color=(0.15, 0.35, 0.95, 1.0),
        vis_mode='particle',    # ← 'particle' pour voir si des particules existent
    ),
)

scene.build()
scene.reset()

# ── Diagnostic particules ─────────────────────────────────────────────────────
state  = frame_mpm.get_state()
pos_np = state.pos.cpu().numpy()
if pos_np.ndim == 3:
    pos_np = pos_np[0]
print(f"[DIAG] Particules dans le frame : {pos_np.shape[0]}")
# Si 0 → le mesh goat_lens.obj est trop fin → revenir vers la Solution A (Box primitives)

state2  = cube.get_state()
pos2_np = state2.pos.cpu().numpy()
if pos2_np.ndim == 3:
    pos2_np = pos2_np[0]
print(f"[DIAG] Particules dans le cube  : {pos2_np.shape[0]}")

for i in range(2000):
    scene.step()