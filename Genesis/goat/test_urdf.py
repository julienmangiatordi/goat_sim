#!/usr/bin/env python3
"""Quick URDF geometry test - just load and inspect"""

import genesis as gs
import numpy as np

gs.init(backend=gs.cpu, precision="64", logging_level="warning")

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.002, gravity=(0.0, 0.0, -9.81)),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(1.5, -1.5, 1.0),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=50,
        max_FPS=60,
    ),
    show_viewer=True,
)

scene.add_entity(gs.morphs.Plane())

goat = scene.add_entity(
    gs.morphs.URDF(
        file="goat_v4_cable_driven.urdf",
        pos=(0.0, 0.0, 0.35),
        euler=(0.0, 0.0, 0.0),
        fixed=False,
    )
)

scene.build()

print("\n=== GOAT Structure Analysis ===\n")

print("Available Links:")
for i, link in enumerate(goat.links):
    pos = link.get_pos() if hasattr(link, 'get_pos') else "N/A"
    name = link.name
    print(f"  [{i}] {name:20s} pos={pos}")

print("\nAvailable Joints:")
for i, joint in enumerate(goat.joints):
    name = joint.name
    print(f"  [{i}] {name:20s}")

print("\nDOFs:")
print(f"  Total DOFs: {goat.n_dofs}")

# Try to find wheel DOFs
try:
    wheel_dofs = []
    for name in ["wheel_FL_joint", "wheel_FR_joint", "wheel_RL_joint", "wheel_RR_joint"]:
        try:
            dof = goat.get_joint(name).dof_idx_local
            wheel_dofs.append(dof)
            print(f"  {name}: DOF index = {dof}")
        except Exception as e:
            print(f"  {name}: NOT FOUND")
except Exception as e:
    print(f"  ERROR: {e}")

print("\n=== Geometric Check ===\n")

# Get payload position
payload = goat.get_link("payload")
if payload:
    payload_pos = payload.get_pos()
    print(f"Payload position: {payload_pos}")

# Try to get corner positions
corners = ["lens_FL", "lens_FR", "lens_RR", "lens_RL"]
print("\nCorner positions (should be at 0.30/±0.35 or ±0.30/±0.35):")
for corner in corners:
    try:
        link = goat.get_link(corner)
        if link:
            pos = link.get_pos()
            print(f"  {corner}: {pos}")
    except:
        print(f"  {corner}: NOT FOUND (merged into parent)")

print("\nEyelet positions:")
for eyelet in ["eyelet_F", "eyelet_R", "eyelet_B", "eyelet_L"]:
    try:
        link = goat.get_link(eyelet)
        if link:
            pos = link.get_pos()
            print(f"  {eyelet}: {pos}")
    except:
        print(f"  {eyelet}: NOT FOUND (merged)")

print("\n=== Simulation Test (run 100 steps) ===\n")
for i in range(100):
    scene.step()
    if i % 25 == 0:
        payload_pos = goat.get_link("payload").get_pos()
        print(f"Step {i}: payload z={payload_pos[2]:.4f}")

print("\nTest complete. Inspect the viewer window for visuals.")
