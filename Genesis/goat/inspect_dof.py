"""
Inspect DOF mapping to understand wheel velocity command order
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import genesis as gs
import numpy as np

gs.init(backend=gs.cpu, precision="64", logging_level="warning")

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.002, gravity=(0.0, 0.0, -9.81)),
    show_viewer=False,
)

scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Rough())

robot_entity = scene.add_entity(
    gs.morphs.URDF(
        file="goat_rover_2.urdf",
        pos=(0.0, 0.0, 0.25),
        euler=(0.0, 0.0, 0.0),
        fixed=False,
    )
)

scene.build()

print("=" * 60)
print("DOF INSPECTION")
print("=" * 60)

print("\n[JOINTS]")
for i, j in enumerate(robot_entity.joints):
    if "wheel" in j.name:
        print(f"  {i}: {j.name:30s} → dof_idx_local={j.dof_idx_local}")

print("\n[WHEEL DOF MAPPING]")
R = robot_entity.get_joint("wheel_joint_X_pos").dof_idx_local
L = robot_entity.get_joint("wheel_joint_X_neg").dof_idx_local
F = robot_entity.get_joint("wheel_joint_Y_pos").dof_idx_local
B = robot_entity.get_joint("wheel_joint_Y_neg").dof_idx_local

dofs_wheels = np.array([R, L, F, B], dtype=int)

print(f"  dof_R (X_pos):  {R}")
print(f"  dof_L (X_neg):  {L}")
print(f"  dof_F (Y_pos):  {F}")
print(f"  dof_B (Y_neg):  {B}")
print(f"  dofs_wheels array: {dofs_wheels}")

print("\n[TEST: Apply velocities to each DOF individually]")
for test_idx in range(4):
    print(f"\n-- Test {test_idx}: Set only DOF {test_idx} to 1.0 rad/s --")
    v = np.zeros(4)
    v[test_idx] = 1.0
    robot_entity.control_dofs_velocity(v, dofs_idx_local= dofs_wheels)
    scene.step()
    
    # Check which wheel is actually spinning
    base = np.array(robot_entity.get_link("base_link").get_pos().tolist())
    
    # Check velocities of each wheel
    for wheel_idx, (name, dof_local) in enumerate([
        ("X_pos", R), ("X_neg", L), ("Y_pos", F), ("Y_neg", B)
    ]):
        try:
            v_dof = robot_entity.get_dofs_velocity(dofs_idx_local=np.array([dof_local]))
            print(f"    {name:8s} (dof={dof_local}): v={v_dof[0]:.3f} rad/s")
        except:
            print(f"    {name:8s} (dof={dof_local}): error reading")

print("\n[Done]")
