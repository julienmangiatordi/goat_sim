#!/usr/bin/env python3
"""
Test script: Y-wheels-only propulsion in open space (no walls)
"""
import sys
import os
import numpy as np
import genesis as gs

# Constants from main file
DT = 0.002
R_FRAME = 4.0 / (2 * np.pi)
ASPECT_RATIO = 2.2
L1_ROVER = 2.0 * R_FRAME
L2_ROVER = L1_ROVER / ASPECT_RATIO
WHEEL_R = 0.25
TRACK_WIDTH = L2_ROVER
Z_INIT = WHEEL_R
TENDON_K = 2000.0
TENDON_DAMP = 120.0
PIVOT_KV = 0.7
WHEEL_KP = 1500.0
WHEEL_KV = 50.0
WHEEL_TORQUE_MAX = 50.0

# Load main module to get GOATRover class
sys.path.insert(0, os.path.dirname(__file__))
from goat_rover_sim import GOATRover

def main():
    print("=" * 60)
    print("TEST: Y-wheels-only locomotion (no walls)")
    print("=" * 60)
    
    gs.init(backend=gs.cpu, precision="64", logging_level="warning")
    
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0.0, 0.0, -9.81)),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.2, -2.5, 1.6),
            camera_lookat=(0.0, 0.0, 0.25),
            camera_fov=55,
            max_FPS=60,
        ),
        show_viewer=False,  # Disabled for faster testing
    )
    
    # Add ground only
    scene.add_entity(
        gs.morphs.Plane(),
        surface=gs.surfaces.Rough(),
    )
    
    # Add robot
    robot_entity = scene.add_entity(
        gs.morphs.URDF(
            file="goat_rover_2.urdf",
            pos=(0.0, 0.0, Z_INIT),
            euler=(0.0, 0.0, 0.0),
            fixed=False,
        )
    )
    
    scene.build()
    
    goat = GOATRover(robot_entity)
    goat.configure_gains()
    goat.set_solver(scene)
    
    print(f"[Genesis] {robot_entity.n_links} links loaded")
    print(f"[TEST] Moving with all wheels (normal skid_steer)")
    print(f"[TEST] Expected: linear motion in +Y direction\n")
    
    # Settle
    print("[SETTLE] 2 seconds...")
    for i in range(int(2.0 / DT)):
        goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
        scene.step()
    
    # Moving phase
    print("[MOVE] 10 seconds at v_lin=0.30 m/s...")
    duration = 10.0
    t_log = 1.0
    steps = int(duration / DT)
    
    for i in range(steps):
        t = i * DT
        pos = np.array(goat.entity.get_link("base_link").get_pos().tolist())
        vel = np.array(goat.entity.get_link("base_link").get_vel().tolist())
        
        # Use regular skid_steer for comparison
        goat.skid_steer(v_lin=0.30, v_ang=0.0)
        goat.apply_tendon_forces(L1_ROVER, L2_ROVER)
        scene.step()
        
        if t >= t_log:
            speed = np.linalg.norm(vel[:2])
            print(f"  t={t:.1f}s | pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m | "
                  f"v={speed:.3f} m/s | v_y={vel[1]:.3f} m/s")
            t_log += 1.0
    
    # Final report
    pos_final = np.array(goat.entity.get_link("base_link").get_pos().tolist())
    dist_y = pos_final[1]
    expected_y = 0.30 * 10.0  # 3.0 m expected
    efficiency = (dist_y / expected_y * 100) if expected_y > 0 else 0
    
    print(f"\n[RESULT]")
    print(f"  Distance traveled in Y: {dist_y:.3f} m")
    print(f"  Expected (v=0.30 for 10s): {expected_y:.3f} m")
    print(f"  Efficiency: {efficiency:.1f}%")

if __name__ == "__main__":
    main()
