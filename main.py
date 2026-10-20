"""
3-DOF Manipulator Forward Kinematics & 3D Visualization Runner
=============================================================
Demonstrates:
1. Standard DH modeling of 3-DOF RRR arm
2. Forward kinematics calculation (Toolbox vs Analytical)
3. 3D visualization and image export
"""

import sys
import io
import numpy as np

# Ensure UTF-8 output encoding for terminals supporting unicode symbols
if sys.stdout.encoding != "utf-8":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

from robot_arm import ThreeDOFManipulator


def run_demonstration():
    print("=" * 70)
    print("     3-DOF REVOLUTE ROBOTIC MANIPULATOR (RRR) KINEMATICS")
    print("=" * 70)

    # 1. Instantiate the manipulator with realistic dimensions (in meters)
    # L1 = 0.50 m (Base height)
    # L2 = 0.40 m (Upper arm length)
    # L3 = 0.30 m (Forearm length)
    arm = ThreeDOFManipulator(l1=0.50, l2=0.40, l3=0.30, name="RoboticArm_3DOF")

    print("\n[1] ROBOT MODEL SPECIFICATION:")
    print(f"    - Link 1 (Base height, L1) : {arm.l1:.2f} m")
    print(f"    - Link 2 (Upper arm,   L2) : {arm.l2:.2f} m")
    print(f"    - Link 3 (Forearm,     L3) : {arm.l3:.2f} m")
    print("\n[2] DENAVIT-HARTENBERG (STANDARD DH) PARAMETERS:")
    print(arm.robot)

    # 2. Define sample test poses (in degrees and radians)
    test_cases = [
        ("Home Position (Zero)", np.array([0.0, 0.0, 0.0])),
        ("Reaching Pose 1", np.deg2rad([30.0, 45.0, 30.0])),
        ("Pick & Place Pose 2", np.deg2rad([-45.0, 60.0, -30.0]))
    ]

    print("\n[3] FORWARD KINEMATICS EVALUATION:")
    for label, q in test_cases:
        print("-" * 70)
        q_deg = np.rad2deg(q)
        print(f"▶ Scenario: {label}")
        print(f"  Joint Angles (deg) : q1 = {q_deg[0]:.2f}°, q2 = {q_deg[1]:.2f}°, q3 = {q_deg[2]:.2f}°")
        print(f"  Joint Angles (rad) : q1 = {q[0]:.4f}, q2 = {q[1]:.4f}, q3 = {q[2]:.4f}")

        # Compute using roboticstoolbox
        T_toolbox = arm.forward_kinematics_toolbox(q)
        pos_toolbox = T_toolbox.t
        rpy_toolbox = T_toolbox.rpy(unit="deg")

        # Compute using analytical equations
        T_analytical, pos_analytical = arm.forward_kinematics_analytical(q)

        # Verification check
        is_matched = arm.verify_kinematics(q)

        print("\n  [A] End-Effector Homogeneous Transformation Matrix T (Base to Tool):")
        # Format matrix nicely
        for row in T_toolbox.A:
            print(f"      [ {row[0]:8.4f}  {row[1]:8.4f}  {row[2]:8.4f}  {row[3]:8.4f} ]")

        print(f"\n  [B] End-Effector Position (X, Y, Z) in meters:")
        print(f"      Toolbox    : X = {pos_toolbox[0]:.4f} m, Y = {pos_toolbox[1]:.4f} m, Z = {pos_toolbox[2]:.4f} m")
        print(f"      Analytical : X = {pos_analytical[0]:.4f} m, Y = {pos_analytical[1]:.4f} m, Z = {pos_analytical[2]:.4f} m")
        print(f"      Verification Match : {'PASSED ✓' if is_matched else 'FAILED ✗'}")

        print(f"\n  [C] End-Effector Orientation (Roll, Pitch, Yaw):")
        print(f"      Roll = {rpy_toolbox[0]:.2f}°, Pitch = {rpy_toolbox[1]:.2f}°, Yaw = {rpy_toolbox[2]:.2f}°")

    # 3. Render and save 3D visualization for the Reaching Pose
    render_q = np.deg2rad([30.0, 45.0, 30.0])
    output_image = "manipulator_3d_pose.png"
    print("\n" + "=" * 70)
    print(f"[4] GENERATING 3D SKETCH/PLOT for q = [30°, 45°, 30°]...")
    arm.plot_3d(render_q, block=False, save_path=output_image)
    print("=" * 70)
    print("\nDemonstration completed successfully!")


if __name__ == "__main__":
    run_demonstration()
