"""
Execute and Render the Detailed 3D CAD-like Model of the 3-DOF Arm
==================================================================
Usage:
    python run_cad.py               # Generates and saves cad_robot_render.png
    python run_cad.py --show        # Opens interactive 3D Matplotlib CAD window
"""

import sys
import io
import argparse
import numpy as np
import matplotlib

if sys.stdout.encoding != "utf-8":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

from robot_arm import ThreeDOFManipulator
from cad_model import render_cad_manipulator


def main():
    parser = argparse.ArgumentParser(description="3-DOF Manipulator 3D CAD Visualizer")
    parser.add_argument(
        "--show", action="store_true", help="Display interactive 3D rotatable window"
    )
    parser.add_argument(
        "--output", type=str, default="cad_robot_render.png", help="Path to save rendered image"
    )
    parser.add_argument(
        "--q", nargs=3, type=float, default=[35.0, 45.0, 30.0],
        help="Joint angles q1 q2 q3 in degrees (default: 35 45 30)"
    )
    args = parser.parse_args()

    # If --show is requested, use an interactive GUI backend
    if not args.show:
        matplotlib.use("Agg")

    print("=" * 70)
    print("     DETAILED 3D CAD-LIKE MODEL OF 3-DOF ROBOTIC MANIPULATOR")
    print("=" * 70)

    # 1. Instantiate the manipulator (L1=0.5m, L2=0.4m, L3=0.3m)
    arm = ThreeDOFManipulator(l1=0.50, l2=0.40, l3=0.30, name="3-DOF Articulated Robot")

    q_target = np.deg2rad(args.q)

    print(f"\n[+] Joint Angles:")
    print(f"    Joint 1 (Waist Azimuth)    : {args.q[0]:+6.1f}°")
    print(f"    Joint 2 (Shoulder Pitch)   : {args.q[1]:+6.1f}°")
    print(f"    Joint 3 (Elbow Flexion)    : {args.q[2]:+6.1f}°")

    print(f"\n[+] Assembling CAD Geometry:")
    print(f"    [1] Industrial grid floor with foundation mounting flange & anchor bolts")
    print(f"    [2] Joint 1 (Base): Lower pedestal, chrome slewing ring, and rotating turret")
    print(f"    [3] Joint 2 (Shoulder): Transverse harmonic drive motor with dual end-caps")
    print(f"    [4] Link 2 (Upper Arm): Solid extruded structural beam with carbon center rib")
    print(f"    [5] Joint 3 (Elbow): Transverse pivot actuator and bearing knuckle")
    print(f"    [6] Link 3 (Forearm): Tapered structural beam with mounting plate")
    print(f"    [7] Tool End-Effector: Circular flange & parallel 2-finger mechanical gripper")
    print("    [8] Coordinate Triads: Base {0}, Shoulder {1}, Elbow {2}, Tool {EE} (RGB)")
    print(f"    [9] Engineering Telemetry HUD displaying forward kinematics in real-time")

    render_cad_manipulator(
        arm=arm,
        q=q_target,
        title="3-DOF Spatial Robotic Manipulator (CAD Model)",
        save_path=args.output,
        show=args.show
    )

    print(f"\n[✓] Render completed successfully! Image saved as: {args.output}")
    if args.show:
        print("[i] Interactive 3D window closed.")
    print("=" * 70)


if __name__ == "__main__":
    main()
