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
    parser.add_argument("--show", action="store_true", help="Display interactive window")
    parser.add_argument("--output", type=str, default="cad_robot_render.png", help="Path to save image")
    parser.add_argument("--q", nargs=3, type=float, default=[35.0, 45.0, 30.0], help="Joint angles q1 q2 q3")
    args = parser.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    arm = ThreeDOFManipulator(l1=0.50, l2=0.40, l3=0.30, name="3-DOF Articulated Robot")
    q_target = np.deg2rad(args.q)

    render_cad_manipulator(
        arm=arm,
        q=q_target,
        title="3-DOF Spatial Robotic Manipulator",
        save_path=args.output,
        show=args.show
    )

if __name__ == "__main__":
    main()
