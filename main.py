import sys
import io
import numpy as np

if sys.stdout.encoding != "utf-8":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

from robot_arm import ThreeDOFManipulator

def run_demonstration():
    arm = ThreeDOFManipulator(l1=0.50, l2=0.40, l3=0.30, name="RoboticArm_3DOF")

    print(arm.robot)

    test_cases = [
        ("Home", np.array([0.0, 0.0, 0.0])),
        ("Reaching", np.deg2rad([30.0, 45.0, 30.0])),
        ("Pick", np.deg2rad([-45.0, 60.0, -30.0]))
    ]

    for label, q in test_cases:
        q_deg = np.rad2deg(q)
        print(f"Scenario: {label}")
        print(f"Joint Angles (deg): q1={q_deg[0]:.2f}, q2={q_deg[1]:.2f}, q3={q_deg[2]:.2f}")

        T_toolbox = arm.forward_kinematics_toolbox(q)
        pos_toolbox = T_toolbox.t
        rpy_toolbox = T_toolbox.rpy(unit="deg")

        T_analytical, pos_analytical = arm.forward_kinematics_analytical(q)
        is_matched = arm.verify_kinematics(q)

        print("Transformation Matrix:")
        for row in T_toolbox.A:
            print(f"  [ {row[0]:8.4f}  {row[1]:8.4f}  {row[2]:8.4f}  {row[3]:8.4f} ]")

        print(f"Position (Toolbox)   : X={pos_toolbox[0]:.4f}, Y={pos_toolbox[1]:.4f}, Z={pos_toolbox[2]:.4f}")
        print(f"Position (Analytical): X={pos_analytical[0]:.4f}, Y={pos_analytical[1]:.4f}, Z={pos_analytical[2]:.4f}")
        print(f"Match Status: {is_matched}")
        print(f"RPY: Roll={rpy_toolbox[0]:.2f}, Pitch={rpy_toolbox[1]:.2f}, Yaw={rpy_toolbox[2]:.2f}\n")

    render_q = np.deg2rad([30.0, 45.0, 30.0])
    arm.plot_3d(render_q, block=False, save_path="manipulator_3d_pose.png")

if __name__ == "__main__":
    run_demonstration()
