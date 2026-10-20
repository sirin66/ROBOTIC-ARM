"""
3-DOF Articulated Robotic Manipulator (RRR)
===========================================
Defines a 3 Degrees of Freedom (RRR) spatial manipulator using:
- Standard Denavit-Hartenberg (DH) parameters
- Peter Corke's roboticstoolbox-python
- Analytical Forward Kinematics verification
- Matplotlib 3D plotting
"""

from typing import Tuple, List, Optional
import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3


class ThreeDOFManipulator:
    """
    3-DOF Spatial Articulated Robotic Arm (RRR).
    
    Joint Configuration:
    - Joint 1 (Base / Waist): Revolute around Z0 (Azimuth)
    - Joint 2 (Shoulder): Revolute around Z1 (Elevation/Pitch)
    - Joint 3 (Elbow): Revolute around Z2 (Flexion/Extension)
    """

    def __init__(
        self,
        l1: float = 0.5,
        l2: float = 0.4,
        l3: float = 0.3,
        name: str = "3DOF_Articulated_Arm"
    ):
        """
        Initialize the 3-DOF manipulator with link lengths.
        
        :param l1: Length of base link (height along Z0) in meters
        :param l2: Length of upper arm in meters
        :param l3: Length of forearm in meters
        :param name: Identifier name for the robot model
        """
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.l3 = float(l3)
        self.name = name

        # Define DH Links using Standard DH convention:
        # Link 1: d = L1, a = 0, alpha = +pi/2
        # Link 2: d = 0,  a = L2, alpha = 0
        # Link 3: d = 0,  a = L3, alpha = 0
        self.link1 = rtb.RevoluteDH(
            d=self.l1,
            a=0.0,
            alpha=np.pi / 2,
            qlim=[-np.pi, np.pi]
        )
        self.link2 = rtb.RevoluteDH(
            d=0.0,
            a=self.l2,
            alpha=0.0,
            qlim=[-np.pi / 2, np.pi / 2]
        )
        self.link3 = rtb.RevoluteDH(
            d=0.0,
            a=self.l3,
            alpha=0.0,
            qlim=[-np.pi * 3 / 4, np.pi * 3 / 4]
        )

        # Assemble the DHRobot serial link manipulator
        self.robot = rtb.DHRobot(
            [self.link1, self.link2, self.link3],
            name=self.name
        )

    def forward_kinematics_toolbox(self, q: np.ndarray | List[float]) -> SE3:
        """
        Compute Forward Kinematics using roboticstoolbox-python.
        
        :param q: Array or list of 3 joint angles [q1, q2, q3] in radians
        :return: SE3 pose object containing 4x4 homogeneous transformation matrix
        """
        q_rad = np.asarray(q, dtype=float)
        T: SE3 = self.robot.fkine(q_rad)
        return T

    def forward_kinematics_analytical(
        self, q: np.ndarray | List[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute Forward Kinematics analytically from first principles:
        T_0_3 = A1 * A2 * A3
        
        :param q: Array or list of 3 joint angles [q1, q2, q3] in radians
        :return: (T, position) where T is (4, 4) homogeneous transformation matrix
                 and position is [px, py, pz]
        """
        q1, q2, q3 = q[0], q[1], q[2]
        
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c3, s3 = np.cos(q3), np.sin(q3)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)

        # Individual Link Transformation Matrices (Standard DH)
        A1 = np.array([
            [c1, 0,  s1, 0],
            [s1, 0, -c1, 0],
            [ 0, 1,   0, self.l1],
            [ 0, 0,   0, 1]
        ])

        A2 = np.array([
            [c2, -s2, 0, self.l2 * c2],
            [s2,  c2, 0, self.l2 * s2],
            [ 0,   0, 1, 0],
            [ 0,   0, 0, 1]
        ])

        A3 = np.array([
            [c3, -s3, 0, self.l3 * c3],
            [s3,  c3, 0, self.l3 * s3],
            [ 0,   0, 1, 0],
            [ 0,   0, 0, 1]
        ])

        # Overall Transformation Matrix
        T = A1 @ A2 @ A3

        # Analytical end-effector position vector:
        # px = cos(q1) * (L2 * cos(q2) + L3 * cos(q2 + q3))
        # py = sin(q1) * (L2 * cos(q2) + L3 * cos(q2 + q3))
        # pz = L1 + L2 * sin(q2) + L3 * sin(q2 + q3)
        px = c1 * (self.l2 * c2 + self.l3 * c23)
        py = s1 * (self.l2 * c2 + self.l3 * c23)
        pz = self.l1 + self.l2 * s2 + self.l3 * s23
        position = np.array([px, py, pz])

        return T, position

    def verify_kinematics(self, q: np.ndarray | List[float], atol: float = 1e-6) -> bool:
        """
        Verify that analytical kinematics match toolbox kinematics within tolerance.
        """
        T_toolbox = self.forward_kinematics_toolbox(q).A
        T_analytical, _ = self.forward_kinematics_analytical(q)
        return bool(np.allclose(T_toolbox, T_analytical, atol=atol))

    def plot_3d(
        self,
        q: np.ndarray | List[float],
        block: bool = False,
        save_path: Optional[str] = None
    ):
        """
        Render a 3D visualization of the manipulator at joint configuration q.
        
        :param q: Joint angles [q1, q2, q3] in radians
        :param block: Whether matplotlib plot blocks execution
        :param save_path: File path to save rendered image (optional)
        """
        q_rad = np.asarray(q, dtype=float)

        # Plot using Peter Corke's PyPlot backend
        env = self.robot.plot(
            q_rad,
            backend="pyplot",
            block=False,
            shadow=True,
            jointaxes=True
        )

        fig = plt.gcf()
        ax = plt.gca()
        ax.set_title(
            f"{self.name}\n"
            f"q = [{np.rad2deg(q_rad[0]):.1f}°, {np.rad2deg(q_rad[1]):.1f}°, {np.rad2deg(q_rad[2]):.1f}°]",
            fontsize=12,
            fontweight="bold"
        )

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"[✓] Saved 3D robot rendering to: {save_path}")

        if block:
            plt.show()

        return env
