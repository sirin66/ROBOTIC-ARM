from typing import Tuple, List, Optional
import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3

class ThreeDOFManipulator:
    def __init__(self, l1: float = 0.5, l2: float = 0.4, l3: float = 0.3, name: str = "3DOF_Manipulator"):
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.l3 = float(l3)
        self.name = name

        self.link1 = rtb.RevoluteDH(d=self.l1, a=0.0, alpha=np.pi / 2, qlim=[-np.pi, np.pi])
        self.link2 = rtb.RevoluteDH(d=0.0, a=self.l2, alpha=0.0, qlim=[-np.pi / 2, np.pi / 2])
        self.link3 = rtb.RevoluteDH(d=0.0, a=self.l3, alpha=0.0, qlim=[-np.pi * 3 / 4, np.pi * 3 / 4])

        self.robot = rtb.DHRobot([self.link1, self.link2, self.link3], name=self.name)

    def forward_kinematics_toolbox(self, q: np.ndarray | List[float]) -> SE3:
        q_rad = np.asarray(q, dtype=float)
        return self.robot.fkine(q_rad)

    def forward_kinematics_analytical(self, q: np.ndarray | List[float]) -> Tuple[np.ndarray, np.ndarray]:
        q1, q2, q3 = q[0], q[1], q[2]
        
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c3, s3 = np.cos(q3), np.sin(q3)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)

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

        T = A1 @ A2 @ A3

        px = c1 * (self.l2 * c2 + self.l3 * c23)
        py = s1 * (self.l2 * c2 + self.l3 * c23)
        pz = self.l1 + self.l2 * s2 + self.l3 * s23
        position = np.array([px, py, pz])

        return T, position

    def verify_kinematics(self, q: np.ndarray | List[float], atol: float = 1e-6) -> bool:
        T_toolbox = self.forward_kinematics_toolbox(q).A
        T_analytical, _ = self.forward_kinematics_analytical(q)
        return bool(np.allclose(T_toolbox, T_analytical, atol=atol))

    def plot_3d(self, q: np.ndarray | List[float], block: bool = False, save_path: Optional[str] = None):
        q_rad = np.asarray(q, dtype=float)

        env = self.robot.plot(q_rad, backend="pyplot", block=False, shadow=True, jointaxes=True)

        fig = plt.gcf()
        ax = plt.gca()
        ax.set_title(
            f"{self.name} | q = [{np.rad2deg(q_rad[0]):.1f}°, {np.rad2deg(q_rad[1]):.1f}°, {np.rad2deg(q_rad[2]):.1f}°]",
            fontsize=12,
            fontweight="bold"
        )

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")

        if block:
            plt.show()

        return env
