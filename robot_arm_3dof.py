import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import roboticstoolbox as rtb
from spatialmath import SE3

class Manipulator3DOF:
    def __init__(self, l1=0.5, l2=0.4, l3=0.3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        self.link1 = rtb.RevoluteDH(d=self.l1, a=0.0, alpha=np.pi/2)
        self.link2 = rtb.RevoluteDH(d=0.0, a=self.l2, alpha=0.0)
        self.link3 = rtb.RevoluteDH(d=0.0, a=self.l3, alpha=0.0)
        
        self.robot = rtb.DHRobot([self.link1, self.link2, self.link3], name="3DOF_Robot")

    def fkine_toolbox(self, q):
        return self.robot.fkine(q)

    def fkine_analytical(self, q):
        q1, q2, q3 = q[0], q[1], q[2]
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c3, s3 = np.cos(q3), np.sin(q3)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)

        A1 = np.array([
            [c1, 0,  s1, 0],
            [s1, 0, -c1, 0],
            [0,  1,  0,  self.l1],
            [0,  0,  0,  1]
        ])

        A2 = np.array([
            [c2, -s2, 0, self.l2 * c2],
            [s2,  c2, 0, self.l2 * s2],
            [0,   0,  1, 0],
            [0,   0,  0, 1]
        ])

        A3 = np.array([
            [c3, -s3, 0, self.l3 * c3],
            [s3,  c3, 0, self.l3 * s3],
            [0,   0,  1, 0],
            [0,   0,  0, 1]
        ])

        T = A1 @ A2 @ A3
        pos = np.array([
            c1 * (self.l2 * c2 + self.l3 * c23),
            s1 * (self.l2 * c2 + self.l3 * c23),
            self.l1 + self.l2 * s2 + self.l3 * s23
        ])
        return T, pos

def get_cylinder(p0, p1, r, res=24):
    v = p1 - p0
    mag = np.linalg.norm(v)
    if mag < 1e-9:
        v = np.array([0, 0, 1.0])
        mag = 1.0
    v = v / mag

    not_v = np.array([1, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1, 0])
    n1 = np.cross(v, not_v)
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(v, n1)

    th = np.linspace(0, 2 * np.pi, res)
    z = np.linspace(0, mag, 2)
    th_grid, z_grid = np.meshgrid(th, z)

    x = p0[0] + r * (np.outer(np.cos(th), np.ones(2)) * n1[0] + np.outer(np.sin(th), np.ones(2)) * n2[0]).T + z_grid * v[0]
    y = p0[1] + r * (np.outer(np.cos(th), np.ones(2)) * n1[1] + np.outer(np.sin(th), np.ones(2)) * n2[1]).T + z_grid * v[1]
    z_mesh = p0[2] + r * (np.outer(np.cos(th), np.ones(2)) * n1[2] + np.outer(np.sin(th), np.ones(2)) * n2[2]).T + z_grid * v[2]
    return x, y, z_mesh

def get_disc(center, normal, r, res=24):
    n = normal / np.linalg.norm(normal)
    not_n = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(n, not_n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)

    r_vals = np.linspace(0, r, 3)
    th_vals = np.linspace(0, 2 * np.pi, res)
    r_g, th_g = np.meshgrid(r_vals, th_vals)

    x = center[0] + r_g * (np.cos(th_g) * u[0] + np.sin(th_g) * v[0])
    y = center[1] + r_g * (np.cos(th_g) * u[1] + np.sin(th_g) * v[1])
    z = center[2] + r_g * (np.cos(th_g) * u[2] + np.sin(th_g) * v[2])
    return x, y, z

def get_box(p0, p1, width, height, lat_vec):
    length_vec = p1 - p0
    L = np.linalg.norm(length_vec)
    if L < 1e-9:
        return []
    u_long = length_vec / L

    u_lat = lat_vec / np.linalg.norm(lat_vec)
    u_trans = np.cross(u_long, u_lat)
    u_trans /= np.linalg.norm(u_trans)

    w = width / 2.0
    h = height / 2.0
    corners = []
    for dl in [0.0, L]:
        for dlat in [-w, w]:
            for dtrans in [-h, h]:
                corners.append(p0 + dl * u_long + dlat * u_lat + dtrans * u_trans)
    corners = np.array(corners)

    faces = [
        [0, 1, 3, 2],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 3, 7, 5]
    ]
    return [corners[idx] for idx in faces]

def draw_axes(ax, origin, R, length=0.1, name=""):
    colors = ["#e63946", "#2a9d8f", "#457b9d"]
    for i in range(3):
        v = origin + length * R[:, i]
        ax.plot([origin[0], v[0]], [origin[1], v[1]], [origin[2], v[2]], color=colors[i], linewidth=2.0)
    if name:
        ax.text(origin[0], origin[1], origin[2] + 0.02, name, color="#ffffff", fontsize=8)

def plot_robot(arm, q, save_path=None, show=True):
    q1, q2, q3 = q[0], q[1], q[2]

    A1 = np.array([
        [np.cos(q1), 0,  np.sin(q1), 0],
        [np.sin(q1), 0, -np.cos(q1), 0],
        [0,          1,  0,          arm.l1],
        [0,          0,  0,          1]
    ])
    A2 = np.array([
        [np.cos(q2), -np.sin(q2), 0, arm.l2 * np.cos(q2)],
        [np.sin(q2),  np.cos(q2), 0, arm.l2 * np.sin(q2)],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])
    A3 = np.array([
        [np.cos(q3), -np.sin(q3), 0, arm.l3 * np.cos(q3)],
        [np.sin(q3),  np.cos(q3), 0, arm.l3 * np.sin(q3)],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])

    T1 = A1
    T2 = A1 @ A2
    T3 = A1 @ A2 @ A3

    P0 = np.array([0.0, 0.0, 0.0])
    P1 = T1[0:3, 3]
    P2 = T2[0:3, 3]
    P3 = T3[0:3, 3]

    z0 = np.array([0.0, 0.0, 1.0])
    z1 = T1[0:3, 2]
    z2 = T2[0:3, 2]
    tool_dir = T3[0:3, 0]
    tool_lat = T3[0:3, 2]

    fig = plt.figure(figsize=(10, 8), facecolor="#14171d")
    ax = fig.add_subplot(111, projection="3d", facecolor="#14171d")

    gx, gy = np.meshgrid(np.linspace(-0.7, 0.7, 15), np.linspace(-0.7, 0.7, 15))
    ax.plot_wireframe(gx, gy, np.zeros_like(gx), color="#2d333f", linewidth=0.6)

    for r in [0.2, 0.4, 0.6]:
        th = np.linspace(0, 2 * np.pi, 50)
        ax.plot(r * np.cos(th), r * np.sin(th), np.zeros_like(th), color="#3b4352", linestyle="--", linewidth=0.8)

    xb, yb, zb = get_cylinder(np.array([0, 0, 0]), np.array([0, 0, 0.03]), 0.16)
    ax.plot_surface(xb, yb, zb, color="#212529", shade=True)
    xd, yd, zd = get_disc(np.array([0, 0, 0.03]), z0, 0.16)
    ax.plot_surface(xd, yd, zd, color="#212529", shade=True)

    xp, yp, zp = get_cylinder(np.array([0, 0, 0.03]), np.array([0, 0, arm.l1 * 0.45]), 0.09)
    ax.plot_surface(xp, yp, zp, color="#343a40", shade=True)

    xr, yr, zr = get_cylinder(np.array([0, 0, arm.l1 * 0.45 - 0.015]), np.array([0, 0, arm.l1 * 0.45 + 0.015]), 0.105)
    ax.plot_surface(xr, yr, zr, color="#adb5bd", shade=True)

    xt, yt, zt = get_cylinder(np.array([0, 0, arm.l1 * 0.45 + 0.015]), P1 - np.array([0, 0, 0.03]), 0.08)
    ax.plot_surface(xt, yt, zt, color="#2b2d42", shade=True)

    p_j2_a = P1 - 0.08 * z1
    p_j2_b = P1 + 0.08 * z1
    xj2, yj2, zj2 = get_cylinder(p_j2_a, p_j2_b, 0.07)
    ax.plot_surface(xj2, yj2, zj2, color="#ced4da", shade=True)

    for pt, nd in [(p_j2_a, -z1), (p_j2_b, z1)]:
        xc, yc, zc = get_cylinder(pt, pt + 0.015 * nd, 0.065)
        ax.plot_surface(xc, yc, zc, color="#495057", shade=True)

    f_arm2 = get_box(P1, P2, 0.07, 0.06, z1)
    ax.add_collection3d(Poly3DCollection(f_arm2, facecolors="#f77f00", edgecolors="#b25e00", linewidths=0.6))

    p_j3_a = P2 - 0.07 * z2
    p_j3_b = P2 + 0.07 * z2
    xj3, yj3, zj3 = get_cylinder(p_j3_a, p_j3_b, 0.055)
    ax.plot_surface(xj3, yj3, zj3, color="#ced4da", shade=True)

    for pt, nd in [(p_j3_a, -z2), (p_j3_b, z2)]:
        xc, yc, zc = get_cylinder(pt, pt + 0.012 * nd, 0.05)
        ax.plot_surface(xc, yc, zc, color="#495057", shade=True)

    f_arm3 = get_box(P2, P3, 0.055, 0.045, z2)
    ax.add_collection3d(Poly3DCollection(f_arm3, facecolors="#f77f00", edgecolors="#b25e00", linewidths=0.6))

    xf, yf, zf = get_cylinder(P3, P3 + 0.02 * tool_dir, 0.04)
    ax.plot_surface(xf, yf, zf, color="#e9ecef", shade=True)

    gb_s = P3 + 0.02 * tool_dir
    gb_e = gb_s + 0.04 * tool_dir
    f_gb = get_box(gb_s, gb_e, 0.07, 0.035, tool_lat)
    ax.add_collection3d(Poly3DCollection(f_gb, facecolors="#495057", edgecolors="#212529", linewidths=0.5))

    for sgn in [-1.0, 1.0]:
        f_s = gb_e + sgn * 0.022 * tool_lat
        f_e = f_s + 0.055 * tool_dir
        box_f = get_box(f_s, f_e, 0.012, 0.02, tool_lat)
        ax.add_collection3d(Poly3DCollection(box_f, facecolors="#ced4da", edgecolors="#343a40", linewidths=0.5))
        
        tip_s = f_e - 0.018 * tool_dir
        box_tip = get_box(tip_s, f_e, 0.014, 0.022, tool_lat)
        ax.add_collection3d(Poly3DCollection(box_tip, facecolors="#00b4d8", edgecolors="#0077b6", linewidths=0.5))

    draw_axes(ax, P0, np.eye(3), 0.12, "{0}")
    draw_axes(ax, P1, T1[0:3, 0:3], 0.08, "{1}")
    draw_axes(ax, P2, T2[0:3, 0:3], 0.08, "{2}")
    P_tip = P3 + 0.115 * tool_dir
    draw_axes(ax, P_tip, T3[0:3, 0:3], 0.1, "{Tool}")

    ax.set_xlim(-0.7, 0.7)
    ax.set_ylim(-0.7, 0.7)
    ax.set_zlim(0.0, 1.1)
    ax.set_box_aspect([1.4, 1.4, 1.1])
    ax.view_init(elev=22, azim=45)

    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(color="#2d333f", linestyle=":", linewidth=0.5)

    ax.set_xlabel("X (m)", color="#ced4da")
    ax.set_ylabel("Y (m)", color="#ced4da")
    ax.set_zlabel("Z (m)", color="#ced4da")
    ax.tick_params(colors="#6c757d", labelsize=8)

    q_deg = np.rad2deg(q)
    ax.set_title(f"3-DOF Manipulator: q = [{q_deg[0]:.1f}°, {q_deg[1]:.1f}°, {q_deg[2]:.1f}°]", color="#ffffff", fontsize=11)

    if save_path:
        plt.savefig(save_path, dpi=300, facecolor=fig.get_facecolor(), bbox_inches="tight")

    if show:
        plt.show()

if __name__ == "__main__":
    arm = Manipulator3DOF(l1=0.5, l2=0.4, l3=0.3)
    
    q_angles = np.deg2rad([35.0, 45.0, 30.0])
    
    T_tb = arm.fkine_toolbox(q_angles)
    T_an, pos_an = arm.fkine_analytical(q_angles)

    print("Joint Angles (deg):", np.rad2deg(q_angles))
    print("Toolbox FK Pose Matrix:")
    print(np.round(T_tb.A, 4))
    print("Analytical FK Position (X, Y, Z):")
    print(np.round(pos_an, 4))

    plot_robot(arm, q_angles, save_path="robot_model.png", show=False)
