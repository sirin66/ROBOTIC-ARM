from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_arm import ThreeDOFManipulator

def create_cylinder_mesh(p0: np.ndarray, p1: np.ndarray, radius: float, resolution: int = 32) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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

    theta = np.linspace(0, 2 * np.pi, resolution)
    z_line = np.linspace(0, mag, 2)
    theta_grid, z_grid = np.meshgrid(theta, z_line)

    x = p0[0] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[0] + np.outer(np.sin(theta), np.ones(2)) * n2[0]).T + z_grid * v[0]
    y = p0[1] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[1] + np.outer(np.sin(theta), np.ones(2)) * n2[1]).T + z_grid * v[1]
    z = p0[2] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[2] + np.outer(np.sin(theta), np.ones(2)) * n2[2]).T + z_grid * v[2]
    return x, y, z

def create_disc_mesh(center: np.ndarray, normal: np.ndarray, radius: float, resolution: int = 32) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = normal / np.linalg.norm(normal)
    not_n = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(n, not_n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)

    r = np.linspace(0, radius, 4)
    theta = np.linspace(0, 2 * np.pi, resolution)
    r_grid, th_grid = np.meshgrid(r, theta)

    x = center[0] + r_grid * (np.cos(th_grid) * u[0] + np.sin(th_grid) * v[0])
    y = center[1] + r_grid * (np.cos(th_grid) * u[1] + np.sin(th_grid) * v[1])
    z = center[2] + r_grid * (np.cos(th_grid) * u[2] + np.sin(th_grid) * v[2])
    return x, y, z

def create_box_mesh(p0: np.ndarray, p1: np.ndarray, width: float, height: float, lateral_axis: np.ndarray) -> List[np.ndarray]:
    length_vec = p1 - p0
    L = np.linalg.norm(length_vec)
    if L < 1e-9:
        return []
    u_long = length_vec / L

    u_lat = lateral_axis / np.linalg.norm(lateral_axis)
    u_trans = np.cross(u_long, u_lat)
    u_trans /= np.linalg.norm(u_trans)

    w = width / 2.0
    h = height / 2.0
    corners = []
    for dl in [0.0, L]:
        for dlat in [-w, w]:
            for dtrans in [-h, h]:
                pt = p0 + dl * u_long + dlat * u_lat + dtrans * u_trans
                corners.append(pt)
    corners = np.array(corners)

    faces_idx = [
        [0, 1, 3, 2],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 2, 6, 4],
        [1, 3, 7, 5],
    ]
    return [corners[idx] for idx in faces_idx]

def render_cad_manipulator(arm: ThreeDOFManipulator, q: np.ndarray | List[float], title: str = "3-DOF Articulated Robot", save_path: Optional[str] = "cad_robot_render.png", show: bool = True):
    q_rad = np.asarray(q, dtype=float)
    q1, q2, q3 = q_rad[0], q_rad[1], q_rad[2]

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
    P_pedestal_mid = np.array([0.0, 0.0, arm.l1 * 0.45])
    P1 = T1[0:3, 3]
    P2 = T2[0:3, 3]
    P3 = T3[0:3, 3]

    z0_axis = np.array([0.0, 0.0, 1.0])
    z1_axis = T1[0:3, 2]
    z2_axis = T2[0:3, 2]
    tool_approach = T3[0:3, 0]
    tool_slide = T3[0:3, 2]

    fig = plt.figure(figsize=(14, 11), facecolor="#0e1117")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0e1117")

    C_BASE_DARK = "#212529"
    C_TURRET = "#343a40"
    C_ARM_PRIMARY = "#f77f00"
    C_ARM_ACCENT = "#2b2d42"
    C_ACTUATOR = "#ced4da"
    C_ACTUATOR_CAP = "#495057"
    C_CHROME = "#e9ecef"
    C_GRIPPER = "#495057"
    C_JAW = "#00b4d8"

    ground_range = np.linspace(-0.8, 0.8, 17)
    GX, GY = np.meshgrid(ground_range, ground_range)
    GZ = np.zeros_like(GX)
    ax.plot_wireframe(GX, GY, GZ, color="#2b303c", linewidth=0.6, alpha=0.6)

    for r in [0.25, 0.50, 0.75]:
        circ_th = np.linspace(0, 2 * np.pi, 60)
        ax.plot(r * np.cos(circ_th), r * np.sin(circ_th), np.zeros_like(circ_th), color="#3d4452", linestyle="--", linewidth=0.8)

    x_bf, y_bf, z_bf = create_cylinder_mesh(np.array([0, 0, 0.0]), np.array([0, 0, 0.035]), radius=0.18)
    ax.plot_surface(x_bf, y_bf, z_bf, color=C_BASE_DARK, shade=True, alpha=0.95)
    xd, yd, zd = create_disc_mesh(np.array([0, 0, 0.035]), z0_axis, radius=0.18)
    ax.plot_surface(xd, yd, zd, color=C_BASE_DARK, shade=True)

    bolt_th = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    for b_th in bolt_th:
        bx = 0.14 * np.cos(b_th)
        by = 0.14 * np.sin(b_th)
        x_bt, y_bt, z_bt = create_cylinder_mesh(np.array([bx, by, 0.035]), np.array([bx, by, 0.048]), radius=0.012)
        ax.plot_surface(x_bt, y_bt, z_bt, color=C_CHROME, shade=True)

    x_ped, y_ped, z_ped = create_cylinder_mesh(np.array([0, 0, 0.035]), P_pedestal_mid, radius=0.10)
    ax.plot_surface(x_ped, y_ped, z_ped, color=C_TURRET, shade=True)

    x_ring, y_ring, z_ring = create_cylinder_mesh(P_pedestal_mid - np.array([0, 0, 0.015]), P_pedestal_mid + np.array([0, 0, 0.015]), radius=0.115)
    ax.plot_surface(x_ring, y_ring, z_ring, color=C_ACTUATOR, shade=True)

    x_turret, y_turret, z_turret = create_cylinder_mesh(P_pedestal_mid + np.array([0, 0, 0.015]), P1 - np.array([0, 0, 0.04]), radius=0.088)
    ax.plot_surface(x_turret, y_turret, z_turret, color=C_ARM_ACCENT, shade=True)

    actuator_half_len = 0.09
    p_act2_start = P1 - actuator_half_len * z1_axis
    p_act2_end = P1 + actuator_half_len * z1_axis
    x_sh, y_sh, z_sh = create_cylinder_mesh(p_act2_start, p_act2_end, radius=0.075)
    ax.plot_surface(x_sh, y_sh, z_sh, color=C_ACTUATOR, shade=True)

    for pt, n_dir in [(p_act2_start, -z1_axis), (p_act2_end, z1_axis)]:
        xc, yc, zc = create_cylinder_mesh(pt, pt + 0.02 * n_dir, radius=0.070)
        ax.plot_surface(xc, yc, zc, color=C_ACTUATOR_CAP, shade=True)
        xd, yd, zd = create_disc_mesh(pt + 0.02 * n_dir, n_dir, radius=0.070)
        ax.plot_surface(xd, yd, zd, color=C_ACTUATOR_CAP, shade=True)

    arm2_faces = create_box_mesh(P1, P2, width=0.075, height=0.065, lateral_axis=z1_axis)
    arm2_poly = Poly3DCollection(arm2_faces, facecolors=C_ARM_PRIMARY, edgecolors="#c05621", linewidths=0.8, alpha=0.95)
    ax.add_collection3d(arm2_poly)

    arm2_rib = create_box_mesh(P1 + 0.08 * (P2 - P1), P2 - 0.08 * (P2 - P1), width=0.085, height=0.025, lateral_axis=z1_axis)
    arm2_rib_poly = Poly3DCollection(arm2_rib, facecolors=C_ARM_ACCENT, edgecolors="#1a1c23", linewidths=0.5, alpha=0.9)
    ax.add_collection3d(arm2_rib_poly)

    p_act3_start = P2 - 0.075 * z2_axis
    p_act3_end = P2 + 0.075 * z2_axis
    x_el, y_el, z_el = create_cylinder_mesh(p_act3_start, p_act3_end, radius=0.060)
    ax.plot_surface(x_el, y_el, z_el, color=C_ACTUATOR, shade=True)

    for pt, n_dir in [(p_act3_start, -z2_axis), (p_act3_end, z2_axis)]:
        xc, yc, zc = create_cylinder_mesh(pt, pt + 0.015 * n_dir, radius=0.055)
        ax.plot_surface(xc, yc, zc, color=C_ACTUATOR_CAP, shade=True)
        xd, yd, zd = create_disc_mesh(pt + 0.015 * n_dir, n_dir, radius=0.055)
        ax.plot_surface(xd, yd, zd, color=C_ACTUATOR_CAP, shade=True)

    arm3_faces = create_box_mesh(P2, P3, width=0.060, height=0.050, lateral_axis=z2_axis)
    arm3_poly = Poly3DCollection(arm3_faces, facecolors=C_ARM_PRIMARY, edgecolors="#c05621", linewidths=0.8, alpha=0.95)
    ax.add_collection3d(arm3_poly)

    arm3_acc = create_box_mesh(P2 + 0.06 * (P3 - P2), P3 - 0.04 * (P3 - P2), width=0.068, height=0.018, lateral_axis=z2_axis)
    arm3_acc_poly = Poly3DCollection(arm3_acc, facecolors=C_ARM_ACCENT, edgecolors="#1a1c23", linewidths=0.5, alpha=0.9)
    ax.add_collection3d(arm3_acc_poly)

    x_fl, y_fl, z_fl = create_cylinder_mesh(P3, P3 + 0.025 * tool_approach, radius=0.045)
    ax.plot_surface(x_fl, y_fl, z_fl, color=C_CHROME, shade=True)

    p_grip_base = P3 + 0.025 * tool_approach
    p_grip_front = p_grip_base + 0.050 * tool_approach
    grip_box = create_box_mesh(p_grip_base, p_grip_front, width=0.080, height=0.040, lateral_axis=tool_slide)
    grip_poly = Poly3DCollection(grip_box, facecolors=C_GRIPPER, edgecolors="#212529", linewidths=0.6, alpha=0.95)
    ax.add_collection3d(grip_poly)

    finger_span = 0.026
    finger_length = 0.065
    finger_thick = 0.014
    for sign in [-1.0, 1.0]:
        f_start = p_grip_front + sign * finger_span * tool_slide
        f_end = f_start + finger_length * tool_approach
        f_box = create_box_mesh(f_start, f_end, width=finger_thick, height=0.022, lateral_axis=tool_slide)
        f_poly = Poly3DCollection(f_box, facecolors=C_ACTUATOR, edgecolors="#2b2d42", linewidths=0.5)
        ax.add_collection3d(f_poly)

        tip_start = f_end - 0.020 * tool_approach
        tip_box = create_box_mesh(tip_start, f_end, width=finger_thick * 1.1, height=0.024, lateral_axis=tool_slide)
        tip_poly = Poly3DCollection(tip_box, facecolors=C_JAW, edgecolors="#0077b6", linewidths=0.5)
        ax.add_collection3d(tip_poly)

    def draw_triad(origin: np.ndarray, R: np.ndarray, length: float = 0.12, name: str = ""):
        cols = [("#e63946", "X"), ("#2a9d8f", "Y"), ("#457b9d", "Z")]
        for i in range(3):
            vec = origin + length * R[:, i]
            ax.plot([origin[0], vec[0]], [origin[1], vec[1]], [origin[2], vec[2]], color=cols[i][0], linewidth=2.5)
        if name:
            ax.text(origin[0], origin[1], origin[2] + 0.03, f" {name}", color="#ffffff", fontsize=9, fontweight="bold")

    draw_triad(P0, np.eye(3), length=0.15, name="{0} Base")
    draw_triad(P1, T1[0:3, 0:3], length=0.10, name="{1}")
    draw_triad(P2, T2[0:3, 0:3], length=0.10, name="{2}")
    P_tip = P3 + (0.025 + 0.050 + finger_length) * tool_approach
    draw_triad(P_tip, T3[0:3, 0:3], length=0.12, name="{EE} Tool")

    q_deg = np.rad2deg(q_rad)
    hud_text = (
        f"Joint 1 : {q_deg[0]:+6.1f}°\n"
        f"Joint 2 : {q_deg[1]:+6.1f}°\n"
        f"Joint 3 : {q_deg[2]:+6.1f}°\n"
        f"Pos X   : {P3[0]:+7.3f} m\n"
        f"Pos Y   : {P3[1]:+7.3f} m\n"
        f"Pos Z   : {P3[2]:+7.3f} m\n"
        f"Reach R : {np.linalg.norm(P3):6.3f} m"
    )
    ax.text2D(0.03, 0.94, hud_text, transform=ax.transAxes, fontsize=9, color="#e0e1dd", family="monospace",
              bbox=dict(boxstyle="round,pad=0.6", facecolor="#1b263b", edgecolor="#415a77", alpha=0.88))

    ax.set_title(f"{title} | q = [{q_deg[0]:.1f}°, {q_deg[1]:.1f}°, {q_deg[2]:.1f}°]", fontsize=12, fontweight="bold", color="#f8f9fa", pad=15)

    limit = 0.85
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0.0, 1.25)
    ax.set_box_aspect([1.7, 1.7, 1.25])
    ax.view_init(elev=24, azim=48)

    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(color="#343a40", linestyle=":", linewidth=0.5)

    ax.set_xlabel("X (m)", color="#adb5bd", labelpad=8)
    ax.set_ylabel("Y (m)", color="#adb5bd", labelpad=8)
    ax.set_zlabel("Z (m)", color="#adb5bd", labelpad=8)
    ax.tick_params(colors="#6c757d", labelsize=8)

    if save_path:
        plt.savefig(save_path, dpi=300, facecolor=fig.get_facecolor(), bbox_inches="tight")

    if show:
        plt.show()

    return fig, ax
