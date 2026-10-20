"""
CAD-Like 3D Visualizer for 3-DOF Robotic Manipulator
===================================================
Renders a highly detailed, visually accurate 3D CAD-like model:
- Solid volumetric links with beveled geometric sections
- Cylindrical motor housings / harmonic drive actuators for all 3 revolute joints
- Ground foundation with mounting flange & bolt circle
- End-effector industrial tool flange & 2-finger parallel gripper
- Triad coordinate axes (X-Red, Y-Green, Z-Blue) for base and tool
- Engineering HUD displaying joint angles and cartesian coordinates
"""

from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_arm import ThreeDOFManipulator


def create_cylinder_mesh(
    p0: np.ndarray,
    p1: np.ndarray,
    radius: float,
    resolution: int = 32
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate 3D surface mesh for a cylinder connecting p0 to p1.
    """
    v = p1 - p0
    mag = np.linalg.norm(v)
    if mag < 1e-9:
        v = np.array([0, 0, 1.0])
        mag = 1.0
    v = v / mag

    # Find two orthogonal vectors perpendicular to v
    not_v = np.array([1, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1, 0])
    n1 = np.cross(v, not_v)
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(v, n1)

    theta = np.linspace(0, 2 * np.pi, resolution)
    z_line = np.linspace(0, mag, 2)
    theta_grid, z_grid = np.meshgrid(theta, z_line)

    x = p0[0] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[0] +
                          np.outer(np.sin(theta), np.ones(2)) * n2[0]).T + z_grid * v[0]
    y = p0[1] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[1] +
                          np.outer(np.sin(theta), np.ones(2)) * n2[1]).T + z_grid * v[1]
    z = p0[2] + radius * (np.outer(np.cos(theta), np.ones(2)) * n1[2] +
                          np.outer(np.sin(theta), np.ones(2)) * n2[2]).T + z_grid * v[2]

    return x, y, z


def create_disc_mesh(
    center: np.ndarray,
    normal: np.ndarray,
    radius: float,
    resolution: int = 32
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate circular disc mesh perpendicular to normal."""
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


def create_box_mesh(
    p0: np.ndarray,
    p1: np.ndarray,
    width: float,
    height: float,
    lateral_axis: np.ndarray
) -> List[np.ndarray]:
    """
    Generate 6 polygon faces of a rectangular solid link connecting p0 to p1.
    """
    length_vec = p1 - p0
    L = np.linalg.norm(length_vec)
    if L < 1e-9:
        return []
    u_long = length_vec / L

    # Lateral axis
    u_lat = lateral_axis / np.linalg.norm(lateral_axis)
    # Transverse axis (mutually orthogonal)
    u_trans = np.cross(u_long, u_lat)
    u_trans /= np.linalg.norm(u_trans)

    # 8 Corner vertices
    w = width / 2.0
    h = height / 2.0
    corners = []
    for dl in [0.0, L]:
        for dlat in [-w, w]:
            for dtrans in [-h, h]:
                pt = p0 + dl * u_long + dlat * u_lat + dtrans * u_trans
                corners.append(pt)
    corners = np.array(corners)

    # Faces indices
    faces_idx = [
        [0, 1, 3, 2],  # Bottom cap (p0)
        [4, 5, 7, 6],  # Top cap (p1)
        [0, 1, 5, 4],  # Side 1
        [2, 3, 7, 6],  # Side 2
        [0, 2, 6, 4],  # Side 3
        [1, 3, 7, 5],  # Side 4
    ]
    return [corners[idx] for idx in faces_idx]


def render_cad_manipulator(
    arm: ThreeDOFManipulator,
    q: np.ndarray | List[float],
    title: str = "3-DOF Articulated Robot - CAD Digital Twin",
    save_path: Optional[str] = "cad_robot_render.png",
    show: bool = True
):
    """
    Render high-fidelity 3D CAD model of the 3-DOF robot with:
    - Base ground plane & foundation mount
    - 3 Revolute actuators with motor caps and bearing housings
    - Form-shaped structural arm links
    - End-effector tool flange & parallel mechanical gripper
    - Coordinate frames and HUD telemetry
    """
    q_rad = np.asarray(q, dtype=float)
    q1, q2, q3 = q_rad[0], q_rad[1], q_rad[2]

    # Compute Forward Kinematics frames
    T0 = np.eye(4)
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

    # Key Joint Positions
    P0 = np.array([0.0, 0.0, 0.0])              # Base ground origin
    P_pedestal_mid = np.array([0.0, 0.0, arm.l1 * 0.45])
    P1 = T1[0:3, 3]                             # Shoulder Joint 2 position
    P2 = T2[0:3, 3]                             # Elbow Joint 3 position
    P3 = T3[0:3, 3]                             # Wrist / Tool Flange position

    # Actuator Axis Orientations
    z0_axis = np.array([0.0, 0.0, 1.0])         # Joint 1 axis
    z1_axis = T1[0:3, 2]                        # Joint 2 axis
    z2_axis = T2[0:3, 2]                        # Joint 3 axis
    tool_approach = T3[0:3, 0]                  # Tool X/pointing axis
    tool_normal = T3[0:3, 1]
    tool_slide = T3[0:3, 2]

    # Create Matplotlib 3D figure
    fig = plt.figure(figsize=(14, 11), facecolor="#0e1117")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0e1117")

    # Styling Palette (Modern Industrial CAD)
    C_BASE_DARK = "#212529"       # Heavy cast iron base
    C_TURRET = "#343a40"          # Dark steel turret
    C_ARM_PRIMARY = "#f77f00"     # High-visibility Industrial Robot Orange
    C_ARM_ACCENT = "#2b2d42"      # Carbon grey accents
    C_ACTUATOR = "#ced4da"        # Precision machined alloy/silver
    C_ACTUATOR_CAP = "#495057"    # Actuator motor end caps
    C_CHROME = "#e9ecef"          # Chrome bearings
    C_GRIPPER = "#495057"         # Mechanical gripper body
    C_JAW = "#00b4d8"             # Cyan gripper fingertips

    # -------------------------------------------------------------
    # 1. GROUND PLANE & FOUNDATION
    # -------------------------------------------------------------
    ground_range = np.linspace(-0.8, 0.8, 17)
    GX, GY = np.meshgrid(ground_range, ground_range)
    GZ = np.zeros_like(GX)
    ax.plot_wireframe(GX, GY, GZ, color="#2b303c", linewidth=0.6, alpha=0.6)

    # Concentric foundation circular rings
    for r in [0.25, 0.50, 0.75]:
        circ_th = np.linspace(0, 2 * np.pi, 60)
        ax.plot(r * np.cos(circ_th), r * np.sin(circ_th), np.zeros_like(circ_th),
                color="#3d4452", linestyle="--", linewidth=0.8)

    # Heavy Base Flange (Ground Mount Disc)
    x_bf, y_bf, z_bf = create_cylinder_mesh(
        np.array([0, 0, 0.0]), np.array([0, 0, 0.035]), radius=0.18
    )
    ax.plot_surface(x_bf, y_bf, z_bf, color=C_BASE_DARK, shade=True, alpha=0.95)
    xd, yd, zd = create_disc_mesh(np.array([0, 0, 0.035]), z0_axis, radius=0.18)
    ax.plot_surface(xd, yd, zd, color=C_BASE_DARK, shade=True)

    # Foundation Anchor Bolts (8 bolts around flange perimeter)
    bolt_th = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    for b_th in bolt_th:
        bx = 0.14 * np.cos(b_th)
        by = 0.14 * np.sin(b_th)
        x_bt, y_bt, z_bt = create_cylinder_mesh(
            np.array([bx, by, 0.035]), np.array([bx, by, 0.048]), radius=0.012
        )
        ax.plot_surface(x_bt, y_bt, z_bt, color=C_CHROME, shade=True)

    # -------------------------------------------------------------
    # 2. JOINT 1 (BASE PEDESTAL & ROTATING TURRET)
    # -------------------------------------------------------------
    # Lower stationary pedestal column
    x_ped, y_ped, z_ped = create_cylinder_mesh(
        np.array([0, 0, 0.035]), P_pedestal_mid, radius=0.10
    )
    ax.plot_surface(x_ped, y_ped, z_ped, color=C_TURRET, shade=True)

    # Slewing ring / Bearing collar
    x_ring, y_ring, z_ring = create_cylinder_mesh(
        P_pedestal_mid - np.array([0, 0, 0.015]),
        P_pedestal_mid + np.array([0, 0, 0.015]),
        radius=0.115
    )
    ax.plot_surface(x_ring, y_ring, z_ring, color=C_ACTUATOR, shade=True)

    # Rotating upper turret body (up to shoulder joint)
    x_turret, y_turret, z_turret = create_cylinder_mesh(
        P_pedestal_mid + np.array([0, 0, 0.015]),
        P1 - np.array([0, 0, 0.04]),
        radius=0.088
    )
    ax.plot_surface(x_turret, y_turret, z_turret, color=C_ARM_ACCENT, shade=True)

    # -------------------------------------------------------------
    # 3. JOINT 2 (SHOULDER ACTUATOR & MOTOR HOUSING)
    # -------------------------------------------------------------
    # Transverse cylinder actuator along z1_axis
    actuator_half_len = 0.09
    p_act2_start = P1 - actuator_half_len * z1_axis
    p_act2_end = P1 + actuator_half_len * z1_axis
    x_sh, y_sh, z_sh = create_cylinder_mesh(p_act2_start, p_act2_end, radius=0.075)
    ax.plot_surface(x_sh, y_sh, z_sh, color=C_ACTUATOR, shade=True)

    # Motor End Caps
    for pt, n_dir in [(p_act2_start, -z1_axis), (p_act2_end, z1_axis)]:
        xc, yc, zc = create_cylinder_mesh(pt, pt + 0.02 * n_dir, radius=0.070)
        ax.plot_surface(xc, yc, zc, color=C_ACTUATOR_CAP, shade=True)
        xd, yd, zd = create_disc_mesh(pt + 0.02 * n_dir, n_dir, radius=0.070)
        ax.plot_surface(xd, yd, zd, color=C_ACTUATOR_CAP, shade=True)

    # -------------------------------------------------------------
    # 4. LINK 2 (UPPER ARM - SOLID STRUCTURAL BEAM)
    # -------------------------------------------------------------
    arm2_faces = create_box_mesh(
        P1, P2, width=0.075, height=0.065, lateral_axis=z1_axis
    )
    arm2_poly = Poly3DCollection(arm2_faces, facecolors=C_ARM_PRIMARY, edgecolors="#c05621",
                                 linewidths=0.8, alpha=0.95)
    ax.add_collection3d(arm2_poly)

    # Center structural rib accent
    arm2_rib = create_box_mesh(
        P1 + 0.08 * (P2 - P1), P2 - 0.08 * (P2 - P1),
        width=0.085, height=0.025, lateral_axis=z1_axis
    )
    arm2_rib_poly = Poly3DCollection(arm2_rib, facecolors=C_ARM_ACCENT, edgecolors="#1a1c23",
                                     linewidths=0.5, alpha=0.9)
    ax.add_collection3d(arm2_rib_poly)

    # -------------------------------------------------------------
    # 5. JOINT 3 (ELBOW ACTUATOR & PIVOT HOUSING)
    # -------------------------------------------------------------
    p_act3_start = P2 - 0.075 * z2_axis
    p_act3_end = P2 + 0.075 * z2_axis
    x_el, y_el, z_el = create_cylinder_mesh(p_act3_start, p_act3_end, radius=0.060)
    ax.plot_surface(x_el, y_el, z_el, color=C_ACTUATOR, shade=True)

    for pt, n_dir in [(p_act3_start, -z2_axis), (p_act3_end, z2_axis)]:
        xc, yc, zc = create_cylinder_mesh(pt, pt + 0.015 * n_dir, radius=0.055)
        ax.plot_surface(xc, yc, zc, color=C_ACTUATOR_CAP, shade=True)
        xd, yd, zd = create_disc_mesh(pt + 0.015 * n_dir, n_dir, radius=0.055)
        ax.plot_surface(xd, yd, zd, color=C_ACTUATOR_CAP, shade=True)

    # -------------------------------------------------------------
    # 6. LINK 3 (FOREARM - SLEEK STRUCTURAL BEAM)
    # -------------------------------------------------------------
    arm3_faces = create_box_mesh(
        P2, P3, width=0.060, height=0.050, lateral_axis=z2_axis
    )
    arm3_poly = Poly3DCollection(arm3_faces, facecolors=C_ARM_PRIMARY, edgecolors="#c05621",
                                 linewidths=0.8, alpha=0.95)
    ax.add_collection3d(arm3_poly)

    # Forearm accent plate
    arm3_acc = create_box_mesh(
        P2 + 0.06 * (P3 - P2), P3 - 0.04 * (P3 - P2),
        width=0.068, height=0.018, lateral_axis=z2_axis
    )
    arm3_acc_poly = Poly3DCollection(arm3_acc, facecolors=C_ARM_ACCENT, edgecolors="#1a1c23",
                                     linewidths=0.5, alpha=0.9)
    ax.add_collection3d(arm3_acc_poly)

    # -------------------------------------------------------------
    # 7. END-EFFECTOR TOOL FLANGE & 2-FINGER PARALLEL GRIPPER
    # -------------------------------------------------------------
    # Tool mounting flange plate
    x_fl, y_fl, z_fl = create_cylinder_mesh(
        P3, P3 + 0.025 * tool_approach, radius=0.045
    )
    ax.plot_surface(x_fl, y_fl, z_fl, color=C_CHROME, shade=True)

    # Gripper central body
    p_grip_base = P3 + 0.025 * tool_approach
    p_grip_front = p_grip_base + 0.050 * tool_approach
    grip_box = create_box_mesh(
        p_grip_base, p_grip_front, width=0.080, height=0.040, lateral_axis=tool_slide
    )
    grip_poly = Poly3DCollection(grip_box, facecolors=C_GRIPPER, edgecolors="#212529",
                                 linewidths=0.6, alpha=0.95)
    ax.add_collection3d(grip_poly)

    # Parallel Gripper Fingers (Finger 1 & Finger 2)
    finger_span = 0.026
    finger_length = 0.065
    finger_thick = 0.014
    for sign in [-1.0, 1.0]:
        f_start = p_grip_front + sign * finger_span * tool_slide
        f_end = f_start + finger_length * tool_approach
        f_box = create_box_mesh(f_start, f_end, width=finger_thick, height=0.022, lateral_axis=tool_slide)
        f_poly = Poly3DCollection(f_box, facecolors=C_ACTUATOR, edgecolors="#2b2d42", linewidths=0.5)
        ax.add_collection3d(f_poly)

        # High-friction fingertip pads
        tip_start = f_end - 0.020 * tool_approach
        tip_box = create_box_mesh(tip_start, f_end, width=finger_thick * 1.1, height=0.024, lateral_axis=tool_slide)
        tip_poly = Poly3DCollection(tip_box, facecolors=C_JAW, edgecolors="#0077b6", linewidths=0.5)
        ax.add_collection3d(tip_poly)

    # -------------------------------------------------------------
    # 8. COORDINATE FRAMES (TRIADS: X-Red, Y-Green, Z-Blue)
    # -------------------------------------------------------------
    def draw_triad(origin: np.ndarray, R: np.ndarray, length: float = 0.12, name: str = ""):
        cols = [("#e63946", "X"), ("#2a9d8f", "Y"), ("#457b9d", "Z")]
        for i in range(3):
            vec = origin + length * R[:, i]
            ax.plot([origin[0], vec[0]], [origin[1], vec[1]], [origin[2], vec[2]],
                    color=cols[i][0], linewidth=2.5)
        if name:
            ax.text(origin[0], origin[1], origin[2] + 0.03, f" {name}",
                    color="#ffffff", fontsize=9, fontweight="bold")

    # Base Frame {0}
    draw_triad(P0, np.eye(3), length=0.15, name="{0} Base")
    # Shoulder Frame {1}
    draw_triad(P1, T1[0:3, 0:3], length=0.10, name="{1}")
    # Elbow Frame {2}
    draw_triad(P2, T2[0:3, 0:3], length=0.10, name="{2}")
    # End-Effector / Tool Frame {3}
    P_tip = P3 + (0.025 + 0.050 + finger_length) * tool_approach
    draw_triad(P_tip, T3[0:3, 0:3], length=0.12, name="{EE} Tool")

    # -------------------------------------------------------------
    # 9. KINEMATICS TELEMETRY HUD OVERLAY
    # -------------------------------------------------------------
    q_deg = np.rad2deg(q_rad)
    hud_text = (
        f"3-DOF ROBOT TELEMETRY\n"
        f"────────────────────────\n"
        f"Joint 1 (Waist)    : {q_deg[0]:+6.1f}°\n"
        f"Joint 2 (Shoulder) : {q_deg[1]:+6.1f}°\n"
        f"Joint 3 (Elbow)    : {q_deg[2]:+6.1f}°\n"
        f"────────────────────────\n"
        f"End-Effector Pos:\n"
        f"  X : {P3[0]:+7.3f} m\n"
        f"  Y : {P3[1]:+7.3f} m\n"
        f"  Z : {P3[2]:+7.3f} m\n"
        f"Reach R : {np.linalg.norm(P3):6.3f} m\n"
        f"────────────────────────\n"
        f"Status: KINEMATICS OK ✓"
    )
    ax.text2D(
        0.03, 0.94, hud_text, transform=ax.transAxes,
        fontsize=9, color="#e0e1dd", family="monospace",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#1b263b", edgecolor="#415a77", alpha=0.88)
    )

    # Title & Legend
    ax.set_title(
        f"{title}\nq = [{q_deg[0]:.1f}°, {q_deg[1]:.1f}°, {q_deg[2]:.1f}°]",
        fontsize=13, fontweight="bold", color="#f8f9fa", pad=15
    )

    # View limits & Equal aspect ratio
    limit = 0.85
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0.0, 1.25)
    ax.set_box_aspect([1.7, 1.7, 1.25])

    # View Angle
    ax.view_init(elev=24, azim=48)

    # Dark Theme Axis Styling
    ax.xaxis.pane.set_edgecolor("#212529")
    ax.yaxis.pane.set_edgecolor("#212529")
    ax.zaxis.pane.set_edgecolor("#212529")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(color="#343a40", linestyle=":", linewidth=0.5)

    ax.set_xlabel("X (meters)", color="#adb5bd", labelpad=8)
    ax.set_ylabel("Y (meters)", color="#adb5bd", labelpad=8)
    ax.set_zlabel("Z (meters)", color="#adb5bd", labelpad=8)
    ax.tick_params(colors="#6c757d", labelsize=8)

    if save_path:
        plt.savefig(save_path, dpi=300, facecolor=fig.get_facecolor(), bbox_inches="tight")
        print(f"[✓] High-resolution CAD 3D model saved to: {save_path}")

    if show:
        plt.show()

    return fig, ax
