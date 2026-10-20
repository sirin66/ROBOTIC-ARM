# 3-DOF Revolute Robotic Manipulator (RRR)

A robotics engineering implementation of a 3-DOF spatial articulated manipulator (waist-shoulder-elbow) using Peter Corke's `roboticstoolbox-python` and standard Denavit-Hartenberg (DH) parameters.

---

## 1. Kinematic Architecture

- **Joint 1 (Base / Waist)**: Revolute joint about the vertical $Z_0$ axis (Azimuth angle $q_1$).
- **Joint 2 (Shoulder)**: Revolute joint about horizontal $Z_1$ axis (Elevation pitch angle $q_2$).
- **Joint 3 (Elbow)**: Revolute joint about horizontal $Z_2$ axis (Flexion/extension pitch angle $q_3$).

### Link Dimensions (Configurable)
- $L_1 = 0.50\text{ m}$ (Base height)
- $L_2 = 0.40\text{ m}$ (Upper arm length)
- $L_3 = 0.30\text{ m}$ (Forearm length)

---

## 2. Denavit-Hartenberg (Standard DH) Parameters

According to the standard DH convention:

| Link $i$ | Joint Variable $\theta_i$ | Link Offset $d_i$ | Link Length $a_i$ | Link Twist $\alpha_i$ |
| :---: | :---: | :---: | :---: | :---: |
| **1** | $q_1$ | $L_1 = 0.50\text{ m}$ | $0$ | $+90^\circ$ ($+\pi/2\text{ rad}$) |
| **2** | $q_2$ | $0$ | $L_2 = 0.40\text{ m}$ | $0^\circ$ ($0\text{ rad}$) |
| **3** | $q_3$ | $0$ | $L_3 = 0.30\text{ m}$ | $0^\circ$ ($0\text{ rad}$) |

---

## 3. Forward Kinematics Formulation

The general homogeneous transformation matrix $A_i$ from frame $i-1$ to frame $i$ is:

$$A_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

Multiplying the consecutive transformations $T_0^3 = A_1 A_2 A_3$ yields:

$$T_0^3 = \begin{bmatrix}
c_1 c_{23} & -c_1 s_{23} & s_1 & c_1 (L_2 c_2 + L_3 c_{23}) \\
s_1 c_{23} & -s_1 s_{23} & -c_1 & s_1 (L_2 c_2 + L_3 c_{23}) \\
s_{23} & c_{23} & 0 & L_1 + L_2 s_2 + L_3 s_{23} \\
0 & 0 & 0 & 1
\end{bmatrix}$$

Where:
- $c_i = \cos(q_i)$, $s_i = \sin(q_i)$
- $c_{23} = \cos(q_2 + q_3)$, $s_{23} = \sin(q_2 + q_3)$

### End-Effector Position:
$$\begin{cases}
p_x = \cos(q_1) \cdot \left[ L_2 \cos(q_2) + L_3 \cos(q_2 + q_3) \right] \\
p_y = \sin(q_1) \cdot \left[ L_2 \cos(q_2) + L_3 \cos(q_2 + q_3) \right] \\
p_z = L_1 + L_2 \sin(q_2) + L_3 \sin(q_2 + q_3)
\end{cases}$$

---

## 4. Setup & Execution

### Prerequisites
- Python 3.10+ installed

### Step 1: Clone / Navigate to directory
```bash
cd "c:\Users\sirin\Desktop\robotic arm"
```

### Step 2: Create and activate virtual environment
```bash
# Windows PowerShell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### Step 3: Install dependencies
```bash
pip install -r requirements.txt
```

### Step 4: Run the kinematics & visualization
```bash
python main.py
```

### Step 5: Render the High-Definition 3D CAD-like Model
```bash
# Render and save high-resolution CAD image:
python run_cad.py

# Specify custom joint angles (e.g. q1=45°, q2=30°, q3=-20°):
python run_cad.py --q 45 30 -20

# Open interactive rotatable 3D CAD window:
python run_cad.py --show
```

Generated 3D renderings are saved to `manipulator_3d_pose.png` and `cad_robot_render.png`.
