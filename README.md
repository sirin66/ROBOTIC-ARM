# 3-DOF Revolute Robotic Manipulator (RRR)

A robotics engineering implementation of a 3-DOF spatial articulated manipulator (waist-shoulder-elbow) featuring:
- Forward kinematics formulation using standard Denavit-Hartenberg (DH) parameters
- Numerical calculation via Peter Corke's `roboticstoolbox-python`
- Detailed 3D CAD volumetric rendering using Matplotlib
- Real-time interactive 3D Web Interface using Flask and Three.js

---

## Kinematic Architecture

- **Joint 1 (Base / Waist)**: Revolute joint about vertical $Z_0$ axis (Azimuth $q_1$)
- **Joint 2 (Shoulder)**: Revolute joint about horizontal $Z_1$ axis (Elevation pitch $q_2$)
- **Joint 3 (Elbow)**: Revolute joint about horizontal $Z_2$ axis (Flexion pitch $q_3$)

### Link Dimensions
- $L_1 = 0.50\text{ m}$ (Base height)
- $L_2 = 0.40\text{ m}$ (Upper arm length)
- $L_3 = 0.30\text{ m}$ (Forearm length)

### Standard DH Parameters

| Link $i$ | Joint Angle $\theta_i$ | Offset $d_i$ | Length $a_i$ | Twist $\alpha_i$ | Range |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | $q_1$ | $0.50\text{ m}$ | $0$ | $+90^\circ$ | $[-180^\circ, +180^\circ]$ |
| **2** | $q_2$ | $0$ | $0.40\text{ m}$ | $0^\circ$ | $[-90^\circ, +90^\circ]$ |
| **3** | $q_3$ | $0$ | $0.30\text{ m}$ | $0^\circ$ | $[-135^\circ, +135^\circ]$ |

### Forward Kinematics Equations

$$\begin{cases}
p_x = \cos(q_1) \cdot \left[ L_2 \cos(q_2) + L_3 \cos(q_2 + q_3) \right] \\
p_y = \sin(q_1) \cdot \left[ L_2 \cos(q_2) + L_3 \cos(q_2 + q_3) \right] \\
p_z = L_1 + L_2 \sin(q_2) + L_3 \sin(q_2 + q_3)
\end{cases}$$

---

## Installation

Clone the repository and install dependencies:

```bash
git clone https://github.com/sirin66/ROBOTIC-ARM.git
cd ROBOTIC-ARM
pip install -r requirements.txt
```

---

## Terminal Commands to Run Every File

### 1. Launch the Real-Time 3D Web Application
Starts the Flask server hosting the interactive Three.js 3D dashboard:
```bash
python app.py
```
Open **`http://127.0.0.1:5000`** in your browser. Features live dual-controls (sliders + number inputs), direct 3D link manipulation, and instant $X, Y, Z$ forward kinematics telemetry.

---

### 2. Run Forward Kinematics Verification & Poses
Computes and verifies analytical vs. toolbox kinematics for multiple test poses:
```bash
python main.py
```

---

### 3. Run Standalone Clean Kinematics & Plot
Executes the self-contained script and outputs matrix, positions, and saves `robot_model.png`:
```bash
python robot_arm_3dof.py
```

---

### 4. Render High-Resolution 3D CAD Model
Generates a volumetric CAD model with actuators, links, and parallel gripper, saving to `cad_robot_render.png`:
```bash
python run_cad.py
```

---

### 5. Open Interactive Rotatable 3D CAD Window
Opens an interactive Matplotlib 3D window where you can orbit, zoom, and inspect the CAD arm:
```bash
python run_cad.py --show
```

---

### 6. Render CAD Model with Custom Joint Angles
Pass any joint angles in degrees ($q_1, q_2, q_3$):
```bash
python run_cad.py --q 45 30 -20
python run_cad.py --show --q 45 30 -20
```

---

## Open All Project Files in Editor from Terminal

To open all project files simultaneously in VS Code from terminal:
```bash
code .
```
Or open specific script files:
```bash
code app.py templates/index.html robot_arm_3dof.py cad_model.py run_cad.py main.py
```
