import os
import numpy as np
from flask import Flask, render_template, request, jsonify
import roboticstoolbox as rtb

app = Flask(__name__)

L1 = 0.50
L2 = 0.40
L3 = 0.30

link1 = rtb.RevoluteDH(d=L1, a=0.0, alpha=np.pi/2)
link2 = rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0)
link3 = rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0)
robot = rtb.DHRobot([link1, link2, link3], name="3DOF_Robot")

def calculate_fkine(q1_deg, q2_deg, q3_deg):
    q1 = np.deg2rad(float(q1_deg))
    q2 = np.deg2rad(float(q2_deg))
    q3 = np.deg2rad(float(q3_deg))

    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c3, s3 = np.cos(q3), np.sin(q3)
    c23 = np.cos(q2 + q3)
    s23 = np.sin(q2 + q3)

    A1 = np.array([
        [c1, 0,  s1, 0],
        [s1, 0, -c1, 0],
        [0,  1,  0,  L1],
        [0,  0,  0,  1]
    ])
    A2 = np.array([
        [c2, -s2, 0, L2 * c2],
        [s2,  c2, 0, L2 * s2],
        [0,   0,  1, 0],
        [0,   0,  0, 1]
    ])
    A3 = np.array([
        [c3, -s3, 0, L3 * c3],
        [s3,  c3, 0, L3 * s3],
        [0,   0,  1, 0],
        [0,   0,  0, 1]
    ])

    T = A1 @ A2 @ A3
    
    px = c1 * (L2 * c2 + L3 * c23)
    py = s1 * (L2 * c2 + L3 * c23)
    pz = L1 + L2 * s2 + L3 * s23
    reach = np.sqrt(px**2 + py**2 + pz**2)

    T_toolbox = robot.fkine([q1, q2, q3])
    rpy = T_toolbox.rpy(unit="deg").tolist()

    return {
        "x": round(float(px), 4),
        "y": round(float(py), 4),
        "z": round(float(pz), 4),
        "reach": round(float(reach), 4),
        "matrix": [[round(float(v), 4) for v in row] for row in T],
        "rpy": [round(float(v), 2) for v in rpy],
        "q": [float(q1_deg), float(q2_deg), float(q3_deg)],
        "l1": L1,
        "l2": L2,
        "l3": L3
    }

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/fkine", methods=["GET", "POST"])
def api_fkine():
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        q1 = data.get("q1", 0.0)
        q2 = data.get("q2", 0.0)
        q3 = data.get("q3", 0.0)
    else:
        q1 = request.args.get("q1", 0.0, type=float)
        q2 = request.args.get("q2", 0.0, type=float)
        q3 = request.args.get("q3", 0.0, type=float)

    result = calculate_fkine(q1, q2, q3)
    return jsonify(result)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
