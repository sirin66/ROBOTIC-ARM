from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether
import os

def generate_pdf():
    pdf_filename = "Micro_Project_3DOF_Robotic_Manipulator.pdf"
    doc = SimpleDocTemplate(
        pdf_filename,
        pagesize=letter,
        rightMargin=54,
        leftMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=18,
        leading=22,
        textColor=colors.HexColor('#1A365D'),
        alignment=1,
        spaceAfter=4
    )

    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Oblique',
        fontSize=11,
        leading=14,
        textColor=colors.HexColor('#4A5568'),
        alignment=1,
        spaceAfter=14
    )

    h1_style = ParagraphStyle(
        'SecHeading',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=12,
        leading=16,
        textColor=colors.HexColor('#2B6CB0'),
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'BodyDark',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=10,
        leading=14,
        textColor=colors.HexColor('#2D3748'),
        spaceAfter=6
    )

    step_title_style = ParagraphStyle(
        'StepTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9.5,
        leading=13,
        textColor=colors.HexColor('#2B6CB0')
    )

    step_desc_style = ParagraphStyle(
        'StepDesc',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9,
        leading=12,
        textColor=colors.HexColor('#4A5568')
    )

    code_style = ParagraphStyle(
        'CodeStyle',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8,
        leading=10.5,
        textColor=colors.HexColor('#1A202C')
    )

    console_style = ParagraphStyle(
        'ConsoleStyle',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=7.8,
        leading=10,
        textColor=colors.HexColor('#E2E8F0')
    )

    caption_style = ParagraphStyle(
        'CaptionStyle',
        parent=styles['Normal'],
        fontName='Helvetica-Oblique',
        fontSize=8.5,
        leading=11,
        textColor=colors.HexColor('#64748B'),
        alignment=1,
        spaceBefore=4,
        spaceAfter=10
    )

    story = []

    # Title & Subtitle
    story.append(Paragraph("Micro Project: 3-DOF Robotic Manipulator", title_style))
    story.append(Paragraph("Kinematic Modeling, Forward Kinematics Formulation & 3D Simulation", subtitle_style))

    # Meta Info Table
    meta_data = [
        [Paragraph("<b>Project Title:</b>", body_style), Paragraph("3-DOF Revolute Robotic Manipulator (RRR)", body_style)],
        [Paragraph("<b>Student Name:</b>", body_style), Paragraph("SIRIN J DEVASSIA", body_style)],
        [Paragraph("<b>Roll No:</b>", body_style), Paragraph("28", body_style)],
        [Paragraph("<b>Department:</b>", body_style), Paragraph("Robotics and Artificial Intelligence (RAI)", body_style)],
        [Paragraph("<b>Academic Year:</b>", body_style), Paragraph("2025 - 2026", body_style)]
    ]

    t_meta = Table(meta_data, colWidths=[130, 374])
    t_meta.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#F8FAFC')),
        ('BOX', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('INNERGRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#E2E8F0')),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(t_meta)
    story.append(Spacer(1, 12))

    # 1. Algorithm
    story.append(Paragraph("1. Algorithm (Pseudo-code)", h1_style))
    story.append(Paragraph("The following step-by-step logic defines the geometric initialization, kinematic transformation pipeline, and 3D simulation of the 3-DOF articulated manipulator:", body_style))

    algo_steps = [
        ("Step 1: Link Parameter Definition", "Define physical link dimensions: Base height L1, Upper arm length L2, and Forearm length L3 in meters."),
        ("Step 2: DH Parameter Formulation", "Assign Standard Denavit-Hartenberg parameters for each link:<br/>• Link 1: d = L1, a = 0, alpha = +pi/2<br/>• Link 2: d = 0, a = L2, alpha = 0<br/>• Link 3: d = 0, a = L3, alpha = 0"),
        ("Step 3: Robot Assembly", "Instantiate RevoluteDH joint objects and assemble the serial-link DHRobot model."),
        ("Step 4: Input Joint Angles", "Acquire joint variables q1, q2, and q3 in degrees and convert them into radians."),
        ("Step 5: Homogeneous Transformation Calculation", "Compute individual link transformation matrices A1, A2, and A3, and evaluate the composite end-effector transformation matrix:<br/>&nbsp;&nbsp;&nbsp;&nbsp;T_0_3 = A1 * A2 * A3"),
        ("Step 6: End-Effector Position Extraction", "Extract translational coordinates (X, Y, Z) from fourth column of T_0_3:<br/>&nbsp;&nbsp;&nbsp;&nbsp;X = cos(q1) * [L2 * cos(q2) + L3 * cos(q2 + q3)]<br/>&nbsp;&nbsp;&nbsp;&nbsp;Y = sin(q1) * [L2 * cos(q2) + L3 * cos(q2 + q3)]<br/>&nbsp;&nbsp;&nbsp;&nbsp;Z = L1 + L2 * sin(q2) + L3 * sin(q2 + q3)"),
        ("Step 7: 3D Kinematic Plotting", "Pass joint angles to the robotics visualization environment to render links, joint axes, and coordinate frames in 3D Euclidean space, and save the plot.")
    ]

    for title, desc in algo_steps:
        step_table = Table([[Paragraph(title, step_title_style)], [Paragraph(desc, step_desc_style)]], colWidths=[504])
        step_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#FFFFFF')),
            ('LEFTPADDING', (0, 0), (-1, -1), 12),
            ('TOPPADDING', (0, 0), (-1, -1), 2),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 2),
        ]))
        story.append(step_table)

    story.append(Spacer(1, 10))

    # 2. Program
    story.append(Paragraph("2. Program", h1_style))
    story.append(Paragraph("Core Python implementation utilizing Peter Corke's Robotics Toolbox for forward kinematics calculation and 3D simulation:", body_style))

    code_text = """import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

class Manipulator3DOF:
    def __init__(self, l1=0.5, l2=0.4, l3=0.3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        self.link1 = rtb.RevoluteDH(d=self.l1, a=0.0, alpha=np.pi/2)
        self.link2 = rtb.RevoluteDH(d=0.0, a=self.l2, alpha=0.0)
        self.link3 = rtb.RevoluteDH(d=0.0, a=self.l3, alpha=0.0)
        
        self.robot = rtb.DHRobot([self.link1, self.link2, self.link3], name="3DOF_Manipulator")

    def forward_kinematics(self, q):
        return self.robot.fkine(q)

    def plot_arm(self, q, save_path=None):
        self.robot.plot(q, backend="pyplot", block=False)
        if save_path:
            plt.savefig(save_path, dpi=300)
        plt.close()

if __name__ == "__main__":
    arm = Manipulator3DOF(l1=0.5, l2=0.4, l3=0.3)
    
    test_angles = [
        [0.0, 0.0, 0.0],
        [30.0, 45.0, 30.0],
        [-45.0, 60.0, -30.0]
    ]
    
    for i, angles in enumerate(test_angles, 1):
        q_rad = np.deg2rad(angles)
        T = arm.forward_kinematics(q_rad)
        pos = T.t
        
        print(f"Test Configuration {i}:")
        print(f"Joint Angles (deg): q1 = {angles[0]}, q2 = {angles[1]}, q3 = {angles[2]}")
        print("Homogeneous Transformation Matrix:")
        print(np.round(T.A, 4))
        print(f"End-Effector Coordinates (m): X = {pos[0]:.4f}, Y = {pos[1]:.4f}, Z = {pos[2]:.4f}\\n")
    
    arm.plot_arm(np.deg2rad([30.0, 45.0, 30.0]), save_path="manipulator_3d_pose.png")"""

    code_html = code_text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\n", "<br/>").replace(" ", "&nbsp;")
    t_code = Table([[Paragraph(code_html, code_style)]], colWidths=[504])
    t_code.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#F8FAFC')),
        ('BOX', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('LINELEFT', (0, 0), (0, -1), 2.5, colors.HexColor('#3182CE')),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(t_code)
    story.append(Spacer(1, 10))

    # 3. Result
    story.append(Paragraph("3. Result", h1_style))
    story.append(Paragraph("The script was executed for three distinct joint angle test configurations to evaluate the spatial displacement of the end-effector:", body_style))

    console_text = """Test Configuration 1:
Joint Angles (deg): q1 = 0.0, q2 = 0.0, q3 = 0.0
Homogeneous Transformation Matrix:
[[ 1.  0.  0.  0.7]
 [ 0.  0. -1.  0. ]
 [ 0.  1.  0.  0.5]
 [ 0.  0.  0.  1. ]]
End-Effector Coordinates (m): X = 0.7000, Y = 0.0000, Z = 0.5000

Test Configuration 2:
Joint Angles (deg): q1 = 30.0, q2 = 45.0, q3 = 30.0
Homogeneous Transformation Matrix:
[[ 0.2241 -0.8365  0.5     0.3122]
 [ 0.1294 -0.483  -0.866   0.1802]
 [ 0.9659  0.2588  0.      1.0726]
 [ 0.      0.      0.      1.    ]]
End-Effector Coordinates (m): X = 0.3122, Y = 0.1802, Z = 1.0726

Test Configuration 3:
Joint Angles (deg): q1 = -45.0, q2 = 60.0, q3 = -30.0
Homogeneous Transformation Matrix:
[[ 0.6124 -0.3536 -0.7071  0.3251]
 [-0.6124  0.3536 -0.7071 -0.3251]
 [ 0.5     0.866   0.      0.9964]
 [ 0.      0.      0.      1.    ]]
End-Effector Coordinates (m): X = 0.3251, Y = -0.3251, Z = 0.9964"""

    console_html = console_text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\n", "<br/>").replace(" ", "&nbsp;")
    t_console = Table([[Paragraph(console_html, console_style)]], colWidths=[504])
    t_console.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#0F172A')),
        ('BOX', (0, 0), (-1, -1), 0.5, colors.HexColor('#1E293B')),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(t_console)
    story.append(Spacer(1, 10))

    # Summary Table
    tab_data = [
        [Paragraph("<b>Config</b>", body_style), Paragraph("<b>Joint Angles (q1, q2, q3)</b>", body_style), Paragraph("<b>X (m)</b>", body_style), Paragraph("<b>Y (m)</b>", body_style), Paragraph("<b>Z (m)</b>", body_style)],
        [Paragraph("1 (Home)", body_style), Paragraph("[0.0°, 0.0°, 0.0°]", body_style), Paragraph("0.7000", body_style), Paragraph("0.0000", body_style), Paragraph("0.5000", body_style)],
        [Paragraph("2 (Reach)", body_style), Paragraph("[30.0°, 45.0°, 30.0°]", body_style), Paragraph("0.3122", body_style), Paragraph("0.1802", body_style), Paragraph("1.0726", body_style)],
        [Paragraph("3 (Pick)", body_style), Paragraph("[-45.0°, 60.0°, -30.0°]", body_style), Paragraph("0.3251", body_style), Paragraph("-0.3251", body_style), Paragraph("0.9964", body_style)],
    ]
    t_summary = Table(tab_data, colWidths=[80, 184, 80, 80, 80])
    t_summary.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#E2E8F0')),
        ('BOX', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('INNERGRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#E2E8F0')),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]))
    story.append(t_summary)
    story.append(Spacer(1, 12))

    # Embedded 3D Plot
    img_file = "manipulator_3d_pose.png"
    if os.path.exists(img_file):
        story.append(Image(img_file, width=320, height=240))
        story.append(Paragraph("Figure 1: Generated 3D Robotic Manipulator Model at Joint Angles q = [30.0°, 45.0°, 30.0°]", caption_style))

    doc.build(story)
    print(f"[OK] Successfully generated PDF: {pdf_filename}")

if __name__ == "__main__":
    generate_pdf()
