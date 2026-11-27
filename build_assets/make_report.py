import os
import docx
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_ALIGN_VERTICAL
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import qn, nsdecls

doc = Document()

# Page setup: A4 matching 23ECE436_End_Sem_project_Report_format.docx
section = doc.sections[0]
section.page_width = Inches(8.27)
section.page_height = Inches(11.69)
section.left_margin = Inches(1.0)
section.right_margin = Inches(1.0)
section.top_margin = Inches(0.85)
section.bottom_margin = Inches(0.85)

# Footer with page numbers
footer = section.footer
fp = footer.paragraphs[0]
fp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
fr1 = fp.add_run("23ECE436 – Embedded Systems for Robotics  |  ASE Bengaluru   |   Page ")
fr1.font.name = "Calibri"
fr1.font.size = Pt(9)
fr1.font.color.rgb = RGBColor(0x64, 0x74, 0x8B)
fldSimple = OxmlElement("w:fldSimple")
fldSimple.set(qn("w:instr"), "PAGE")
fp._p.append(fldSimple)

# Base Normal style
normal_style = doc.styles["Normal"]
normal_style.font.name = "Calibri"
normal_style.font.size = Pt(11)
normal_style.font.color.rgb = RGBColor(0x1E, 0x29, 0x3B)
normal_style.paragraph_format.line_spacing = 1.18
normal_style.paragraph_format.space_after = Pt(6)

AMRITA_MAROON = RGBColor(0x8B, 0x15, 0x38)
DARK_NAVY = RGBColor(0x0F, 0x17, 0x2A)
ACCENT_BLUE = RGBColor(0x02, 0x84, 0xC7)
MUTED = RGBColor(0x47, 0x55, 0x69)


def set_cell_shading(cell, hex_color):
    shading = parse_xml(f'<w:shd {nsdecls("w")} w:fill="{hex_color}"/>')
    cell._tc.get_or_add_tcPr().append(shading)


def set_cell_margins(cell, top=80, bottom=80, left=120, right=120):
    tcPr = cell._tc.get_or_add_tcPr()
    tcMar = OxmlElement("w:tcMar")
    for m, val in [("top", top), ("bottom", bottom), ("left", left), ("right", right)]:
        node = OxmlElement(f"w:{m}")
        node.set(qn("w:w"), str(val))
        node.set(qn("w:type"), "dxa")
        tcMar.append(node)
    tcPr.append(tcMar)


def add_heading_1(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.keep_with_next = True
    r = p.add_run(text)
    r.font.name = "Calibri"
    r.font.size = Pt(15)
    r.font.bold = True
    r.font.color.rgb = AMRITA_MAROON
    # Bottom border on Heading 1
    pPr = p._p.get_or_add_pPr()
    pBdr = parse_xml(f'<w:pBdr {nsdecls("w")}><w:bottom w:val="single" w:sz="6" w:space="3" w:color="CBD5E1"/></w:pBdr>')
    pPr.append(pBdr)
    return p


def add_heading_2(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.keep_with_next = True
    r = p.add_run(text)
    r.font.name = "Calibri"
    r.font.size = Pt(12.5)
    r.font.bold = True
    r.font.color.rgb = DARK_NAVY
    return p


def add_body(text, bold_prefix=None, justify=True):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY if justify else WD_ALIGN_PARAGRAPH.LEFT
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.line_spacing = 1.18
    if bold_prefix:
        rb = p.add_run(bold_prefix)
        rb.bold = True
        rb.font.color.rgb = DARK_NAVY
    r = p.add_run(text)
    return p


def add_bullet(lead, text):
    p = doc.add_paragraph(style="List Bullet")
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.line_spacing = 1.15
    if lead:
        r1 = p.add_run(lead + " ")
        r1.bold = True
        r1.font.color.rgb = DARK_NAVY
    p.add_run(text)
    return p


def add_equation(eq_text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(6)
    r = p.add_run(eq_text)
    r.font.name = "Cambria Math"
    r.font.size = Pt(11)
    r.font.italic = True
    r.font.color.rgb = DARK_NAVY
    return p


def add_figure(img_path, caption, width_in=6.0):
    if not os.path.exists(img_path):
        return
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.keep_with_next = True
    p.add_run().add_picture(img_path, width=Inches(width_in))

    cp = doc.add_paragraph()
    cp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cp.paragraph_format.space_after = Pt(8)
    cr = cp.add_run(caption)
    cr.font.size = Pt(9.5)
    cr.font.italic = True
    cr.font.color.rgb = MUTED


def add_styled_table(headers, rows, col_widths=None):
    tbl = doc.add_table(rows=len(rows) + 1, cols=len(headers))
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    tbl.style = "Table Grid"
    for c_i, h in enumerate(headers):
        cell = tbl.rows[0].cells[c_i]
        set_cell_shading(cell, "0F172A")
        set_cell_margins(cell, 90, 90, 110, 110)
        cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
        p = cell.paragraphs[0]
        p.paragraph_format.space_after = Pt(0)
        r = p.add_run(h)
        r.bold = True
        r.font.size = Pt(9.5)
        r.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)

    for r_i, row in enumerate(rows):
        bg = "FFFFFF" if r_i % 2 == 0 else "F8FAFC"
        for c_i, val in enumerate(row):
            cell = tbl.rows[r_i + 1].cells[c_i]
            set_cell_shading(cell, bg)
            set_cell_margins(cell, 70, 70, 110, 110)
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            p = cell.paragraphs[0]
            p.paragraph_format.space_after = Pt(0)
            p.paragraph_format.line_spacing = 1.12
            r = p.add_run(val)
            r.font.size = Pt(9.5)
            if c_i == 0:
                r.bold = True
                r.font.color.rgb = DARK_NAVY
    if col_widths:
        for row in tbl.rows:
            for idx, w in enumerate(col_widths):
                row.cells[idx].width = Inches(w)
    sp = doc.add_paragraph()
    sp.paragraph_format.space_after = Pt(4)
    return tbl


# ===========================================================================
# COVER PAGE (Matching 23ECE436_End_Sem_project_Report_format.docx)
# ===========================================================================
if os.path.exists("build_assets/amrita_logo.png"):
    lp = doc.add_paragraph()
    lp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    lp.paragraph_format.space_after = Pt(8)
    lp.add_run().add_picture("build_assets/amrita_logo.png", width=Inches(3.6))

for line, sz, bold, col, sa in [
    ("Amrita Vishwa Vidyapeetham", 15, True, AMRITA_MAROON, 2),
    ("Amrita School of Engineering, Bengaluru", 13, True, DARK_NAVY, 10),
    ("B. Tech in Electronics and Communication Engineering", 12, True, DARK_NAVY, 2),
    ("VII Semester", 12, False, MUTED, 14),
    ("23ECE436 – Embedded Systems for Robotics", 13.5, True, AMRITA_MAROON, 2),
    ("End Semester Project Report", 13, True, DARK_NAVY, 22),
    ("Vision and LiDAR-Guided Owner-Locked\nHuman-Following Mobile Robot", 18, True, DARK_NAVY, 22),
    ("Submitted by", 12, True, MUTED, 6),
    ("BL.EN.U4ECE23213 – Gade Srishanth\nBL.EN.U4ECE23208 – D V S Sainath Reddy\nBL.EN.U4ECE23207 – Rushikesh D", 12, True, DARK_NAVY, 24),
]:
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(sa)
    r = p.add_run(line)
    r.font.size = Pt(sz)
    r.bold = bold
    r.font.color.rgb = col

eval_tbl = doc.add_table(rows=1, cols=2)
eval_tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
eval_tbl.style = "Table Grid"
set_cell_shading(eval_tbl.rows[0].cells[0], "F8FAFC")
set_cell_shading(eval_tbl.rows[0].cells[1], "F8FAFC")
set_cell_margins(eval_tbl.rows[0].cells[0], 100, 100, 140, 140)
set_cell_margins(eval_tbl.rows[0].cells[1], 100, 100, 140, 140)
eval_tbl.rows[0].cells[0].width = Inches(1.8)
eval_tbl.rows[0].cells[1].width = Inches(3.2)
p0 = eval_tbl.rows[0].cells[0].paragraphs[0]
p0.paragraph_format.space_after = Pt(0)
r0 = p0.add_run("Evaluated by:")
r0.bold = True
r0.font.size = Pt(11)
p1 = eval_tbl.rows[0].cells[1].paragraphs[0]
p1.paragraph_format.space_after = Pt(0)
r1 = p1.add_run("Dr. Sreeja Kochuvila\nAssociate Professor\nDept. of ECE")
r1.font.size = Pt(11)

for line, sb, sa, bold in [
    ("Amrita School of Engineering, Bengaluru - 560035", 26, 2, True),
    ("November 2026", 0, 0, False),
]:
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(sb)
    p.paragraph_format.space_after = Pt(sa)
    r = p.add_run(line)
    r.font.size = Pt(11.5)
    r.bold = bold
    r.font.color.rgb = DARK_NAVY

doc.add_page_break()

# ===========================================================================
# TITLE + ABSTRACT
# ===========================================================================
tp = doc.add_paragraph()
tp.alignment = WD_ALIGN_PARAGRAPH.CENTER
tp.paragraph_format.space_after = Pt(10)
tr = tp.add_run("Vision and LiDAR-Guided Owner-Locked Human-Following Mobile Robot")
tr.font.size = Pt(16)
tr.bold = True
tr.font.color.rgb = DARK_NAVY

add_heading_2("Abstract")
add_body(
    "This report presents the design, implementation, and experimental validation of an autonomous, owner-locked "
    "human-following mobile robot built on the TurtleBot3 Burger platform. Inspired by the FollowMe framework [1], "
    "the robot enrolls a single designated operator using natural upper-body gestures, distinguishes that owner from "
    "bystanders and inanimate clutter in real time, maintains a smooth adaptive following distance, and recovers "
    "autonomously when the owner steps behind obstacles or leaves the camera field of view. To overcome the compute "
    "limitations of the onboard Raspberry Pi 4, all computer vision, appearance re-identification, sensor fusion, "
    "mapping, and path planning run on a laptop edge-compute station at roughly 39 FPS (CPU only), while the Raspberry Pi "
    "acts strictly as a thin ROS 2 Humble sensor-and-actuator bridge protected by a 0.35 s hardware watchdog. "
    "Camera-derived monocular distance and bearing from MediaPipe BlazePose landmarks are fused with 360-degree "
    "LDS-02 LiDAR person clusters inside a 4-state polar Extended Kalman Filter (EKF), enabling online scale and "
    "bearing self-calibration as well as close-range leg tracking when the owner stands inside the camera blind spot. "
    "When occluded behind a wall or obstacle, the robot first drives around the obstruction via an A* planner on a "
    "live scan-matched occupancy grid before initiating a LiDAR-guarded rotational search. The complete system is "
    "validated across 161 automated unit and simulation tests (100% pass rate) and live laboratory trials on the "
    "physical TurtleBot3 Burger."
)

# ===========================================================================
# 1. INTRODUCTION
# ===========================================================================
add_heading_1("1. Introduction")
add_body(
    "Human-following mobile robots have become central to practical service robotics in hospitals, logistics warehouses, "
    "airports, and domestic assistance environments. Rather than requiring continuous joystick teleoperation or fixed "
    "magnetic tracks, a companion robot must seamlessly perceive a specific human operator, maintain a comfortable and "
    "safe separation distance, navigate around static furniture or unexpected obstacles, and yield immediately when "
    "commanded. While simple ultrasonic or color-blob trackers can follow a single isolated target in an empty corridor, "
    "real indoor spaces introduce multiple challenges: bystanders walking across the robot's path, lighting variations "
    "across rooms, chairs or coat racks resembling human silhouettes, temporary occlusions behind walls, and the "
    "geometric blind spot of a low-mounted robot camera when a person stands close."
)
add_body(
    "Building such capability onto a compact differential-drive platform like the TurtleBot3 Burger poses an additional "
    "embedded systems constraint. Although the onboard Raspberry Pi 4 Model B handles ROS 2 middleware, serial motor "
    "communication with the OpenCR 1.0 controller, and LiDAR streaming reliably, running 33-point neural pose inference "
    "alongside multi-resolution scan matching and A* path planning directly on the Pi saturates the ARM Cortex-A72 CPU "
    "and introduces control jitter. A hung vision thread on the robot itself can also delay emergency stops. Separating "
    "intelligence from actuation solves both problems: the laptop executes the full perception-to-control stack over a "
    "custom low-latency UDP protocol, while the Pi enforces deterministic velocity clamping and a tight 0.35 s deadman "
    "watchdog right above the wheel servos."
)
add_body(
    "In our implementation, the robot begins in an idle standby state until an operator performs an explicit 'FOLLOW_ME' "
    "gesture (raising one hand above head level) or clicks Enroll on the web dashboard. An immutable protected appearance "
    "profile is captured in Hue–Saturation color space alongside anatomical proportions, ensuring the robot never swaps "
    "its target to a bystander or locks onto furniture. As the operator walks, an EKF fuses monocular pose estimates with "
    "LiDAR returns to drive a PID + feed-forward velocity controller. When the operator stops, the robot halts smoothly "
    "within a hysteresis band and waits indefinitely without hunting. When the operator disappears behind an obstacle, "
    "map-based recall routes the robot around the obstruction before any rotational search begins."
)

# ===========================================================================
# 2. OBJECTIVES
# ===========================================================================
add_heading_1("2. Objectives")
add_body("The primary engineering objectives of this project are:")
add_bullet("Distributed Edge-Compute Architecture:", "Partition the system cleanly so the laptop executes real-time AI perception, sensor fusion, mapping, and control while the Raspberry Pi operates strictly as a thin ROS 2 Humble camera/LiDAR streamer and motor bridge.")
add_bullet("Anatomical Human Verification:", "Filter out chairs, bags, and background clutter by validating 33-point MediaPipe BlazePose skeletal geometry before any candidate enters tracking.")
add_bullet("Gesture-Driven Human–Robot Interaction (HRI):", "Support intuitive, debounced upper-body gestures (FOLLOW_ME, WAIT, RELEASE, STOP_HALT) executed exclusively by the enrolled owner (with STOP_HALT accepted from anyone for safety).")
add_bullet("Owner-Locked Multi-Template Re-Identification:", "Maintain a protected baseline appearance template and up to 8 adaptive side/back views in V-channel-invariant HSV space, halting safely under visual ambiguity.")
add_bullet("Adaptive Following Distance & Anti-Chatter Control:", "Regulate following distance dynamically around a 1.20 m set-point using speed headway and confidence margins, enforcing a 25 cm follow-and-wait hysteresis band.")
add_bullet("Multi-Layer Safety & Hardware Watchdogs:", "Guarantee collision-free operation through LiDAR reactive avoidance, a 0.20 m forward hard stop, dual network watchdogs (0.35 s Pi / 0.60 s Laptop), and an emergency stop latch.")

add_heading_2("Novelty of the Proposed Work")
add_bullet("Novelty 1 — Online Camera–LiDAR Co-Mapping with Self-Calibration:", "Every 360-degree LiDAR return is projected into the camera image frame ('laser curtain'), allowing direct per-person LiDAR cluster association inside the EKF. While both sensors observe the owner, the laptop continuously self-calibrates monocular scale (kv) and bearing bias (b_cam) against the 5 cm-accurate LiDAR, and seamlessly hands tracking over to LiDAR clusters when the owner stands too close for the low-mounted camera to see their upper body.")
add_bullet("Novelty 2 — Map-Memory A* Recall Prior to Rotational Search:", "Unlike conventional followers that immediately spin in place upon losing the target—often losing sight of an owner who walked around a corner—our system retains the owner's last map-frame coordinate on a live M3RSM/Gauss-Newton SLAM occupancy grid, plans an inflated A* detour around the occluding wall, and invokes its 3-phase LiDAR-guarded rotational search only if the remembered spot is empty.")

# ===========================================================================
# 3. LITERATURE SURVEY
# ===========================================================================
add_heading_1("3. Literature Survey")
add_body(
    "Person-following mobile robots have been studied extensively across indoor service, assistive, and industrial "
    "domains. Early implementations relied on ultrasonic arrays or single-color clothing segmentation, which failed "
    "rapidly in cluttered or crowded rooms. Recent research combines deep pose estimation, visual re-identification "
    "(Re-ID), multi-sensor fusion, and robust recovery planning. Below, we critically review ten foundational and "
    "contemporary works that directly shaped our architecture."
)

add_heading_2("3.1 Gesture-Guided Following and Visual Re-Identification")
add_body(
    "Rollo et al. [1] introduced the FollowMe framework at IEEE ARSO 2023, which serves as the primary reference for "
    "this project. FollowMe couples real-time person detection with visual appearance Re-ID and hand-gesture commands "
    "(Follow, Wait, Release) so a mobile manipulator or base follows only the authorized operator. While FollowMe "
    "demonstrates strong multi-person disambiguation using an RGB-D camera, its reliance on depth cameras increases "
    "cost and leaves the system vulnerable when an owner steps into a camera blind spot at very close range or walks "
    "around a sharp corner. Our work adopts FollowMe's gesture-and-Re-ID paradigm on a low-cost monocular USB webcam "
    "and augments it with 360-degree 2D LiDAR fusion and map-memory recall."
)
add_body(
    "Gupta et al. [4] proposed a vision-based tracking algorithm in IEEE Transactions on Systems, Man, and Cybernetics "
    "that combines appearance cues with adaptive motion prediction for indoor human following. Their experiments highlight "
    "how sudden illumination changes across doorways distort RGB color features and cause target loss. To mitigate "
    "brightness sensitivity, our appearance descriptor discards the Value (V) channel completely and evaluates a normalized "
    "18×16 Hue–Saturation histogram using both Bhattacharyya distance and Pearson correlation. Similarly, Holkar et al. [2] "
    "explored voice-activated human following with computer vision (IJRASET, 2025). Although voice commands offer natural "
    "interaction in quiet rooms, ambient motor whine and multi-speaker background noise in university labs degrade "
    "speech recognition accuracy; silent skeletal gestures recognized by MediaPipe BlazePose [5] provide deterministic, "
    "noise-immune HRI with 4-frame temporal debouncing."
)

add_heading_2("3.2 Target Loss Recovery and Trajectory Estimation")
add_body(
    "Recovering a lost target without latching onto bystanders is a recurring bottleneck in person-following systems. "
    "Do Hoang et al. [3] presented a reliable recovery mechanism at IEEE URAI 2017 specifically for missing-target "
    "scenarios, recording the target's exit direction and commanding controlled search maneuvers rather than erratic "
    "spinning. However, purely reactive search turns cannot reach an owner who steps behind a partition wall. By pairing "
    "Kalman trajectory estimation [11] with a persistent occupancy map, our robot navigates around walls toward the "
    "owner's remembered world coordinate before starting its directional sweep."
)
add_body(
    "For lightweight, real-time human landmark extraction on CPU-constrained hardware, Bazarevsky et al. [5] introduced "
    "BlazePose, achieving real-time 33-keypoint skeletal inference without a discrete GPU. Using BlazePose landmarks "
    "enables explicit anatomical validation—checking shoulder-to-hip vertical ordering and facial alignment—which "
    "eliminates false positives on office chairs and coats that frequently fool bounding-box-only detectors. OpenCV's "
    "optimized histogram and geometric primitives [10] further keep per-frame HSV signature extraction under 0.6 ms."
)

add_heading_2("3.3 LiDAR SLAM, Scan Matching, and Embedded ROS 2 Platforms")
add_body(
    "Accurate local mapping is essential when wheel odometry drifts on smooth laboratory tiles. Olson [6] introduced "
    "Multi-Resolution Real-Time Correlative Scan Matching (M3RSM), constructing coarse-to-fine likelihood pyramids "
    "that resist local minima without heavy particle-filter overhead. Kohlbrecher et al. [7] demonstrated Hector SLAM "
    "at IEEE SSRR 2011, using Gauss-Newton optimization on bilinearly interpolated occupancy grids for sub-cell pose "
    "refinement. Our laptop SLAM engine combines Olson's coarse correlative search with Hector-style Gauss-Newton "
    "refinement while explicitly masking out the tracked owner and moving objects so walking humans never smear the "
    "static costmap."
)
add_body(
    "On the embedded middleware side, Macenski et al. [8] detailed the real-time deterministic execution model of ROS 2 "
    "(Science Robotics, 2022), while ROBOTIS [9] documented the TurtleBot3 Burger hardware stack combining an OpenCR 1.0 "
    "STM32F7 controller and Dynamixel XL430 actuators. Running ROS 2 Humble on the Raspberry Pi 4 purely for `/scan`, "
    "`/odom`, and `/cmd_vel` transport while bridging to our laptop over UDP achieves low-latency closed-loop control "
    "with independent hardware safety watchdogs."
)

add_styled_table(
    ["Ref.", "Author / Work", "Focus Area", "Key Limitation Addressed by Our System"],
    [
        ["[1]", "Rollo et al., IEEE ARSO 2023", "FollowMe: Visual Re-ID + Gestures", "RGB-D blind spot & no wall-detour map recall -> Added 360° LiDAR EKF + A* recall"],
        ["[2]", "Holkar et al., IJRASET 2025", "Voice-activated CV following", "Noise sensitivity -> Silent debounced skeletal gestures + 12 safety layers"],
        ["[3]", "Do Hoang et al., IEEE URAI 2017", "Missing-target recovery", "Reactive turn only -> Map-frame PersonMemory + A* detour before 3-phase rotation"],
        ["[4]", "Gupta et al., IEEE TSMC 2017", "Adaptive vision-based tracking", "RGB lighting drift -> V-channel-invariant 18×16 H–S histograms + Δ<0.08 ambiguity stop"],
        ["[5]", "Bazarevsky et al., 2020", "BlazePose 33-pt pose model", "Single-sensor range error -> 3-anchor monocular range calibrated online by LiDAR"],
        ["[6]", "Olson, IEEE ICRA 2009", "M3RSM correlative scan matching", "Static-world assumption -> Dynamic owner/cluster masking before map integration"],
        ["[7]", "Kohlbrecher et al., IEEE SSRR 2011", "Hector SLAM Gauss-Newton", "Corridor sliding -> Coarse M3RSM motion-prior search feeds sub-cell GN step"],
        ["[8]", "Macenski et al., Sci. Rob. 2022", "ROS 2 Humble architecture", "Heavy onboard inference -> Distributed UDP bridge with 0.35 s Pi watchdog"],
        ["[9]", "ROBOTIS e-Manual, 2024", "TurtleBot3 Burger & OpenCR 1.0", "Default bringup lacks HRI -> Integrated torque retry & hardware HIL simulator"],
        ["[10],[11]", "Bradski (2000); Kalman (1960)", "OpenCV HSV & EKF Foundations", "Linear KF fails on polar radar/cam -> Nonlinear polar EKF with Joseph covariance"],
    ],
    col_widths=[0.55, 1.85, 1.75, 2.12]
)

# ===========================================================================
# 4. METHODOLOGY
# ===========================================================================
add_heading_1("4. Methodology")
add_body(
    "The system operates as a closed-loop distributed pipeline split across two compute nodes. Rather than streaming "
    "uncompressed video over HTTP or running neural models on the Raspberry Pi, the Raspberry Pi streams timestamped "
    "JPEG datagrams and 360-degree LiDAR scans over UDP port 5000, while the laptop processes every frame through pose "
    "detection, appearance Re-ID, EKF sensor fusion, occupancy mapping, finite-state control, and safety gating before "
    "transmitting velocity commands at 30 Hz back to the Pi on UDP port 8888."
)

add_figure("build_assets/fig_architecture.png", "Figure 1: Distributed Laptop-AI and Thin-Pi System Architecture.", width_in=6.25)

add_heading_2("4.1 Camera Pinhole Geometry and Multi-Anchor Range Estimation")
add_body(
    "For an image of width W pixels and configured horizontal field of view θ_HFOV = 57°, the pinhole focal length in "
    "pixels and the owner's bearing β_cam (positive left) from torso centroid column c_x are computed as:"
)
add_equation("f_px = (W / 2) / tan(θ_HFOV / 2)     ;     β_cam = -arctan((c_x - W / 2) / f_px) + b_cam")
add_body(
    "where b_cam ∈ [-0.1, +0.1] rad is an online bearing bias updated slowly whenever LiDAR confirms the owner: "
    "b ← 0.97·b + 0.03·(β_lidar - β_raw). Monocular range d_cam averages whichever of three anatomical anchors are "
    "visible (landmark visibility > 0.35): shoulder span (landmarks 11–12, K = 235 px·m), pelvic span (23–24, "
    "K = 160 px·m), and vertical torso height (11/12 to 23/24, K = 230 px·m), rescaled by W / 640 and clipped to "
    "[0.35, 4.50] m. Averaging anchors prevents error when the person turns sideways (shrinking shoulder width) or leans "
    "(shrinking torso height). When LiDAR returns agree, a vision scale factor k_v ∈ [0.6, 1.6] adapts online "
    "(k_v ← 0.98·k_v + 0.02·r_lidar / d_cam)."
)

add_heading_2("4.2 HSV Appearance Signature and Multi-Template Re-Identification")
add_body(
    "For each anatomically validated human candidate, the upper-torso patch between shoulder and hip landmarks is "
    "converted to HSV and quantized into an L1-normalized 18×16 Hue–Saturation histogram (288 bins), discarding the "
    "brightness channel V. A secondary head histogram and two geometric aspect ratios (a, t) complete the signature. "
    "Color similarity combines Bhattacharyya distance d_B (OpenCV HISTCMP_BHATTACHARYYA) and Pearson correlation ρ:"
)
add_equation("S_color = 0.5·max(0, 1 - d_B) + 0.5·max(0, ρ)     ;     S = 0.70·S_color + 0.18·S_head + 0.12·S_geom")
add_body(
    "If S_color >= 0.45, S is floored at 0.88·S_color so turning away never penalizes strong clothing matches. Upon "
    "enrollment, an immutable protected baseline T_0 is saved alongside up to 8 side/back templates updated only when "
    "the owner is alone in view (EMA α <= 0.08). If the top two candidates both exceed 0.40 and lie within Δ < 0.08, "
    "the state enters TARGET_AMBIGUOUS and the robot halts immediately rather than guessing."
)

add_heading_2("4.3 Extended Kalman Filter and Camera–LiDAR Fusion")
add_body(
    "The owner's state in the robot frame (x forward, y left) is tracked as s = [p_x, p_y, u_x, u_y]^T. During prediction "
    "over interval Δt, robot ego-motion (v, ω) is removed linearly via rotation matrix R(-ωΔt) with discrete white-noise "
    "acceleration process noise (σ_a = 0.9 m/s²). Polar observations z = [r, β]^T are fused using the nonlinear "
    "measurement Jacobian H, Joseph-form covariance update P = (I - KH)P(I - KH)^T + KRK^T, and χ² gating (normalized "
    "innovation squared <= 11.3). Camera noise is modeled as σ_r = 0.12·d, σ_β = 1.5°, whereas matched LiDAR person "
    "clusters receive σ_r = 0.05 m, σ_β = 1.0°. During temporary loss, velocity decays as exp(-Δt / 2)."
)

add_heading_2("4.4 PID + Feed-Forward Control and Follow-and-Wait Hysteresis")
add_body(
    "The controller tracks the EKF state 0.35 s ahead with range error e_d = r - d* and bearing error e_β = β:"
)
add_equation("ω = Kpβ·e_β + Kiβ·∫e_β dt + Kdβ·β̇_EKF     ;     v = [Kpd·e_d + Kid·∫e_d dt + Kdd·ṙ_EKF + Kff·max(0, u_los)·φ(r)]·cos²(β)")
add_body(
    "Using EKF derivative rates avoids amplifying raw sensor noise, while cos²(β) prioritizes turning before driving "
    "forward when off-axis. The dynamic target distance expands smoothly with owner speed ||u|| and uncertainty (1 - c): "
    "d* = d_set + min(0.60, 0.50·||u|| + 0.35·(1 - c)). With base d* = 1.20 m, the robot enters WAIT_FOR_OWNER_MOVEMENT "
    "at d_stop = 1.10 m and resumes FOLLOWING only after r >= 1.35 m for 3 consecutive frames—eliminating stop-go chatter."
)

add_figure("build_assets/fig_hysteresis.png", "Figure 2: Follow-and-Wait Velocity Profile and 25 cm Anti-Chatter Hysteresis Band.", width_in=5.8)

add_heading_2("4.5 16-State Finite State Machine, SLAM, and A* Recall")
add_body(
    "The behavioral state machine (Figure 3) separates 12 normal tracking/recovery states from 4 independent safety/mode "
    "overrides (SAFETY_STOP, SENSOR_FAULT, MANUAL_CONTROL, TARGET_AMBIGUOUS). On the mapping thread (4 Hz), a 12×12 m "
    "log-odds grid (0.05 m cells) is aligned via 2-level M3RSM correlative search followed by Gauss-Newton refinement. "
    "Obstacles are inflated by 0.22 m (0.105 m Burger radius + safety margin) while keeping a 1-cell hard obstacle core "
    "around real wall returns inside the robot's local clearing bubble. On target loss, PersonMemory guides A* around "
    "walls to the last-seen coordinate for up to 25 s before invoking the 3-phase 360-degree search."
)

add_figure("build_assets/dashboard_fsm.png", "Figure 3: Complete 16-State Behavioral Automaton and Independent Safety Overrides (Live Dashboard State Machine Visualizer).", width_in=4.8)

# ===========================================================================
# 5. IMPLEMENTATION AND RESULTS
# ===========================================================================
add_heading_1("5. Implementation and Results")

add_heading_2("5.1 Hardware Specifications and Software Architecture")
add_body(
    "The physical robot is a ROBOTIS TurtleBot3 Burger equipped with an OpenCR 1.0 board, two Dynamixel XL430-W250-T "
    "servos, a 360-degree LDS-02 LiDAR, a 640×480 USB UVC webcam, and a Raspberry Pi 4 running Ubuntu 22.04 with ROS 2 "
    "Humble (ROS_DOMAIN_ID=30). Table 1 summarizes the hardware and compute stack, while Table 2 lists the implemented "
    "software modules across laptop/, pi/, hil/, and tests/."
)

add_styled_table(
    ["Component", "Model / Specification", "System Role"],
    [
        ["Mobile Base & Actuators", "TurtleBot3 Burger; 2× Dynamixel XL430-W250-T", "Differential drive capped at 0.22 m/s, 2.0 rad/s"],
        ["Embedded Controller", "OpenCR 1.0 (STM32F746, 216 MHz, IMU)", "Wheel PID loops, /odom, battery monitor via /dev/ttyACM0"],
        ["Onboard Companion SBC", "Raspberry Pi 4 Model B (BCM2711, Wi-Fi ac)", "Thin UDP node (ROS 2 Humble); 0.35 s motor watchdog"],
        ["360° Range Sensor", "ROBOTIS LDS-02 (0.16–8.0 m, 360×1° bins)", "EKF cluster fusion, reactive avoidance, M3RSM SLAM"],
        ["Monocular Camera", "USB UVC Webcam (640×480 @ 25 FPS, 57° HFOV)", "MediaPipe pose, HSV Re-ID, gestures, laser-curtain overlay"],
        ["Power Source", "11.1 V 3S LiPo, 1800 mAh (cutoff < 11.0 V)", "Powers OpenCR and Raspberry Pi"],
        ["Laptop Edge Station", "AMD Ryzen 7 6800HS, 8C/16T (CPU only)", "39 FPS vision, EKF, SLAM, A* planner, Flask dashboard (:8000)"],
    ],
    col_widths=[1.6, 2.35, 2.32]
)

add_styled_table(
    ["Module Path", "Core Responsibility"],
    [
        ["laptop/cv_pipeline.py", "Per-frame orchestrator: BlazePose detection, anatomical filter, Re-ID, EKF, 16-state FSM, HUD"],
        ["laptop/appearance_model.py", "18×16 H–S histograms, Bhattacharyya + Pearson scoring, protected T0 + 8-view bank, ambiguity gate"],
        ["laptop/gesture_recognizer.py", "4-frame debounced recognition of STOP_HALT, RELEASE, FOLLOW_ME, and WAIT gestures"],
        ["laptop/ekf_person.py", "4-state polar EKF with ego-motion removal, χ² gating (<11.3), and velocity coasting"],
        ["laptop/follow_controller.py", "Range & bearing PIDs with anti-windup, EKF rate derivatives, and cos²(β) steering"],
        ["laptop/lidar.py & navigation.py", "Scan clustering, M3RSM + GN SLAM occupancy grid (0.05 m), 0.22 m inflated A* & PersonMemory"],
        ["laptop/robot_controller.py", "Slew-rate limiter (1.6 m/s², 8 rad/s²), 0.20 m LiDAR hard stop, 0.60 s watchdog, 30 Hz UDP TX"],
        ["laptop/safety_monitor.py & app.py", "Proximity/gesture/video-loss interlocks; Flask MJPEG + SSE dashboard & runtime tuner"],
        ["pi/pi_turtlebot_node.py & tb_proto.py", "UDP JPEG chunk sender (:5000), scan forwarder, /cmd_vel receiver (:8888), 0.35 s watchdog"],
        ["hil/ & tests/", "Virtual Burger kinematic HIL bench (run_hil.bat) and 161-test automated verification suite"],
    ],
    col_widths=[2.1, 4.17]
)

add_heading_2("5.2 Custom UDP Wire Protocol, Gestures, and Safety Stack")
add_body(
    "To prevent TCP head-of-line blocking over Wi-Fi, camera frames are fragmented into <= 1200 B UDP datagrams prefixed "
    "by a binary header (!2sBIHHd, magic 'TB'). Any frame missing a datagram is dropped immediately so the newest frame "
    "always wins. The laptop sends a 2 Hz HELLO packet from port 5000 to Pi port 8888 so the Pi discovers the laptop's "
    "IP dynamically without hardcoding. Velocity commands stream at 30 Hz as ASCII 'V:<m/s>,W:<rad/s>,S:<seq>\\n', doubling "
    "as the Pi's heartbeat. Table 3 lists the four priority-ordered skeletal gestures, and Table 4 summarizes the "
    "independent safety layers."
)

add_styled_table(
    ["Priority & Gesture", "Landmark Rule (Normalized Image Coords)", "System Response"],
    [
        ["1. STOP_HALT", "Both wrists >= 0.08 above shoulders (4 frames)", "Latches SAFETY_STOP override; accepted from anyone"],
        ["2. RELEASE", "(x_wl - x_wr)(x_sl - x_sr) < -0.005 at chest height", "Deletes owner profile; transitions to OWNER_RELEASED -> IDLE"],
        ["3. FOLLOW_ME", "Either wrist >= 0.04 above nose (4 frames)", "Enrolls owner if IDLE, or starts/resumes FOLLOWING"],
        ["4. WAIT", "Wrist between nose & shoulder, above elbow", "Pauses linear motion (PAUSED); ignores bystanders"],
    ],
    col_widths=[1.45, 2.55, 2.27]
)

add_styled_table(
    ["Safety Layer", "Trigger Condition", "Protective Action"],
    [
        ["Anatomical Pose Gate", "< 6 landmarks or inverted nose/shoulder/hip geometry", "Rejects chairs/furniture before Re-ID"],
        ["Ambiguity Interlock", "Top 2 candidates >= 0.40 within Δ < 0.08", "Enters TARGET_AMBIGUOUS; holds stationary"],
        ["Human Proximity Stop", "Tracked owner closer than 0.55 m", "CRITICAL_PROXIMITY_STOP blocks forward drive"],
        ["LiDAR Reactive & Hard Stop", "Front < 0.30 m (stop) / ±15° raw < 0.20 m (hard); side < 0.20 m", "Zeroes v immediately; blocks turning into obstacle"],
        ["Dual Watchdogs & Fault Gate", "Pi silent > 0.35 s; Laptop > 0.60 s; Cam > 0.8 s; LiDAR > 1.5 s", "Hardware stops motors; enters SENSOR_FAULT"],
    ],
    col_widths=[1.7, 2.55, 2.02]
)

add_body(
    "To monitor perception, localization, and safety interlocks in real time, a Flask web dashboard (http://localhost:8000) "
    "streams the annotated camera feed alongside a top-down Map/RViz occupancy grid, a live 16-state machine diagram, "
    "30-second control-loop plots, and runtime-validated configuration controls (Figure 4 and Figure 5)."
)
add_figure("build_assets/dashboard_top.png", "Figure 4: Live Web Dashboard (Top View) — Simulation AI Feed, Map/RViz Panel, and Safety Status Header.", width_in=6.0)
add_figure("build_assets/dashboard_fsm_view.png", "Figure 5: Live Web Dashboard (Control & Diagnostics View) — State Machine, Control Loop Plots, and Live Settings.", width_in=6.0)

add_heading_2("5.3 Quantitative Benchmarks and Automated Verification")
add_body(
    "All software benchmarks were measured on the development laptop (AMD Ryzen 7 6800HS, Windows 11, Python 3.14.6, "
    "MediaPipe 1.1.1, OpenCV 5.0, CPU only). Running the full verification suite ('python -m unittest discover tests') "
    "executes 161 automated tests spanning unit logic, EKF/PID closed-loop trajectories, A* wall detours, 16-state FSM "
    "coverage, and real Pi-to-laptop UDP loopback communication—passing all 161 tests in 26.75 s with 0 failures."
)

add_figure("build_assets/fig_latency_bench.png", "Figure 6: Per-Stage CPU Execution Latency on Laptop (Median 25.9 ms / ~39 FPS).", width_in=5.9)
add_figure("build_assets/fig_accuracy_clearance.png", "Figure 7: (Left) Closed-Loop Range RMS Error; (Right) Wall-Corner Clearance Before and After Bubble-Core Fix.", width_in=6.15)

add_body(
    "As shown in Figure 6, the full per-frame pipeline executes in a median of 25.9 ms (95th percentile 28.5 ms), leaving "
    "35% CPU headroom inside a 40 ms (25 FPS) real-time budget. Figure 7 highlights two major quantitative gains: "
    "(1) fusing LiDAR clusters with monocular MediaPipe estimates reduces closed-loop range RMS error by 29% (from "
    "0.167 m camera-only down to 0.119 m on walking arcs and 0.106 m on sharp 90° turns, max bearing error <= 4.7°); "
    "(2) retaining a 1-cell obstacle core inside the planner's robot clearing bubble plus adding a 0.20 m side keep-out "
    "zone raised minimum wall-corner clearance from 0.07–0.09 m (a collision with the 0.105 m Burger radius) to "
    "0.15–0.17 m. Furthermore, in a 6 s curved teleoperation test on a simulated room with 0.02 rad/s injected yaw drift, "
    "raw wheel odometry drifted 17.6° while our M3RSM + Gauss-Newton SLAM held pose error to 2 cm and 1.1° (match score 0.98)."
)

add_heading_2("5.4 Physical Laboratory Trials (TurtleBot3 Burger)")
add_body(
    "Live end-to-end validation was conducted in the robotics laboratory using the physical TurtleBot3 Burger and "
    "captured in two demonstration videos (videos/1.mp4 and videos/2.mp4). Figure 8 illustrates Trial 1 (videos/1.mp4, "
    "44.6 s): a cardboard box obstacle was placed directly between the robot and the owner. Upon receiving the FOLLOW_ME "
    "hand gesture, the laptop locked onto the owner (48%–75% Re-ID match), mapped the cardboard box on the live "
    "occupancy grid, generated a magenta A* DETOUR path around the left edge of the box, and drove the physical "
    "TurtleBot3 cleanly around the obstruction while projecting live green LiDAR laser-curtain beams onto the owner "
    "(e.g., cam 3.07 m | lidar 2.68 m)."
)

add_figure("build_assets/v1_detour_poster.png", "Figure 8: Live Trial 1 (videos/1.mp4) — TurtleBot3 Executing A* Detour Around Cardboard Obstacle with Laser-Curtain Fusion.", width_in=6.15)
add_figure("build_assets/v1_ambig_frame.png", "Figure 9: Live Trial 1 (videos/1.mp4) — Multi-Person Bystander Ambiguity Protection ('TWO LOOK-ALIKES: holding until they separate').", width_in=6.15)

add_body(
    "Figure 9 captures the bystander disambiguation event from Trial 1 (at t = 33.5 s): when a second student walked "
    "across the background wearing similar light-colored clothing, the system immediately flagged TARGET_AMBIGUOUS "
    "('TWO LOOK-ALIKES: holding until they separate') and held the robot stationary rather than risking a target swap. "
    "At t = 40.5 s, raising both hands triggered the red EMERGENCY_STOP_GESTURE latch instantaneously. Figure 10 shows "
    "Trial 2 (videos/2.mp4, 27.4 s), demonstrating longitudinal Follow-and-Wait control: the TurtleBot3 accelerates "
    "smoothly up to 0.22 m/s behind the walking owner, decelerates without overshoot as the owner stops at ~1.10–1.20 m, "
    "remains stationary without hunting, and resumes pursuit once the owner steps beyond 1.35 m."
)

add_figure("build_assets/v2_follow_poster.png", "Figure 10: Live Trial 2 (videos/2.mp4) — Physical TurtleBot3 Burger Performing Follow-and-Wait Distance Regulation.", width_in=4.8)

# ===========================================================================
# 6. DISCUSSION
# ===========================================================================
add_heading_1("6. Discussion")
add_body(
    "Analyzing both our simulation benchmarks and the physical TurtleBot3 lab trials reveals several important "
    "embedded robotics insights. First, partitioning computation between the Raspberry Pi 4 and the laptop proved "
    "decisive. Running MediaPipe BlazePose, HSV Re-ID, EKF fusion, and 2-stage SLAM on a single Ryzen 7 CPU thread "
    "consumed 25.9 ms per frame (~39 FPS), leaving generous margin beneath the 40 ms budget, while the Pi 4 remained "
    "lightweight and responsive for ROS 2 `/scan` publishing and the 0.35 s UDP watchdog. Using raw UDP datagrams with "
    "newest-frame-wins semantics completely eliminated the multi-second video lag typically seen with buffered TCP/MJPEG "
    "streams over shared campus Wi-Fi."
)
add_body(
    "Second, camera-only monocular tracking exhibits two structural weaknesses on a 192 mm-tall TurtleBot3 Burger: "
    "range error grows quadratically with pixel span error (0.167 m RMS vs 0.119 m with LiDAR fusion), and when an "
    "owner stands closer than 1.0 m, their shoulders and head rise above the camera's vertical field of view. Coupling "
    "3D camera–LiDAR co-mapping resolved both issues simultaneously. Because the LDS-02 LiDAR sits horizontally at ankle "
    "height, it reliably maintains a compact leg/ankle cluster at close range right where the camera loses upper-body "
    "landmarks, allowing the robot to transition seamlessly to LIDAR_GUIDED_SEARCH or hold cleanly inside the stop "
    "distance without triggering false target-loss spins."
)
add_body(
    "Third, automated closed-loop simulation (`test_recall_sim.py`) uncovered a subtle geometric bug before physical "
    "deployment: clearing a 0.30 m robot-centered bubble in the A* costmap inadvertently erased the inflation around a "
    "sharp wall corner, allowing the smoothed path to clip within 0.07 m of the wall (less than the Burger's 0.105 m "
    "radius). Preserving a 1-cell hard core around real LiDAR returns inside the bubble and enforcing a 0.20 m side "
    "keep-out zone eliminated the collision completely, raising clearance to 0.15–0.17 m—which directly enabled the "
    "clean cardboard-box detour observed in Trial 1 (videos/1.mp4)."
)
add_body(
    "Finally, Trial 1 also confirmed both the value and the boundary of HSV color-histogram Re-ID. When a bystander "
    "entered the lab wearing a similar pale shirt and trousers, the ambiguity detector (Δ < 0.08) rightly refused to "
    "guess and paused the robot safely. While this guarantees zero false target swaps, separating identically dressed "
    "individuals without stopping requires stronger semantic features than color histograms alone."
)

# ===========================================================================
# 7. CONCLUSIONS AND FUTURE SCOPE
# ===========================================================================
add_heading_1("7. Conclusions and Future Scope")
add_body(
    "We have designed, implemented, and experimentally validated an owner-locked human-following mobile robot on the "
    "TurtleBot3 Burger using a distributed Laptop-AI and Thin-Pi architecture. By combining 33-point MediaPipe BlazePose "
    "anatomical filtering, V-channel-invariant multi-template HSV Re-ID, 4-state polar EKF camera–LiDAR fusion, "
    "M3RSM/Gauss-Newton SLAM, A* obstacle recall, and 12 independent safety layers, the robot reliably follows an "
    "authorized operator at ~39 FPS, avoids static obstacles, rejects bystanders, and responds instantaneously to "
    "hands-free gestures. All 161 automated unit and simulation tests pass without failure, and physical lab trials "
    "(videos/1.mp4 and videos/2.mp4) confirm smooth distance regulation, live box detours, ambiguity holding, and "
    "emergency stops on real hardware."
)
add_body(
    "For future work, replacing the 18×16 H–S histogram with a lightweight deep Re-ID embedding network (such as OSNet "
    "or MobileNet-ReID) will allow the robot to disambiguate even identically dressed individuals without pausing. "
    "Extending the A* static planner with a Dynamic Window Approach (DWA) velocity-obstacle layer will improve "
    "anticipation of fast-moving people or rolling carts in busy corridors. In addition, integrating full graph-based "
    "loop closure (`slam_toolbox`) for multi-room traversal, wake-word voice confirmation alongside visual gestures, "
    "and autonomous low-battery return-to-dock behavior will transition the prototype toward continuous 24/7 service deployment."
)

# ===========================================================================
# 8. REFERENCES
# ===========================================================================
add_heading_1("8. References")
refs = [
    "[1] F. Rollo, A. Zunino, G. Raiola, F. Amadio, A. Ajoudani, and N. Tsagarakis, \"FollowMe: A Robust Person Following Framework Based on Visual Re-Identification and Gestures,\" in Proc. 2023 IEEE Int. Conf. Advanced Robotics and Its Social Impacts (ARSO), Berlin, Germany, 2023, pp. 84–89, doi: 10.1109/ARSO56563.2023.10187536.",
    "[2] S. Holkar, V. Mane, P. Badhe, S. Harihar, and B. B. Deshmukh, \"Voice Activated Human Following Robot Using Computer Vision,\" Int. J. Res. Appl. Sci. Eng. Technol. (IJRASET), 2025, doi: 10.22214/ijraset.2025.70950.",
    "[3] M. Do Hoang, S.-S. Yun, and J.-S. Choi, \"The Reliable Recovery Mechanism for Person-Following Robot in Case of Missing Target,\" in Proc. 2017 14th Int. Conf. Ubiquitous Robots and Ambient Intelligence (URAI), Jeju, South Korea, 2017, pp. 800–803, doi: 10.1109/URAI.2017.7992828.",
    "[4] M. Gupta, S. Kumar, L. Behera, and V. K. Subramanian, \"A Novel Vision-Based Tracking Algorithm for a Human-Following Mobile Robot,\" IEEE Trans. Syst., Man, Cybern., Syst., vol. 47, no. 7, pp. 1415–1427, Jul. 2017, doi: 10.1109/TSMC.2016.2616343.",
    "[5] V. Bazarevsky, I. Grishchenko, K. Raveendran, T. Zhu, F. Zhang, and M. Grundmann, \"BlazePose: On-device Real-time Body Pose tracking,\" arXiv preprint arXiv:2006.10204, Jun. 2020.",
    "[6] E. B. Olson, \"Real-time correlative scan matching,\" in Proc. 2009 IEEE Int. Conf. Robotics and Automation (ICRA), Kobe, Japan, 2009, pp. 4387–4393, doi: 10.1109/ROBOT.2009.5152375.",
    "[7] S. Kohlbrecher, O. von Stryk, J. Meyer, and U. Klingauf, \"A flexible and scalable SLAM system with full 3D motion estimation,\" in Proc. 2011 IEEE Int. Symp. Safety, Security, and Rescue Robotics (SSRR), Kyoto, Japan, 2011, pp. 155–160.",
    "[8] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, \"Robot Operating System 2: Design, architecture, and uses in the wild,\" Science Robotics, vol. 7, no. 66, p. eabm6074, May 2022.",
    "[9] ROBOTIS Co., Ltd., \"TurtleBot3 Burger & OpenCR 1.0 Hardware and Bringup e-Manual,\" ROBOTIS Documentation, 2024. [Online]. Available: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/",
    "[10] G. Bradski, \"The OpenCV Library,\" Dr. Dobb's Journal of Software Tools, vol. 25, no. 11, pp. 120–125, 2000.",
    "[11] R. E. Kalman, \"A New Approach to Linear Filtering and Prediction Problems,\" Trans. ASME, J. Basic Eng., vol. 82, no. 1, pp. 35–45, Mar. 1960.",
    "[12] P. E. Hart, N. J. Nilsson, and B. Raphael, \"A Formal Basis for the Heuristic Determination of Minimum Cost Paths,\" IEEE Trans. Syst. Sci. Cybern., vol. 4, no. 2, pp. 100–107, Jul. 1968.",
]
for ref in refs:
    rp = doc.add_paragraph()
    rp.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    rp.paragraph_format.space_after = Pt(5)
    rp.paragraph_format.left_indent = Inches(0.32)
    rp.paragraph_format.first_line_indent = Inches(-0.32)
    r = rp.add_run(ref)
    r.font.size = Pt(10.2)

out_docx = "23ECE436_End_Sem_Project_Report.docx"
doc.save(out_docx)
print(f"Saved {out_docx} successfully.")
