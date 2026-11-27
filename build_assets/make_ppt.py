import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

# Palette: Clean Academic & Robotics Engineering
MAROON = RGBColor(0x8B, 0x15, 0x38)
NAVY = RGBColor(0x0F, 0x17, 0x2A)
BLUE = RGBColor(0x02, 0x84, 0xC7)
TEAL = RGBColor(0x0D, 0x94, 0x88)
SLATE = RGBColor(0x47, 0x55, 0x69)
LIGHT_BG = RGBColor(0xF8, 0xFA, 0xFC)
CARD_BORDER = RGBColor(0xCB, 0xD5, 0xE1)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
SOFT_BLUE = RGBColor(0xE0, 0xF2, 0xFE)
SOFT_MAROON = RGBColor(0xFD, 0xF2, 0xF4)

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
blank_layout = prs.slide_layouts[6]


def add_header(slide, title, subtitle="", slide_num=None):
    # Top accent bar
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(0), Inches(13.333), Inches(0.10))
    bar.fill.solid()
    bar.fill.fore_color.rgb = MAROON
    bar.line.fill.background()

    # Title
    tb = slide.shapes.add_textbox(Inches(0.55), Inches(0.22), Inches(11.5), Inches(0.55))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0
    p = tf.paragraphs[0]
    r = p.add_run()
    r.text = title
    r.font.name = "Segoe UI"
    r.font.size = Pt(24)
    r.font.bold = True
    r.font.color.rgb = NAVY

    if subtitle:
        sb = slide.shapes.add_textbox(Inches(0.55), Inches(0.75), Inches(11.5), Inches(0.32))
        stf = sb.text_frame
        stf.margin_left = stf.margin_top = stf.margin_right = stf.margin_bottom = 0
        sp = stf.paragraphs[0]
        sr = sp.add_run()
        sr.text = subtitle
        sr.font.name = "Segoe UI"
        sr.font.size = Pt(12.5)
        sr.font.color.rgb = BLUE

    # Divider line
    line = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.55), Inches(1.10), Inches(12.23), Inches(0.02))
    line.fill.solid()
    line.fill.fore_color.rgb = CARD_BORDER
    line.line.fill.background()

    # Footer
    ft = slide.shapes.add_textbox(Inches(0.55), Inches(7.12), Inches(9.5), Inches(0.28))
    ftf = ft.text_frame
    ftf.margin_left = ftf.margin_top = ftf.margin_right = ftf.margin_bottom = 0
    fp = ftf.paragraphs[0]
    fr = fp.add_run()
    fr.text = "23ECE436 – Embedded Systems for Robotics  |  Human-Following Mobile Robot  |  ASE Bengaluru"
    fr.font.name = "Segoe UI"
    fr.font.size = Pt(9.5)
    fr.font.color.rgb = SLATE

    if slide_num is not None:
        nt = slide.shapes.add_textbox(Inches(11.8), Inches(7.12), Inches(0.98), Inches(0.28))
        ntf = nt.text_frame
        ntf.margin_left = ntf.margin_top = ntf.margin_right = ntf.margin_bottom = 0
        np_ = ntf.paragraphs[0]
        np_.alignment = PP_ALIGN.RIGHT
        nr = np_.add_run()
        nr.text = f"{slide_num} / 16"
        nr.font.name = "Segoe UI"
        nr.font.size = Pt(9.5)
        nr.font.bold = True
        nr.font.color.rgb = MAROON


def add_card(slide, x, y, w, h, title=None, bullets=None, fill_rgb=LIGHT_BG, border_rgb=CARD_BORDER,
             title_rgb=NAVY, font_size=13, title_size=14.5):
    card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    card.adjustments[0] = 0.03
    card.fill.solid()
    card.fill.fore_color.rgb = fill_rgb
    card.line.color.rgb = border_rgb
    card.line.width = Pt(1.1)

    tb = slide.shapes.add_textbox(Inches(x + 0.18), Inches(y + 0.14), Inches(w - 0.36), Inches(h - 0.26))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0

    first = True
    if title:
        p = tf.paragraphs[0]
        first = False
        p.space_after = Pt(6)
        r = p.add_run()
        r.text = title
        r.font.name = "Segoe UI"
        r.font.size = Pt(title_size)
        r.font.bold = True
        r.font.color.rgb = title_rgb

    if bullets:
        for b in bullets:
            p = tf.paragraphs[0] if first else tf.add_paragraph()
            first = False
            p.space_after = Pt(5)
            p.line_spacing = 1.15
            if isinstance(b, tuple):
                lead, rest = b
                r1 = p.add_run()
                r1.text = "• " + lead + " "
                r1.font.name = "Segoe UI"
                r1.font.size = Pt(font_size)
                r1.font.bold = True
                r1.font.color.rgb = NAVY
                r2 = p.add_run()
                r2.text = rest
                r2.font.name = "Segoe UI"
                r2.font.size = Pt(font_size)
                r2.font.color.rgb = SLATE
            else:
                r = p.add_run()
                r.text = "• " + b
                r.font.name = "Segoe UI"
                r.font.size = Pt(font_size)
                r.font.color.rgb = SLATE
    return tb


def style_table(table, header_bg=NAVY, font_size=11):
    for r_idx, row in enumerate(table.rows):
        for c_idx, cell in enumerate(row.cells):
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.margin_left = Inches(0.08)
            cell.margin_right = Inches(0.08)
            cell.margin_top = Inches(0.04)
            cell.margin_bottom = Inches(0.04)
            fill = cell.fill
            fill.solid()
            if r_idx == 0:
                fill.fore_color.rgb = header_bg
            elif r_idx % 2 == 1:
                fill.fore_color.rgb = WHITE
            else:
                fill.fore_color.rgb = LIGHT_BG
            for p in cell.text_frame.paragraphs:
                for run in p.runs:
                    run.font.name = "Segoe UI"
                    run.font.size = Pt(font_size if r_idx > 0 else font_size + 0.5)
                    if r_idx == 0:
                        run.font.bold = True
                        run.font.color.rgb = WHITE
                    else:
                        run.font.color.rgb = NAVY if c_idx == 0 else SLATE
                        if c_idx == 0:
                            run.font.bold = True


# ===========================================================================
# SLIDE 1: TITLE SLIDE
# ===========================================================================
s1 = prs.slides.add_slide(blank_layout)
top_strip = s1.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(0), Inches(13.333), Inches(0.18))
top_strip.fill.solid()
top_strip.fill.fore_color.rgb = MAROON
top_strip.line.fill.background()

bot_strip = s1.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(7.32), Inches(13.333), Inches(0.18))
bot_strip.fill.solid()
bot_strip.fill.fore_color.rgb = NAVY
bot_strip.line.fill.background()

if os.path.exists("build_assets/amrita_logo.png"):
    s1.shapes.add_picture("build_assets/amrita_logo.png", Inches(4.55), Inches(0.45), width=Inches(4.2))

tb1 = s1.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(11.73), Inches(0.8))
tf1 = tb1.text_frame
p = tf1.paragraphs[0]
p.alignment = PP_ALIGN.CENTER
r = p.add_run()
r.text = "Amrita School of Engineering, Bengaluru  •  Department of ECE  •  VII Semester\n23ECE436 – Embedded Systems for Robotics"
r.font.name = "Segoe UI"
r.font.size = Pt(14)
r.font.bold = True
r.font.color.rgb = SLATE

tb_title = s1.shapes.add_textbox(Inches(0.9), Inches(2.55), Inches(11.53), Inches(1.5))
ttf = tb_title.text_frame
ttf.word_wrap = True
tp = ttf.paragraphs[0]
tp.alignment = PP_ALIGN.CENTER
tr = tp.add_run()
tr.text = "Vision & LiDAR-Guided Owner-Locked\nHuman-Following Mobile Robot"
tr.font.name = "Segoe UI"
tr.font.size = Pt(34)
tr.font.bold = True
tr.font.color.rgb = NAVY

tp2 = ttf.add_paragraph()
tp2.alignment = PP_ALIGN.CENTER
tp2.space_before = Pt(8)
tr2 = tp2.add_run()
tr2.text = "Distributed Edge-AI Architecture on TurtleBot3 Burger with Gesture HRI, Multi-Template Re-ID & A* Recall"
tr2.font.name = "Segoe UI"
tr2.font.size = Pt(14)
tr2.font.color.rgb = BLUE

# Team & Evaluator Cards
add_card(s1, 1.1, 4.45, 5.4, 2.35,
         title="Submitted By (Batch Members)",
         bullets=[
             ("Gade Srishanth", "— BL.EN.U4ECE23213"),
             ("D V S Sainath Reddy", "— BL.EN.U4ECE23208"),
             ("Rushikesh D", "— BL.EN.U4ECE23207"),
         ],
         fill_rgb=LIGHT_BG, border_rgb=CARD_BORDER, title_rgb=MAROON, font_size=13.5, title_size=15)

add_card(s1, 6.85, 4.45, 5.38, 2.35,
         title="Course Evaluation & Details",
         bullets=[
             ("Evaluated by:", "Dr. Sreeja Kochuvila"),
             ("Designation:", "Associate Professor, Dept. of ECE"),
             ("Platform:", "TurtleBot3 Burger + Raspberry Pi 4 + Laptop"),
             ("Date:", "November 2026"),
         ],
         fill_rgb=SOFT_MAROON, border_rgb=MAROON, title_rgb=MAROON, font_size=13, title_size=15)


# ===========================================================================
# SLIDE 2: INTRODUCTION
# ===========================================================================
s2 = prs.slides.add_slide(blank_layout)
add_header(s2, "1. Introduction", "Motivation, Real-World Challenges & System Concept", 2)

add_card(s2, 0.55, 1.35, 5.9, 5.5,
         title="Why Owner-Locked Human Following?",
         bullets=[
             ("Service & Assistive Robotics:", "Hospitals, airports, warehouses, and smart homes need mobile robots that carry loads behind a specific user without manual teleoperation."),
             ("Limitations of Naive Tracking:", "Basic blob or nearest-person trackers swap targets whenever a bystander crosses, confuse chairs with people, and fail when the user turns sideways."),
             ("Compute Bottleneck on Pi:", "A Raspberry Pi 4 cannot run modern skeletal pose estimation alongside ROS 2 motor control and SLAM at real-time frame rates."),
         ],
         fill_rgb=LIGHT_BG, font_size=13.5)

add_card(s2, 6.75, 1.35, 6.0, 2.65,
         title="Our Distributed Solution",
         bullets=[
             ("Thin-Pi Hardware Bridge:", "Raspberry Pi 4 handles only USB camera capture, LDS-02 LiDAR forwarding, and a 0.35 s safety watchdog to OpenCR 1.0."),
             ("Laptop Edge-AI Hub:", "All heavy vision, MediaPipe pose, HSV Re-ID, EKF fusion, SLAM, A* path planning, and PID run at ~39 FPS on the laptop."),
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, title_rgb=NAVY, font_size=13)

add_card(s2, 6.75, 4.20, 6.0, 2.65,
         title="Key Operational Guarantees",
         bullets=[
             ("Explicit Gesture Control:", "Enroll, pause, resume, release, or emergency-stop with intuitive upper-body gestures."),
             ("Never Hunts When Stopped:", "Holds indefinitely at 1.10 m while the owner rests; activates map recall and 360° search only on genuine target loss."),
         ],
         fill_rgb=WHITE, border_rgb=TEAL, title_rgb=TEAL, font_size=13)


# ===========================================================================
# SLIDE 3: OBJECTIVES
# ===========================================================================
s3 = prs.slides.add_slide(blank_layout)
add_header(s3, "2. Objectives & Novelty", "Core Engineering Goals & Project Contributions", 3)

add_card(s3, 0.55, 1.35, 7.2, 5.5,
         title="Primary Project Objectives",
         bullets=[
             ("Distributed Real-Time Pipeline:", "Decouple low-level ROS 2 motor actuation (Pi) from high-level AI perception and planning (Laptop) over low-latency UDP."),
             ("Anatomical Human Verification:", "Use 33-point MediaPipe BlazePose landmarks to reject chairs, furniture, and background clutter."),
             ("Identity-Locked Re-Identification:", "Enroll a single owner via gesture and maintain identity across viewpoints using a protected multi-template HSV bank."),
             ("Adaptive Distance & Smooth Motion:", "Maintain a dynamic following distance (base 1.20 m) with 25 cm anti-chatter hysteresis and slew-rate limited PID control."),
             ("Safe Obstacle Avoidance & Watchdogs:", "Combine 360° LiDAR reactive keep-out zones, A* detours, and independent Pi/Laptop hardware watchdogs."),
         ],
         fill_rgb=LIGHT_BG, font_size=13)

add_card(s3, 8.0, 1.35, 4.78, 5.5,
         title="Novelty of Our Work",
         bullets=[
             ("1. Online Camera–LiDAR Co-Mapping & Self-Calibration:",
              "Projects 360° LiDAR beams onto the camera image ('laser curtain') to fuse EKF range/bearing and adapt camera scale (kv) and yaw bias (b_cam) online — maintaining lock via LiDAR clusters even when the low camera only sees legs at close range."),
             ("2. Map-Memory A* Recall Before Rotational Search:",
              "Instead of spinning blindly when occluded behind a wall, the robot drives around the obstacle using A* on a scan-matched occupancy grid to the owner's last-seen coordinate before initiating a 3-phase LiDAR-guarded search."),
         ],
         fill_rgb=SOFT_MAROON, border_rgb=MAROON, title_rgb=MAROON, font_size=12.8, title_size=15.5)


# ===========================================================================
# SLIDE 4: LITERATURE SURVEY
# ===========================================================================
s4 = prs.slides.add_slide(blank_layout)
add_header(s4, "3. Literature Survey", "Comparative Review of Prior Human-Following Frameworks", 4)

rows_lit = [
    ["Author(s) & Year", "Core Approach", "Identified Limitation", "How Our Work Addresses It"],
    ["Rollo et al., IEEE ARSO 2023\n(FollowMe — Base Paper)", "Visual Re-ID + hand gestures for person locking & following", "Monocular/RGB-D range degrades or loses legs at close range; no map recall", "Adds 360° LiDAR–camera EKF fusion, online scale calibration & A* recall"],
    ["Do Hoang et al., URAI 2017", "Missing-target recovery mechanism for person following", "Relies on reactive turns without building a persistent obstacle map", "Maintains 12x12 m M3RSM SLAM occupancy grid for wall-detour recall"],
    ["Gupta et al., IEEE TSMC 2017", "Novel vision-based tracking algorithm for mobile robots", "Sensitive to illumination shifts and similar-looking bystanders", "Uses V-channel-invariant 18x16 H-S histograms + ambiguity stop (Δ<0.08)"],
    ["Holkar et al., IJRASET 2025", "Voice-activated human-following robot using CV", "Voice commands fail in noisy labs; lacks multi-layer safety gating", "Uses silent visual gestures + 12 independent safety boundary layers"],
    ["Olson (ICRA 2011) & Kohlbrecher\n(SSRR 2011 — Hector SLAM)", "M3RSM correlative scan matching & Gauss-Newton alignment", "Static-world assumption smears maps when dynamic humans walk", "Masks tracked owner & transient objects from occupancy updates"],
]

tbl_shape = s4.shapes.add_table(len(rows_lit), 4, Inches(0.55), Inches(1.35), Inches(12.23), Inches(5.45))
tbl = tbl_shape.table
col_ws = [Inches(2.7), Inches(2.9), Inches(3.2), Inches(3.43)]
for idx, w in enumerate(col_ws):
    tbl.columns[idx].width = w
for r_i, row in enumerate(rows_lit):
    for c_i, val in enumerate(row):
        tbl.cell(r_i, c_i).text = val
style_table(tbl, header_bg=NAVY, font_size=11)


# ===========================================================================
# SLIDE 5: METHODOLOGY
# ===========================================================================
s5 = prs.slides.add_slide(blank_layout)
add_header(s5, "4. Methodology", "Closed-Loop Perception, Fusion, Planning & Actuation Pipeline", 5)

add_card(s5, 0.55, 1.32, 12.23, 1.35,
         title="Per-Frame Closed-Loop Execution Flow (~25.9 ms median / 39 FPS on CPU)",
         bullets=[
             "UDP Frame RX (newest wins)  ->  Resize 480 px  ->  MediaPipe PoseLandmarker (33 pts)  ->  Anatomical Filter  ->  HSV Signature & 3-Anchor Distance",
             "->  Gesture Classifier (4-frame debounce)  ->  Owner Re-ID & Ambiguity Gate  ->  EKF Fusion (Camera + LiDAR)  ->  16-State FSM  ->  PID + A* + LiDAR Safety  ->  30 Hz UDP TX",
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, title_rgb=NAVY, font_size=11.8, title_size=13.5)

add_card(s5, 0.55, 2.85, 5.95, 4.05,
         title="Mathematical Modelling Summary",
         bullets=[
             ("Pinhole Bearing & Range:", "f_px = (W/2)/tan(HFOV/2);  d_cam = mean(K_i / s_i) over shoulder (235), hip (160) & torso (230) anchors."),
             ("HSV Appearance Score:", "S = 0.70*S_color + 0.18*S_head + 0.12*S_geom using Bhattacharyya distance d_B and Pearson correlation ρ."),
             ("4-State Polar EKF:", "State s = [px, py, ux, uy]^T with ego-motion removal R(-ωΔt), Joseph covariance update & χ² gate (< 11.3)."),
             ("PID + Feed-Forward:", "v = [Kp*ed + Ki*∫ed + Kd*ṙ + Kff*u_los*φ(r)]*cos²(β), capped at 0.22 m/s and 2.0 rad/s."),
         ],
         fill_rgb=LIGHT_BG, font_size=12.2)

add_card(s5, 6.75, 2.85, 6.03, 4.05,
         title="Dynamic Following & Recovery Strategy",
         bullets=[
             ("Adaptive Set-Point:", "d* = d_set + min(0.60, 0.50*||u|| + 0.35*(1 - c)), expanding safely with speed or lower confidence."),
             ("Anti-Chatter Hysteresis:", "Stops at d_stop = 1.10 m (WAIT_FOR_OWNER_MOVEMENT); resumes only when r >= 1.35 m for 3 consecutive frames."),
             ("Co-Mapping Calibration:", "Slowly adapts vision scale k_v ∈ [0.6, 1.6] and bearing bias b_cam against trusted 5 cm-accurate LiDAR clusters."),
             ("Recall Before Search:", "On target loss, A* navigates to last map-frame position (<=25 s) before executing 3-phase 360° rotational search."),
         ],
         fill_rgb=WHITE, border_rgb=CARD_BORDER, title_rgb=MAROON, font_size=12.2)


# ===========================================================================
# SLIDE 6: DESIGN (1/3) — ARCHITECTURE & HARDWARE
# ===========================================================================
s6 = prs.slides.add_slide(blank_layout)
add_header(s6, "5. System Design (1/3): Distributed Architecture & Hardware", "Choice of Sensors, Actuators & Edge Compute Split", 6)

if os.path.exists("build_assets/fig_architecture.png"):
    s6.shapes.add_picture("build_assets/fig_architecture.png", Inches(0.55), Inches(1.25), width=Inches(7.75))

hw_rows = [
    ["Subsystem", "Hardware Selected", "Role & Spec"],
    ["Mobile Base", "TurtleBot3 Burger", "Diff. drive; 0.22 m/s, 2.84 rad/s"],
    ["Actuators", "2x Dynamixel XL430", "Closed-loop wheel velocity"],
    ["Controller", "OpenCR 1.0 (STM32F7)", "IMU, battery & /dev/ttyACM0"],
    ["Onboard SBC", "Raspberry Pi 4 (4GB)", "ROS 2 Humble, UDP bridge only"],
    ["360° Sensor", "ROBOTIS LDS-02", "0.16–8.0 m, 360x1° bins"],
    ["Vision Sensor", "USB UVC Camera", "640x480 @ 25 FPS, 57° HFOV"],
    ["Power", "11.1V 3S LiPo 1800mAh", "Powers OpenCR + Pi (cutoff 11.0V)"],
    ["AI Compute", "Ryzen 7 6800HS Laptop", "39 FPS CPU inference + UI"],
]
t6 = s6.shapes.add_table(len(hw_rows), 3, Inches(8.42), Inches(1.30), Inches(4.38), Inches(5.55)).table
t6.columns[0].width = Inches(1.15)
t6.columns[1].width = Inches(1.55)
t6.columns[2].width = Inches(1.68)
for r_i, row in enumerate(hw_rows):
    for c_i, val in enumerate(row):
        t6.cell(r_i, c_i).text = val
style_table(t6, header_bg=MAROON, font_size=9.5)

add_card(s6, 0.55, 5.15, 7.75, 1.72,
         title="Why Thin-Pi + Laptop Edge Compute?",
         bullets=[
             "Pi 4 lacks throughput for 33-pt BlazePose + SLAM simultaneously; offloading AI to the laptop yields 39 FPS while keeping safety-critical watchdogs local on the Pi (0.35 s cutoff).",
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, font_size=11.5, title_size=12.5)


# ===========================================================================
# SLIDE 7: DESIGN (2/3) — PERCEPTION, RE-ID & FUSION
# ===========================================================================
s7 = prs.slides.add_slide(blank_layout)
add_header(s7, "5. System Design (2/3): Perception, Re-ID & Sensor Fusion", "Human Validation, Multi-Template Identity & EKF Tracking", 7)

add_card(s7, 0.55, 1.35, 3.9, 5.5,
         title="1. Pose & Anatomical Filter",
         bullets=[
             ("MediaPipe BlazePose (Lite):", "Extracts 33 landmarks up to MAX_POSES = 3 people per frame."),
             ("Chair / Clutter Rejection:", "Requires >= 6 confident landmarks, valid shoulder/hip hierarchy, nose above shoulders, and plausible shoulder span."),
             ("Close-Range Resilience:", "Uses pelvic & torso anchors when the head leaves the top border at <1.1 m; hands off to LiDAR cluster if only legs remain visible."),
         ],
         fill_rgb=LIGHT_BG, font_size=12.2)

add_card(s7, 4.70, 1.35, 4.0, 5.5,
         title="2. Multi-Template HSV Re-ID",
         bullets=[
             ("V-Channel Invariant:", "18x16 Hue-Saturation histograms ignore indoor brightness fluctuations."),
             ("Protected Anchor T₀:", "Immutable baseline stored on enrollment; up to 8 side/back templates added only when alone and score >= 0.45 (EMA α <= 0.08)."),
             ("Strict Re-Acquisition:", "After loss, requires >= 5 consecutive frames above max(0.40, 0.60·S̄); stops immediately in TARGET_AMBIGUOUS if top 2 match within Δ < 0.08."),
         ],
         fill_rgb=WHITE, border_rgb=BLUE, title_rgb=BLUE, font_size=12.2)

add_card(s7, 8.95, 1.35, 3.83, 5.5,
         title="3. EKF & LiDAR Co-Mapping",
         bullets=[
             ("3D Projection:", "Projects 360° LiDAR beams into image coordinates (u, v) as a live laser curtain."),
             ("Noise Covariance:", "Camera σ_r = 0.12d, σ_β = 1.5°; LiDAR cluster σ_r = 0.05 m, σ_β = 1.0°."),
             ("Online Calibration:", "LiDAR range slowly tunes camera scale k_v and bearing bias b_cam."),
             ("M3RSM SLAM + A*:", "0.05 m occupancy grid corrected via correlative + Gauss-Newton scan matching."),
         ],
         fill_rgb=SOFT_BLUE, border_rgb=NAVY, title_rgb=NAVY, font_size=12.2)


# ===========================================================================
# SLIDE 8: DESIGN (3/3) — STATE MACHINE, GESTURES & SAFETY
# ===========================================================================
s8 = prs.slides.add_slide(blank_layout)
add_header(s8, "5. System Design (3/3): 16-State FSM, Gestures & Safety", "Deterministic Behavior Transitions & Multi-Layer Interlocks", 8)

if os.path.exists("build_assets/dashboard_fsm.png"):
    s8.shapes.add_picture("build_assets/dashboard_fsm.png", Inches(0.55), Inches(1.22), width=Inches(5.55))
elif os.path.exists("build_assets/fig_fsm.png"):
    s8.shapes.add_picture("build_assets/fig_fsm.png", Inches(0.50), Inches(1.22), width=Inches(8.15))

gest_rows = [
    ["Gesture", "Pose Rule (4-frame debounce)", "Action"],
    ["FOLLOW_ME", "Either wrist >= 0.04 above nose", "Enroll / Follow"],
    ["WAIT", "Upright forearm at chest/shoulder", "Pause (Hold)"],
    ["RELEASE", "Wrists crossed in 'X' over chest", "Unlock -> IDLE"],
    ["STOP_HALT", "Both wrists >= 0.08 above shoulders", "E-STOP Latch"],
]
t8 = s8.shapes.add_table(len(gest_rows), 3, Inches(6.30), Inches(1.28), Inches(6.48), Inches(2.55)).table
t8.columns[0].width = Inches(1.35)
t8.columns[1].width = Inches(3.25)
t8.columns[2].width = Inches(1.88)
for r_i, row in enumerate(gest_rows):
    for c_i, val in enumerate(row):
        t8.cell(r_i, c_i).text = val
style_table(t8, header_bg=MAROON, font_size=10)

add_card(s8, 6.30, 4.00, 6.48, 2.95,
         title="12 Independent Safety Layers",
         bullets=[
             ("Proximity Stop:", "Blocks motion if human < 0.55 m."),
             ("LiDAR Hard Stop:", "Zeroes forward v if ±15° raw < 0.20 m."),
             ("Keep-Out Zone:", "Blocks turn toward side obstacle < 0.20 m."),
             ("Dual Watchdogs:", "Pi cuts motors in 0.35 s; Laptop in 0.60 s."),
             ("Stale Video Gate:", "Rejects offline frames > 0.8 s (SENSOR_FAULT)."),
         ],
         fill_rgb=SOFT_MAROON, border_rgb=MAROON, title_rgb=MAROON, font_size=11.2, title_size=13)


# ===========================================================================
# SLIDE 9: IMPLEMENTATION (1/2) — SOFTWARE & DASHBOARD
# ===========================================================================
s9 = prs.slides.add_slide(blank_layout)
add_header(s9, "6. Implementation (1/2): Software Stack & Web Dashboard", "Custom UDP Protocol, Modular Python Architecture & Live UI", 9)

if os.path.exists("build_assets/dashboard_top.png"):
    s9.shapes.add_picture("build_assets/dashboard_top.png", Inches(0.55), Inches(1.25), width=Inches(5.35))
if os.path.exists("build_assets/dashboard_fsm_view.png"):
    s9.shapes.add_picture("build_assets/dashboard_fsm_view.png", Inches(0.55), Inches(4.78), width=Inches(5.35))

add_card(s9, 6.10, 1.25, 6.68, 5.65,
         title="Web Dashboard & UDP Implementation",
         bullets=[
             ("Live AI & RViz Panels (Top Left):", "Displays live camera/simulation feed with MediaPipe skeleton & LiDAR laser curtain alongside real-time 0.05 m occupancy grid, robot pose, and A* path."),
             ("Control Loop & Live Settings (Bottom Left):", "Streams 30 s range/velocity/Re-ID telemetry over SSE and tunes follow distance, PID gains & PI_IP live into config.local.json."),
             ("laptop/ (13 modules):", "cv_pipeline.py, ekf_person.py, lidar.py, navigation.py, appearance_model.py, follow_controller.py, robot_controller.py, app.py."),
             ("pi/ (Thin Node):", "pi_turtlebot_node.py, tb_proto.py, start_robot.sh (ROS 2 Humble, domain 30)."),
             ("Binary UDP Protocol:", "Header '!2sBIHHd' (magic 'TB'); <=1200 B JPEG chunks on :5000 (newest frame wins); V:vx,W:wz,S:seq @ 30 Hz on :8888."),
         ],
         fill_rgb=LIGHT_BG, font_size=11.5, title_size=13.5)


# ===========================================================================
# SLIDE 10: IMPLEMENTATION (2/2) — HIL BENCH, TESTS & HYSTERESIS
# ===========================================================================
s10 = prs.slides.add_slide(blank_layout)
add_header(s10, "6. Implementation (2/2): Hardware Bringup, HIL Bench & Verification", "TurtleBot3 Bringup, Kinematic Simulator & Automated Test Suite", 10)

if os.path.exists("build_assets/fig_hysteresis.png"):
    s10.shapes.add_picture("build_assets/fig_hysteresis.png", Inches(0.55), Inches(1.32), width=Inches(7.25))

add_card(s10, 0.55, 4.48, 7.25, 2.40,
         title="Pi Bringup & HIL Simulation Bench",
         bullets=[
             ("On-Robot Bringup (pi/start_robot.sh):", "Launches turtlebot3_bringup, enables Dynamixel torque (/motor_power), and binds UDP :8888 without hosting any HTTP server."),
             ("HIL Bench (hil/run_hil.py):", "Simulates Burger differential-drive kinematics, 360° LiDAR ray-casting, and UDP protocol using the laptop webcam while robot battery charges."),
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, font_size=11.5, title_size=13)

add_card(s10, 8.00, 1.32, 4.78, 5.56,
         title="Automated Verification (161 Tests — OK)",
         bullets=[
             ("test_behaviors.py (24):", "Wait/resume hysteresis, LiDAR-only close tracking, 360° search, safety overrides."),
             ("test_navigation.py (22):", "Log-odds grid, M3RSM scan matching, A* detours, person memory."),
             ("test_system.py (16):", "Gestures, chair rejection, ambiguity stop & owner lock."),
             ("test_lidar.py (13):", "Clustering, sector extraction, keep-out avoidance."),
             ("test_follow_sim.py (11):", "Closed-loop EKF + PID trajectory tracking."),
             ("test_udp_link.py (11):", "Real Pi node <-> laptop over UDP loopback."),
             ("test_fusion_config (11) & pipeline_states (8):", "Co-mapping geometry & 16-state transition coverage."),
             ("Additional Suites (45):", "HIL plant, fusion pipeline, recall sim & state-machine checks — all 161 pass."),
         ],
         fill_rgb=LIGHT_BG, font_size=11, title_size=13)


# ===========================================================================
# SLIDE 11: RESULTS & ANALYSIS (1/3) — BENCHMARKS
# ===========================================================================
s11 = prs.slides.add_slide(blank_layout)
add_header(s11, "7. Results & Analysis (1/3): Quantitative Benchmarks", "Real-Time Latency, EKF Tracking Accuracy & Obstacle Clearance", 11)

if os.path.exists("build_assets/fig_latency_bench.png"):
    s11.shapes.add_picture("build_assets/fig_latency_bench.png", Inches(0.50), Inches(1.25), width=Inches(6.20))

if os.path.exists("build_assets/fig_accuracy_clearance.png"):
    s11.shapes.add_picture("build_assets/fig_accuracy_clearance.png", Inches(6.80), Inches(1.25), width=Inches(6.05))

add_card(s11, 0.50, 4.42, 12.33, 2.48,
         title="Key Quantitative Findings",
         bullets=[
             ("Real-Time CPU Throughput (~39 FPS):", "Full per-frame pipeline median is 25.9 ms (p95 28.5 ms) on Ryzen 7 6800HS CPU — using 65% of the 40 ms budget at 25 FPS."),
             ("29% Lower Range Error via Sensor Fusion:", "LiDAR+Camera EKF reduces closed-loop range RMS error from 0.167 m (camera only) to 0.119 m (0.106 m in 90° turns), with <= 4.7° max bearing error."),
             ("Eliminated Corner-Clipping Collision:", "Keeping a 1-cell hard obstacle core inside the planner's robot bubble + side keep-out zone raised minimum wall-corner clearance from 0.07–0.09 m to 0.15–0.17 m (safely above 0.105 m Burger radius)."),
             ("SLAM Drift Correction:", "On a 6 s curved teleop run with 0.02 rad/s injected yaw drift, odometry drifted 17.6° while laptop M3RSM+GN SLAM held error to 2 cm and 1.1° (match score 0.98)."),
         ],
         fill_rgb=LIGHT_BG, font_size=11.3, title_size=13)


# ===========================================================================
# SLIDE 12: RESULTS & ANALYSIS (2/3) — LIVE DEMO VIDEO 1
# ===========================================================================
s12 = prs.slides.add_slide(blank_layout)
add_header(s12, "7. Results & Analysis (2/3): Physical Trial 1 — Obstacle Detour & Re-ID", "Live TurtleBot3 Burger Lab Evaluation (videos/1.mp4)", 12)

if os.path.exists("videos/1.mp4"):
    poster1 = "build_assets/v1_detour_poster.png" if os.path.exists("build_assets/v1_detour_poster.png") else None
    s12.shapes.add_movie(
        "videos/1.mp4",
        Inches(0.55), Inches(1.32), Inches(8.15), Inches(3.50),
        poster_frame_image=poster1,
        mime_type="video/mp4"
    )

add_card(s12, 8.88, 1.32, 3.90, 5.55,
         title="Observed Trial 1 Behavior",
         bullets=[
             ("1. Gesture Enrollment:", "Owner raises hand (FOLLOW_ME) behind cardboard obstacle; Re-ID locks onto owner (48%–75% match)."),
             ("2. A* Detour Navigation:", "LDS-02 LiDAR maps the box on the occupancy grid; magenta A* path routes cleanly around the left edge (DETOUR / AVOIDING)."),
             ("3. Laser Curtain Fusion:", "Live HUD confirms tight agreement between camera and LiDAR range (e.g., cam 3.07 m | lidar 2.68 m)."),
             ("4. Bystander Ambiguity:", "When a second person enters at 0:33, system triggers TARGET_AMBIGUOUS ('TWO LOOK-ALIKES') and holds safely until separation."),
             ("5. STOP_HALT Latch:", "Raising both hands at 0:40 halts motors immediately."),
         ],
         fill_rgb=LIGHT_BG, font_size=11.2, title_size=13)

add_card(s12, 0.55, 4.98, 8.15, 1.88,
         title="Video Clip Note (Click Video Above to Play in Slide Show)",
         bullets=[
             ("File:", "videos/1.mp4 (2520x1080, 30 fps, 44.6 s) — Split view: physical robot in lab (left) + live Flask Web Dashboard & LiDAR Map (right)."),
             ("Verification:", "Demonstrates obstacle avoidance, camera–LiDAR co-mapping, bystander protection, and gesture E-STOP on real TurtleBot3 hardware."),
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, font_size=11.2, title_size=12.5)


# ===========================================================================
# SLIDE 13: RESULTS & ANALYSIS (3/3) — LIVE DEMO VIDEO 2
# ===========================================================================
s13 = prs.slides.add_slide(blank_layout)
add_header(s13, "7. Results & Analysis (3/3): Physical Trial 2 — Follow & Wait Control", "Live Distance Hysteresis & Smooth Pursuit (videos/2.mp4)", 13)

if os.path.exists("videos/2.mp4"):
    poster2 = "build_assets/v2_follow_poster.png" if os.path.exists("build_assets/v2_follow_poster.png") else None
    s13.shapes.add_movie(
        "videos/2.mp4",
        Inches(0.55), Inches(1.32), Inches(8.15), Inches(3.50),
        poster_frame_image=poster2,
        mime_type="video/mp4"
    )

add_card(s13, 8.88, 1.32, 3.90, 5.55,
         title="Observed Trial 2 Behavior",
         bullets=[
             ("1. Smooth Pursuit:", "Owner initiates FOLLOW_ME and walks forward/backward along the lab corridor."),
             ("2. Velocity & Slew Control:", "Burger accelerates smoothly up to 0.22 m/s without wheel slip or jerky oscillation."),
             ("3. Follow-and-Wait Stop:", "As owner pauses, the robot decelerates smoothly and halts cleanly at the target buffer (~1.10–1.20 m)."),
             ("4. Zero Hunting:", "While the owner stands stationary, the robot remains locked and stationary — never rotating to search for another person."),
             ("5. Hysteresis Resume:", "When owner steps back beyond 1.35 m, robot resumes tracking smoothly."),
         ],
         fill_rgb=LIGHT_BG, font_size=11.2, title_size=13)

add_card(s13, 0.55, 4.98, 8.15, 1.88,
         title="Video Clip Note (Click Video Above to Play in Slide Show)",
         bullets=[
             ("File:", "videos/2.mp4 (2520x1080, 30 fps, 27.4 s) — Direct camera recording of physical TurtleBot3 Burger executing longitudinal follow-and-wait."),
             ("Verification:", "Confirms smooth Dynamixel actuation over UDP :8888 and stable distance holding without oscillation."),
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, font_size=11.2, title_size=12.5)


# ===========================================================================
# SLIDE 14: CONCLUSIONS & NOVELTY
# ===========================================================================
s14 = prs.slides.add_slide(blank_layout)
add_header(s14, "8. Conclusions & Novelty", "Summary of Engineering Outcomes & Key Contributions", 14)

add_card(s14, 0.55, 1.38, 5.95, 5.45,
         title="Project Conclusions",
         bullets=[
             ("Verified Distributed Architecture:", "Offloading AI to the laptop achieved ~39 FPS CPU throughput while keeping the Pi ultra-thin and deterministic."),
             ("Robust Owner-Locked Following:", "Multi-template HSV Re-ID + anatomical pose checks + ambiguity gating prevent target-swapping and false furniture locks."),
             ("Reliable Multi-Sensor Control:", "EKF camera–LiDAR fusion lowered range error by 29% (0.119 m RMS) and eliminated close-range camera blind spots."),
             ("Hardware & Test Proven:", "161 automated unit/sim tests (0 failures) and live TurtleBot3 lab trials confirm smooth following, A* box detours, and instant STOP_HALT response."),
         ],
         fill_rgb=LIGHT_BG, font_size=13)

add_card(s14, 6.78, 1.38, 6.00, 5.45,
         title="Summary of Novelties",
         bullets=[
             ("Camera–LiDAR Co-Mapping & Self-Calibration:", "Real-time 3D beam projection calibrates monocular scale k_v and bearing bias b_cam online against LiDAR clusters."),
             ("Map-Memory A* Recall Before Search:", "Remembers the owner's map-frame coordinate behind occluding walls and plans a collision-free detour (0.15–0.17 m clearance) before rotating."),
             ("12-Layer Independent Safety Stack:", "Hardware watchdogs (0.35 s Pi / 0.60 s Laptop), LiDAR hard stop (<0.20 m), side keep-out, and stale-frame gating."),
         ],
         fill_rgb=SOFT_MAROON, border_rgb=MAROON, title_rgb=MAROON, font_size=13)


# ===========================================================================
# SLIDE 15: FUTURE SCOPE
# ===========================================================================
s15 = prs.slides.add_slide(blank_layout)
add_header(s15, "9. Future Scope", "Planned Enhancements for Next-Generation Deployment", 15)

add_card(s15, 0.55, 1.38, 6.0, 2.55,
         title="1. Deep-Learning Re-ID Embeddings",
         bullets=[
             "Replace H–S color histograms with lightweight neural embeddings (e.g., OSNet / MobileNet-ReID) to separate two people wearing identically colored clothing.",
         ],
         fill_rgb=LIGHT_BG, font_size=13)

add_card(s15, 6.78, 1.38, 6.0, 2.55,
         title="2. Dynamic Obstacle Velocity Prediction",
         bullets=[
             "Upgrade A* static detours with Dynamic Window Approach (DWA) or velocity obstacles to anticipate fast-moving carts and walking crowds.",
         ],
         fill_rgb=SOFT_BLUE, border_rgb=BLUE, font_size=13)

add_card(s15, 0.55, 4.20, 6.0, 2.65,
         title="3. Loop-Closure Multi-Room SLAM",
         bullets=[
             "Integrate full graph-based loop closure (slam_toolbox) across multi-floor corridors to prevent long-term map drift beyond the 12x12 m sliding window.",
         ],
         fill_rgb=WHITE, border_rgb=TEAL, title_rgb=TEAL, font_size=13)

add_card(s15, 6.78, 4.20, 6.0, 2.65,
         title="4. Multimodal HRI & Auto-Docking",
         bullets=[
             "Add wake-word voice confirmation alongside visual gestures, plus automated battery-low return-to-dock behavior (<11.1 V threshold).",
         ],
         fill_rgb=LIGHT_BG, font_size=13)


# ===========================================================================
# SLIDE 16: REFERENCES
# ===========================================================================
s16 = prs.slides.add_slide(blank_layout)
add_header(s16, "10. References", "Academic Literature, Frameworks & Hardware Documentation", 16)

refs = [
    "[1] F. Rollo, A. Zunino, G. Raiola, F. Amadio, A. Ajoudani, and N. Tsagarakis, 'FollowMe: A Robust Person Following Framework Based on Visual Re-Identification and Gestures,' Proc. IEEE ARSO, Berlin, 2023, pp. 84–89. (Base Paper)",
    "[2] S. Holkar, V. Mane, P. Badhe, S. Harihar, and B. B. Deshmukh, 'Voice Activated Human Following Robot Using Computer Vision,' IJRASET, 2025, doi: 10.22214/ijraset.2025.70950.",
    "[3] M. Do Hoang, S.-S. Yun, and J.-S. Choi, 'The Reliable Recovery Mechanism for Person-Following Robot in Case of Missing Target,' Proc. IEEE URAI, Jeju, 2017, pp. 800–803.",
    "[4] M. Gupta, S. Kumar, L. Behera, and V. K. Subramanian, 'A Novel Vision-Based Tracking Algorithm for a Human-Following Mobile Robot,' IEEE Trans. Syst., Man, Cybern., Syst., vol. 47, no. 7, pp. 1415–1427, Jul. 2017.",
    "[5] V. Bazarevsky et al., 'BlazePose: On-device Real-time Body Pose tracking,' arXiv:2006.10204, 2020 (MediaPipe PoseLandmarker).",
    "[6] E. Olson, 'Real-time correlative scan matching,' Proc. IEEE ICRA, Shanghai, 2009, pp. 4387–4393 (M3RSM Multi-Resolution Search).",
    "[7] S. Kohlbrecher, O. von Stryk, J. Meyer, and U. Klingauf, 'A flexible and scalable SLAM system with full 3D motion estimation,' Proc. IEEE SSRR, 2011, pp. 155–160 (Hector SLAM).",
    "[8] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, 'Robot Operating System 2: Design, architecture, and uses in the wild,' Science Robotics, vol. 7, no. 66, 2022 (ROS 2 Humble).",
    "[9] ROBOTIS Co., Ltd., 'TurtleBot3 Burger & OpenCR 1.0 e-Manual,' ROBOTIS Documentation, 2024.",
    "[10] G. Bradski, 'The OpenCV Library,' Dr. Dobb's Journal of Software Tools, 2000 (Bhattacharyya Histogram & Image Processing).",
    "[11] R. E. Kalman, 'A New Approach to Linear Filtering and Prediction Problems,' Trans. ASME J. Basic Eng., vol. 82, pp. 35–45, 1960.",
]

tb16 = s16.shapes.add_textbox(Inches(0.55), Inches(1.25), Inches(12.23), Inches(5.75))
tf16 = tb16.text_frame
tf16.word_wrap = True
for idx, ref in enumerate(refs):
    p = tf16.paragraphs[0] if idx == 0 else tf16.add_paragraph()
    p.space_after = Pt(5.5)
    p.line_spacing = 1.12
    r = p.add_run()
    r.text = ref
    r.font.name = "Segoe UI"
    r.font.size = Pt(11.2)
    r.font.color.rgb = NAVY if idx == 0 else SLATE
    if idx == 0:
        r.font.bold = True

out_pptx = "Human_Following_Robot_Presentation.pptx"
prs.save(out_pptx)
print(f"Saved {out_pptx} with {len(prs.slides)} slides.")
