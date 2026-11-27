import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np

plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial"]
plt.rcParams["axes.edgecolor"] = "#CBD5E1"
plt.rcParams["axes.linewidth"] = 0.8

# ---------------------------------------------------------------------------
# 1. Distributed System Architecture Block Diagram
# ---------------------------------------------------------------------------
def make_architecture():
    fig, ax = plt.subplots(figsize=(11.5, 5.2), dpi=300)
    ax.set_xlim(0, 11.5)
    ax.set_ylim(0, 5.2)
    ax.axis("off")

    # Robot container
    robot_box = FancyBboxPatch((0.2, 0.35), 4.1, 4.45, boxstyle="round,pad=0.12,rounding_size=0.18",
                               facecolor="#F8FAFC", edgecolor="#0F4C81", linewidth=2)
    ax.add_patch(robot_box)
    ax.text(2.25, 4.58, "TURTLEBOT3 BURGER (ROBOT SIDE)", ha="center", va="center",
            fontsize=10.5, fontweight="bold", color="#0F4C81")

    # Laptop container
    lap_box = FancyBboxPatch((6.1, 0.35), 5.2, 4.45, boxstyle="round,pad=0.12,rounding_size=0.18",
                             facecolor="#F0F9FF", edgecolor="#0284C7", linewidth=2)
    ax.add_patch(lap_box)
    ax.text(8.7, 4.58, "LAPTOP EDGE-AI STATION (CONTROL HUB)", ha="center", va="center",
            fontsize=10.5, fontweight="bold", color="#0284C7")

    def draw_node(x, y, w, h, title, sub, fc="#FFFFFF", ec="#334155", tc="#0F172A"):
        b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06,rounding_size=0.1",
                           facecolor=fc, edgecolor=ec, linewidth=1.4)
        ax.add_patch(b)
        ax.text(x + w/2, y + h*0.64, title, ha="center", va="center", fontsize=8.8, fontweight="bold", color=tc)
        ax.text(x + w/2, y + h*0.28, sub, ha="center", va="center", fontsize=7.4, color="#475569")

    # Robot nodes
    draw_node(0.42, 3.35, 1.7, 0.85, "USB Camera", "640x480 @ 25 FPS\nHFOV 57 deg", fc="#E0F2FE", ec="#0369A1")
    draw_node(2.38, 3.35, 1.7, 0.85, "LDS-02 LiDAR", "360 deg /scan\n0.16 - 8.0 m", fc="#E0F2FE", ec="#0369A1")
    draw_node(0.65, 1.95, 3.2, 0.95, "Raspberry Pi 4 (ROS 2 Humble)", "pi_turtlebot_node.py | Domain 30\n0.35s Hardware Watchdog", fc="#DBEAFE", ec="#1D4ED8")
    draw_node(0.42, 0.60, 1.75, 0.90, "OpenCR 1.0", "STM32F746 + IMU\n/dev/ttyACM0", fc="#FEF3C7", ec="#B45309")
    draw_node(2.38, 0.60, 1.70, 0.90, "2x Dynamixel", "XL430-W250-T\n<= 0.22 m/s", fc="#FEE2E2", ec="#B91C1C")

    # Laptop nodes
    draw_node(6.32, 3.35, 2.25, 0.85, "VideoStreamer (UDP :5000)", "JPEG Reassembler + Scan RX\nNAT HELLO Handshake", fc="#FFFFFF", ec="#0284C7")
    draw_node(8.85, 3.35, 2.25, 0.85, "Perception & Re-ID", "MediaPipe 33-pt Pose\n18x16 HSV Multi-Template", fc="#DCFCE7", ec="#15803D")
    draw_node(6.32, 2.00, 2.25, 0.95, "EKF + Co-Mapping", "4-State Polar Fusion\nOnline Scale/Bias Calib.", fc="#DCFCE7", ec="#15803D")
    draw_node(8.85, 2.00, 2.25, 0.95, "LidarMap + A* Planner", "0.05m Grid + M3RSM SLAM\nPerson Memory & Detour", fc="#E0E7FF", ec="#4338CA")
    draw_node(6.32, 0.60, 2.25, 0.95, "16-State FSM & PID", "Follow / Wait / Recall / Search\nSlew-Limited RobotController", fc="#FEF9C3", ec="#A16207")
    draw_node(8.85, 0.60, 2.25, 0.95, "Safety & Web Dashboard", "Flask :8000 (MJPEG + SSE)\nLiDAR Hard Stop < 0.20 m", fc="#FCE7F3", ec="#BE185D")

    # Internal Robot arrows
    arrow_kw = dict(arrowstyle="-|>", lw=1.4, color="#334155", mutation_scale=11)
    ax.annotate("", xy=(1.27, 2.92), xytext=(1.27, 3.33), arrowprops=arrow_kw)
    ax.annotate("", xy=(3.23, 2.92), xytext=(3.23, 3.33), arrowprops=arrow_kw)
    ax.annotate("", xy=(1.30, 1.52), xytext=(1.30, 1.93), arrowprops=arrow_kw)
    ax.annotate("", xy=(2.36, 1.05), xytext=(2.19, 1.05), arrowprops=arrow_kw)

    # Network arrows
    net_rx = dict(arrowstyle="-|>", lw=2.0, color="#0284C7", mutation_scale=14)
    net_tx = dict(arrowstyle="-|>", lw=2.0, color="#DC2626", mutation_scale=14)
    ax.annotate("", xy=(6.28, 3.75), xytext=(3.88, 2.62), arrowprops=net_rx)
    ax.text(5.15, 3.55, "UDP :5000\nJPEG + /scan + Status", ha="center", va="center",
            fontsize=7.6, fontweight="bold", color="#0369A1",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#BAE6FD"))

    ax.annotate("", xy=(3.88, 2.20), xytext=(6.28, 1.05), arrowprops=net_tx)
    ax.text(5.15, 1.35, "UDP :8888 (30 Hz)\nV:vx, W:wz, S:seq", ha="center", va="center",
            fontsize=7.6, fontweight="bold", color="#B91C1C",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#FECACA"))

    # Internal Laptop arrows
    ax.annotate("", xy=(8.83, 3.77), xytext=(8.59, 3.77), arrowprops=arrow_kw)
    ax.annotate("", xy=(7.45, 2.97), xytext=(9.95, 3.33), arrowprops=arrow_kw)
    ax.annotate("", xy=(8.83, 2.47), xytext=(8.59, 2.47), arrowprops=arrow_kw)
    ax.annotate("", xy=(7.45, 1.57), xytext=(7.45, 1.98), arrowprops=arrow_kw)
    ax.annotate("", xy=(8.83, 1.07), xytext=(8.59, 1.07), arrowprops=arrow_kw)

    plt.tight_layout(pad=0.2)
    plt.savefig("build_assets/fig_architecture.png", dpi=300)
    plt.close()

# ---------------------------------------------------------------------------
# 2. Computational Latency Chart
# ---------------------------------------------------------------------------
def make_latency():
    stages = [
        "Owner Re-ID Match (9 templates)",
        "EKF Predict + Update",
        "HSV Signature Extraction",
        "LiDAR Sectors & Clustering",
        "A* Detour Plan (4 Hz thread)",
        "M3RSM + Gauss-Newton SLAM",
        "Full Per-Frame Pipeline (640x480)"
    ]
    med = [0.06, 0.08, 0.55, 0.69, 7.9, 10.7, 25.9]
    p95 = [0.09, 0.15, 0.98, 1.25, 9.2, 13.5, 28.5]

    y = np.arange(len(stages))
    fig, ax = plt.subplots(figsize=(8.6, 4.2), dpi=300)
    h = 0.36
    b1 = ax.barh(y - h/2, med, h, label="Median Latency (ms)", color="#0284C7")
    b2 = ax.barh(y + h/2, p95, h, label="95th Percentile (ms)", color="#94A3B8")
    ax.axvline(40.0, color="#DC2626", linestyle="--", linewidth=1.4, label="25 FPS Real-Time Budget (40 ms)")

    for bar in b1:
        w = bar.get_width()
        ax.text(w + 0.4, bar.get_y() + bar.get_height()/2, f"{w:.2f} ms" if w < 1 else f"{w:.1f} ms",
                va="center", fontsize=8, fontweight="bold", color="#0F172A")

    ax.set_yticks(y)
    ax.set_yticklabels(stages, fontsize=8.8)
    ax.set_xlabel("Execution Time (ms) — AMD Ryzen 7 6800HS (CPU Only)", fontsize=9.2, fontweight="bold")
    ax.set_title("Laptop Pipeline Per-Stage Computational Latency (~39 FPS Throughput)", fontsize=10.5, fontweight="bold", color="#0F172A")
    ax.set_xlim(0, 44.5)
    ax.legend(loc="lower right", fontsize=8.2)
    ax.grid(axis="x", linestyle=":", alpha=0.6)
    plt.tight_layout()
    plt.savefig("build_assets/fig_latency_bench.png", dpi=300)
    plt.close()

# ---------------------------------------------------------------------------
# 3. Range Error & Obstacle Clearance Benchmark
# ---------------------------------------------------------------------------
def make_accuracy_clearance():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.2, 4.0), dpi=300)

    # Panel 1: Range RMS error
    scenarios = ["Walk + Arc\n(Camera Only)", "Walk + Arc\n(LiDAR + Cam)", "Sharp 90° Turn\n(LiDAR + Cam)"]
    rms = [0.167, 0.119, 0.106]
    colors = ["#94A3B8", "#0284C7", "#059669"]
    bars = ax1.bar(scenarios, rms, color=colors, width=0.52, edgecolor="#1E293B", linewidth=0.8)
    for b in bars:
        val = b.get_height()
        ax1.text(b.get_x() + b.get_width()/2, val + 0.005, f"{val:.3f} m", ha="center", fontsize=9, fontweight="bold")
    ax1.set_ylabel("Range RMS Error (m)", fontsize=9.5, fontweight="bold")
    ax1.set_title("Closed-Loop Tracking Accuracy\n(29% Error Reduction via EKF Fusion)", fontsize=10, fontweight="bold")
    ax1.set_ylim(0, 0.21)
    ax1.grid(axis="y", linestyle=":", alpha=0.6)

    # Panel 2: Wall clearance before vs after
    labels = ["Before Fix\n(Mean Min)", "After Fix\n(Mean Min)"]
    mins = [0.08, 0.16]
    errs = [0.01, 0.01]
    b2 = ax2.bar(labels, mins, yerr=errs, capsize=6, color=["#EF4444", "#10B981"], width=0.45, edgecolor="#1E293B", linewidth=0.8)
    ax2.axhline(0.105, color="#B91C1C", linestyle="--", linewidth=1.4, label="TurtleBot3 Burger Radius (0.105 m)")
    for b in b2:
        val = b.get_height()
        ax2.text(b.get_x() + b.get_width()/2, val + 0.014, f"{val:.2f} m", ha="center", fontsize=9, fontweight="bold")
    ax2.set_ylabel("Minimum Wall-Corner Clearance (m)", fontsize=9.5, fontweight="bold")
    ax2.set_title("A* Detour Clearance Around Occluding Wall\n(Bubble-Core + Side Keep-Out Fix)", fontsize=10, fontweight="bold")
    ax2.set_ylim(0, 0.22)
    ax2.legend(loc="upper left", fontsize=8)
    ax2.grid(axis="y", linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig("build_assets/fig_accuracy_clearance.png", dpi=300)
    plt.close()

# ---------------------------------------------------------------------------
# 4. Follow-and-Wait Hysteresis Diagram
# ---------------------------------------------------------------------------
def make_hysteresis():
    fig, ax = plt.subplots(figsize=(8.2, 3.4), dpi=300)
    d = np.linspace(0.3, 2.2, 400)
    ax.axvspan(0.3, 0.55, color="#FEE2E2", alpha=0.8, label="Critical Proximity Stop (< 0.55 m)")
    ax.axvspan(0.55, 1.10, color="#FEF3C7", alpha=0.8, label="WAIT_FOR_OWNER_MOVEMENT (<= 1.10 m)")
    ax.axvspan(1.10, 1.35, color="#E0F2FE", alpha=0.8, label="25 cm Hysteresis Band (Hold State)")
    ax.axvspan(1.35, 2.20, color="#DCFCE7", alpha=0.8, label="FOLLOWING (>= 1.35 m for 3 frames)")

    # Velocity curve
    v = np.where(d <= 1.10, 0.0, np.clip(0.45 * (d - 1.20) + 0.08, 0.08, 0.22))
    ax.plot(d, v, color="#0F172A", linewidth=2.4, label="Commanded Linear Velocity v (m/s)")
    ax.axvline(1.20, color="#0284C7", linestyle="-.", linewidth=1.3, label="Desired Set-Point d* = 1.20 m")

    ax.set_xlabel("Estimated Owner Distance r (m)", fontsize=9.5, fontweight="bold")
    ax.set_ylabel("Linear Velocity v (m/s)", fontsize=9.5, fontweight="bold")
    ax.set_title("Follow-and-Wait Distance Control & Anti-Chatter Hysteresis", fontsize=10.5, fontweight="bold")
    ax.set_xlim(0.3, 2.2)
    ax.set_ylim(-0.01, 0.27)
    ax.legend(loc="upper left", fontsize=7.5, ncol=2)
    ax.grid(True, linestyle=":", alpha=0.5)
    plt.tight_layout()
    plt.savefig("build_assets/fig_hysteresis.png", dpi=300)
    plt.close()

# ---------------------------------------------------------------------------
# 5. State Machine Overview Diagram
# ---------------------------------------------------------------------------
def make_fsm():
    fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=300)
    ax.set_xlim(0, 10.5)
    ax.set_ylim(0, 4.8)
    ax.axis("off")

    def state(x, y, w, h, name, fc="#E0F2FE", ec="#0369A1", ls="-"):
        b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04,rounding_size=0.12",
                           facecolor=fc, edgecolor=ec, linewidth=1.5, linestyle=ls)
        ax.add_patch(b)
        ax.text(x + w/2, y + h/2, name, ha="center", va="center", fontsize=7.8, fontweight="bold", color="#0F172A")

    # Main behavior flow
    state(0.3, 3.6, 1.4, 0.65, "IDLE", fc="#F1F5F9", ec="#475569")
    state(2.2, 3.6, 1.8, 0.65, "OWNER_LOCKED", fc="#DBEAFE", ec="#1D4ED8")
    state(4.5, 3.6, 2.4, 0.65, "AWAITING_FOLLOW\n_GESTURE", fc="#E0F2FE", ec="#0284C7")
    state(7.5, 3.6, 1.9, 0.65, "FOLLOWING", fc="#DCFCE7", ec="#15803D")

    state(7.5, 2.1, 2.2, 0.70, "WAIT_FOR_OWNER\n_MOVEMENT", fc="#FEF3C7", ec="#B45309")
    state(4.8, 2.1, 1.8, 0.65, "PAUSED", fc="#FEF9C3", ec="#A16207")

    state(4.5, 0.65, 2.2, 0.65, "LIDAR_GUIDED\n_SEARCH (<=4s)", fc="#E0E7FF", ec="#4338CA")
    state(1.9, 0.65, 2.1, 0.65, "RECALLING_LAST\n_POSITION (<=25s)", fc="#EDE9FE", ec="#6D28D9")
    state(0.2, 1.85, 1.8, 0.65, "SEARCH_ROTATING\n(3-Phase 360°)", fc="#FCE7F3", ec="#BE185D")
    state(0.2, 0.65, 1.4, 0.65, "OWNER_LOST", fc="#FFE4E6", ec="#BE123C")
    state(2.2, 2.1, 1.8, 0.65, "OWNER_RELEASED", fc="#F1F5F9", ec="#64748B")

    # Overrides box
    ov_box = FancyBboxPatch((7.3, 0.35), 2.95, 1.4, boxstyle="round,pad=0.06,rounding_size=0.12",
                            facecolor="#FFF1F2", edgecolor="#E11D48", linewidth=1.4, linestyle="--")
    ax.add_patch(ov_box)
    ax.text(8.77, 1.52, "INDEPENDENT OVERRIDES", ha="center", va="center", fontsize=8, fontweight="bold", color="#BE123C")
    ax.text(8.77, 0.92, "• SAFETY_STOP (STOP_HALT / E-STOP)\n• SENSOR_FAULT (LiDAR >1.5s / Cam >0.8s)\n• MANUAL_CONTROL (Teleop)\n• TARGET_AMBIGUOUS (Δ < 0.08)",
            ha="center", va="center", fontsize=7.1, color="#881337")

    ak = dict(arrowstyle="-|>", lw=1.2, color="#334155", mutation_scale=10)
    ax.annotate("", xy=(2.18, 3.92), xytext=(1.72, 3.92), arrowprops=ak)
    ax.text(1.95, 4.10, "FOLLOW_ME", fontsize=6.5, ha="center")
    ax.annotate("", xy=(4.48, 3.92), xytext=(4.02, 3.92), arrowprops=ak)
    ax.text(4.25, 4.10, "Matched", fontsize=6.5, ha="center")
    ax.annotate("", xy=(7.48, 3.92), xytext=(6.92, 3.92), arrowprops=ak)
    ax.text(7.20, 4.10, "Hand Up", fontsize=6.5, ha="center")

    ax.annotate("", xy=(8.45, 2.82), xytext=(8.45, 3.58), arrowprops=ak)
    ax.text(9.05, 3.20, "r <= d_stop", fontsize=6.8, ha="center")
    ax.annotate("", xy=(8.10, 3.58), xytext=(8.10, 2.82), arrowprops=ak)
    ax.text(7.45, 3.20, "r >= d_resume\n(3 frames)", fontsize=6.4, ha="center")

    ax.annotate("", xy=(5.70, 2.77), xytext=(5.70, 3.58), arrowprops=ak)
    ax.text(6.15, 3.20, "WAIT/PAUSE", fontsize=6.5, ha="center")

    ax.annotate("", xy=(5.75, 1.32), xytext=(7.48, 3.60), arrowprops=dict(arrowstyle="-|>", lw=1.1, color="#4338CA", mutation_scale=10))
    ax.text(6.85, 2.65, "Cam Lost\n(LiDAR OK)", fontsize=6.5, color="#4338CA", ha="center")

    ax.annotate("", xy=(4.02, 0.97), xytext=(4.48, 0.97), arrowprops=ak)
    ax.annotate("", xy=(1.82, 1.83), xytext=(2.60, 1.32), arrowprops=ak)
    ax.annotate("", xy=(0.90, 1.32), xytext=(0.90, 1.83), arrowprops=ak)
    ax.text(1.45, 1.56, "360°/30s", fontsize=6.3, ha="center")

    ax.annotate("", xy=(1.00, 3.58), xytext=(2.50, 2.77), arrowprops=ak)
    ax.text(1.45, 3.05, "2s -> IDLE", fontsize=6.5, ha="center")

    plt.tight_layout(pad=0.15)
    plt.savefig("build_assets/fig_fsm.png", dpi=300)
    plt.close()

if __name__ == "__main__":
    make_architecture()
    make_latency()
    make_accuracy_clearance()
    make_hysteresis()
    make_fsm()
    print("Generated all 5 figures successfully.")
