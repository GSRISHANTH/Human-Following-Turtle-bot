/**
 * Complete Dashboard Frontend Controller for TurtleBot Human Following (Phase 2):
 * - Target Identification & Re-ID
 * - Dynamic Following Distance & Steering Alignment
 * - Gesture HRI Commands
 * - Live Motor Telemetry & Active Velocity Speedometers
 * - Full Teleop Control (Virtual D-pad + WASD keyboard keys)
 * - Hardware Emergency Stop interlock
 * - Real-Time Chronological Event Log
 */

document.addEventListener("DOMContentLoaded", () => {
    // Header & Connection
    const connDot = document.getElementById("conn-dot");
    const connText = document.getElementById("conn-text");
    const fpsVal = document.getElementById("fps-val");
    const safetyHeadVal = document.getElementById("safety-head-val");
    const robotModeVal = document.getElementById("robot-mode-val");
    const btnEstop = document.getElementById("btn-estop");

    // Video Overlays
    const targetPill = document.getElementById("target-pill");
    const targetPillText = document.getElementById("target-pill-text");
    const distanceBadge = document.getElementById("distance-badge");
    const offsetBadge = document.getElementById("offset-badge");
    const gestureOverlay = document.getElementById("gesture-overlay");

    // Target Identification & Owner Re-ID
    const targetColorSwatch = document.getElementById("target-color-swatch");
    const targetIdLabel = document.getElementById("target-id-label");
    const targetStateLabel = document.getElementById("target-state-label");
    const ownerStatusTag = document.getElementById("owner-status-tag");
    const peopleCountLabel = document.getElementById("people-count-label");
    const btnEnrollOwner = document.getElementById("btn-enroll-owner");
    const btnStartFollow = document.getElementById("btn-start-follow");
    const btnPauseFollow = document.getElementById("btn-pause-follow");
    const btnReleaseOwner = document.getElementById("btn-release-owner");
    const btnUnlockTarget = document.getElementById("btn-unlock-target");
    const btnClearOwner = document.getElementById("btn-clear-owner");

    // Dynamic Distance & Alignment
    const distNumber = document.getElementById("dist-number");
    const distStatusBadge = document.getElementById("dist-status-badge");
    const distMarker = document.getElementById("dist-marker");
    const labelDynDist = document.getElementById("label-dyn-dist");
    const alignAngleSub = document.getElementById("align-angle-sub");
    const horizonPointer = document.getElementById("horizon-pointer");

    // Motor Drive & Teleop Controls
    const btnModeAuto = document.getElementById("btn-mode-auto");
    const btnModeManual = document.getElementById("btn-mode-manual");
    const btnTestMotors = document.getElementById("btn-test-motors");
    const cmdLinear = document.getElementById("cmd-linear");
    const cmdAngular = document.getElementById("cmd-angular");
    const dpadContainer = document.getElementById("dpad-container");
    const dpadUp = document.getElementById("dpad-up");
    const dpadDown = document.getElementById("dpad-down");
    const dpadLeft = document.getElementById("dpad-left");
    const dpadRight = document.getElementById("dpad-right");
    const dpadCenter = document.getElementById("dpad-center");

    // Safety & Event Terminal
    const safetyBox = document.getElementById("safety-box");
    const safetyDot = document.getElementById("safety-dot");
    const safetyStatusTitle = document.getElementById("safety-status-title");
    const safetyAlerts = document.getElementById("safety-alerts");
    const eventTerminal = document.getElementById("event-terminal");
    const btnModeNetwork = document.getElementById("btn-mode-network");
    const btnModeSim = document.getElementById("btn-mode-sim");

    let lastRenderedEventsCount = 0;
    let currentMode = "AUTO";
    let modeClickPending = false;

    // --- Telemetry: pushed by the server as each pose result lands ---
    let latestOverlay = [];
    let overlayAt = 0;
    let lastPoseStamp = 0;

    function connectTelemetry() {
        const es = new EventSource("/api/stream");
        es.onmessage = (ev) => {
            let data;
            try { data = JSON.parse(ev.data); } catch (e) { return; }
            if (data.last_update !== lastPoseStamp) {
                lastPoseStamp = data.last_update;
                latestOverlay = data.overlay || [];
                overlayAt = performance.now();
            }
            updateUI(data);
        };
        es.onerror = () => {
            connDot.className = "status-dot offline";
            connText.textContent = "Dashboard server offline";
        };
    }

    // --- UI Update Renderer ---
    function updateUI(data) {
        if (!data) return;
        try {
            // 1. Connection & Header Stats
            if (connDot && connText) {
                if (data.connected) {
                    connDot.className = "status-dot online";
                    connText.textContent = data.source_mode === "sim" ? "Simulation Active" : "Pi Connected";
                } else {
                    connDot.className = "status-dot offline";
                    connText.textContent = "Pi Stream Offline";
                }
            }

            if (fpsVal) {
                fpsVal.textContent = data.fps > 0 ? `${Math.round(data.fps)}` : "--";
            }
            updateLinkHealth(data);
            updateFusion(data);

            // Robot Mode
            const rTel = data.robot_telemetry || {};
            const activeRobotMode = rTel.mode || currentMode;
            if (robotModeVal) {
                robotModeVal.textContent = rTel.emergency_stop ? "E-STOP" : activeRobotMode;
                if (rTel.emergency_stop) {
                    robotModeVal.className = "metric-val highlight-crimson";
                } else if (activeRobotMode === "AUTO") {
                    robotModeVal.className = "metric-val highlight-green";
                } else {
                    robotModeVal.className = "metric-val highlight";
                }
            }

            // 2. Target Identification & Owner Re-ID
            const targetId = data.target_id;
            const targetState = data.target_state || "SEARCHING";
            if (targetIdLabel) targetIdLabel.textContent = targetId ? `TARGET: #${targetId}` : "TARGET: NONE";
            if (targetStateLabel) targetStateLabel.textContent = `STATE: ${targetState}`;
            if (peopleCountLabel) peopleCountLabel.textContent = `People in view: ${data.people_count || 0}`;

            if (ownerStatusTag) {
                if (data.owner_enrolled) {
                    if (data.is_owner_matched) {
                        const pct = Math.round((data.owner_match_score || 1.0) * 100);
                        ownerStatusTag.textContent = `★ OWNER RECOGNIZED (${pct}% MATCH)`;
                        ownerStatusTag.style.color = "#00ff78";
                    } else {
                        ownerStatusTag.textContent = `★ OWNER ENROLLED (WAITING FOR OWNER)`;
                        ownerStatusTag.style.color = "#ffaa00";
                    }
                } else {
                    ownerStatusTag.textContent = "NOT ENROLLED (STANDBY)";
                    ownerStatusTag.style.color = "#8899aa";
                }
            }

            if (data.target_color && targetColorSwatch) {
                targetColorSwatch.style.background = data.target_color;
                targetColorSwatch.style.boxShadow = `0 0 12px ${data.target_color}`;
            }

            // Target Indicator Pill on Video
            if (targetPill && targetPillText) {
                if (data.fusion && data.fusion.lidar_only) {
                    targetPill.className = "target-indicator locked";
                    targetPillText.textContent = `★ OWNER (LIDAR) [${targetState}]`;
                } else if (data.owner_enrolled && data.is_owner_matched) {
                    targetPill.className = "target-indicator locked";
                    targetPillText.textContent = `★ OWNER #${targetId} [${targetState}]`;
                } else if (targetState === "TRACKING" || targetState === "REACQUIRED") {
                    targetPill.className = "target-indicator locked";
                    targetPillText.textContent = `TARGET #${targetId} [${targetState}]`;
                } else if (targetState === "OCCLUDED") {
                    targetPill.className = "target-indicator searching";
                    targetPillText.textContent = "OCCLUDED (PREDICTING)";
                } else if (targetState === "RECALLING_LAST_POSITION") {
                    targetPill.className = "target-indicator searching";
                    targetPillText.textContent = "RECALLING LAST POSITION (MAP)";
                } else if (targetState === "SEARCH_ROTATING") {
                    targetPill.className = "target-indicator searching";
                    targetPillText.textContent = "360° SEARCH ROTATING";
                } else {
                    targetPill.className = "target-indicator searching";
                    targetPillText.textContent = data.owner_enrolled ? "SEARCHING FOR OWNER" : "SEARCHING";
                }
            }

            // 3. Dynamic Distance & Alignment
            if (data.detected && data.distance_m > 0) {
                if (distNumber) distNumber.textContent = data.distance_m.toFixed(2);
                if (distanceBadge) distanceBadge.textContent = `${data.distance_m.toFixed(2)} m`;
                
                const clampedDist = Math.max(0.4, Math.min(3.0, data.distance_m));
                const pctDist = ((clampedDist - 0.4) / (3.0 - 0.4)) * 100;
                if (distMarker) distMarker.style.left = `${pctDist}%`;

                const dynTarget = data.dynamic_target_distance || 1.3;
                if (labelDynDist) labelDynDist.textContent = `Dynamic: ${dynTarget.toFixed(2)}m`;

                if (distStatusBadge) {
                    if (data.distance_m < 0.65) {
                        distStatusBadge.textContent = "TOO CLOSE";
                        distStatusBadge.className = "dist-status-badge too-close";
                    } else if (Math.abs(data.distance_m - dynTarget) < 0.20) {
                        distStatusBadge.textContent = "OPTIMAL";
                        distStatusBadge.className = "dist-status-badge";
                    } else if (data.distance_m > dynTarget + 0.5) {
                        distStatusBadge.textContent = "FAR";
                        distStatusBadge.className = "dist-status-badge";
                    } else {
                        distStatusBadge.textContent = "TRACKING";
                        distStatusBadge.className = "dist-status-badge";
                    }
                }

                const offDeg = data.offset_deg || 0.0;
                if (offsetBadge) offsetBadge.textContent = `OFFSET: ${offDeg > 0 ? '+' : ''}${offDeg.toFixed(1)}°`;
                if (alignAngleSub) alignAngleSub.textContent = `OFFSET: ${offDeg > 0 ? '+' : ''}${offDeg.toFixed(1)}°`;

                const normX = Math.max(-1.0, Math.min(1.0, data.offset_x_norm || 0.0));
                const pctHorizon = (normX + 1.0) * 50;
                if (horizonPointer) horizonPointer.style.left = `${pctHorizon}%`;
            } else {
                if (distNumber) distNumber.textContent = "--";
                if (distanceBadge) distanceBadge.textContent = "-- m";
                if (distStatusBadge) {
                    distStatusBadge.textContent = targetState;
                    distStatusBadge.className = "dist-status-badge searching";
                }
                if (distMarker) distMarker.style.left = "0%";
                if (offsetBadge) offsetBadge.textContent = "OFFSET: 0.0°";
                if (alignAngleSub) alignAngleSub.textContent = "OFFSET: 0.0°";
                if (horizonPointer) horizonPointer.style.left = "50%";
            }

            // Gesture Overlay
            const gesture = data.active_gesture || "NONE";
            if (gestureOverlay) {
                gestureOverlay.textContent = `GESTURE: ${gesture}`;
                if (gesture !== "NONE") {
                    gestureOverlay.style.borderColor = "var(--neon-green)";
                    gestureOverlay.style.color = "var(--neon-green)";
                } else {
                    gestureOverlay.style.borderColor = "rgba(255, 255, 255, 0.1)";
                    gestureOverlay.style.color = "var(--text-primary)";
                }
            }

            // 4. Live Motor Outputs
            const liveLin = rTel.current_linear !== undefined ? rTel.current_linear : (data.phase2_cmd_vel?.linear_x || 0.0);
            const liveAng = rTel.current_angular !== undefined ? rTel.current_angular : (data.phase2_cmd_vel?.angular_z || 0.0);
            if (cmdLinear) cmdLinear.textContent = `${liveLin >= 0 ? '+' : ''}${liveLin.toFixed(2)} m/s`;
            if (cmdAngular) cmdAngular.textContent = `${liveAng >= 0 ? '+' : ''}${liveAng.toFixed(2)} rad/s`;

            // 5. Safety Layer
            const safetyStatus = data.safety_status || "SAFE";
            if (safetyHeadVal) safetyHeadVal.textContent = safetyStatus;
            if (safetyStatusTitle) safetyStatusTitle.textContent = `STATUS: ${safetyStatus}`;

            if (safetyBox) {
                safetyBox.className = "safety-box";
                if (safetyStatus.includes("STOP") || safetyStatus.includes("CRITICAL")) {
                    safetyBox.classList.add("danger");
                    if (safetyHeadVal) safetyHeadVal.className = "metric-val highlight-crimson";
                    if (safetyDot) {
                        safetyDot.style.background = "var(--neon-crimson)";
                        safetyDot.style.boxShadow = "0 0 8px var(--neon-crimson)";
                    }
                } else if (safetyStatus.includes("WARNING") || safetyStatus.includes("CAUTION") || safetyStatus.includes("LOST")) {
                    safetyBox.classList.add("warning");
                    if (safetyHeadVal) safetyHeadVal.className = "metric-val highlight-amber";
                    if (safetyDot) {
                        safetyDot.style.background = "var(--neon-amber)";
                        safetyDot.style.boxShadow = "0 0 8px var(--neon-amber)";
                    }
                } else {
                    if (safetyHeadVal) safetyHeadVal.className = "metric-val highlight-green";
                    if (safetyDot) {
                        safetyDot.style.background = "var(--neon-green)";
                        safetyDot.style.boxShadow = "0 0 8px var(--neon-green)";
                    }
                }
            }

            const alerts = data.safety_alerts || [];
            if (safetyAlerts) {
                safetyAlerts.textContent = alerts.length > 0 ? alerts.join(" • ") : "Nominal. All zones clear.";
            }

            // 6. Event Terminal
            if (eventTerminal && data.events && data.events.length > 0 && data.events.length !== lastRenderedEventsCount) {
                lastRenderedEventsCount = data.events.length;
                eventTerminal.innerHTML = "";
                data.events.forEach(evt => {
                    const el = document.createElement("div");
                    el.className = `log-entry ${getEventClass(evt.type)}`;
                    el.textContent = `[${evt.time}] ${evt.type}: ${evt.message}`;
                    eventTerminal.appendChild(el);
                });
                eventTerminal.scrollTop = eventTerminal.scrollHeight;
            }
        } catch (err) {
            console.error("updateUI error:", err);
        }
    }

    // --- Link health: video FPS, pose FPS, age, bitrate, and what the Pi reports about itself ---
    const setText = (id, text, cls) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.textContent = text;
        if (cls !== undefined) el.className = "lh-val " + cls;
    };
    const fpsHistory = [];
    const spark = document.getElementById("fps-spark");

    function updateLinkHealth(d) {
        const pi = d.pi || {};
        const age = d.frame_age_ms;
        setText("lh-fps", d.fps > 0 ? d.fps.toFixed(0) : "--", d.fps >= 20 ? "ok" : d.fps > 8 ? "warn" : "bad");
        setText("lh-pose", d.pose_fps > 0 ? d.pose_fps.toFixed(0) : "--", d.pose_fps >= 12 ? "ok" : d.pose_fps > 5 ? "warn" : "bad");
        const poseHead = document.getElementById("lh-pose-head");
        if (poseHead) poseHead.textContent = d.pose_fps > 0 ? Math.round(d.pose_fps) : "--";
        setText("lh-age", age == null ? "--" : `${age} ms`, age == null ? "" : age < 150 ? "ok" : age < 500 ? "warn" : "bad");
        setText("lh-kbps", d.kbps ? `${(d.kbps / 1000).toFixed(1)} Mb/s` : "--");
        setText("lh-drop", `${d.dropped || 0}`, (d.dropped || 0) > 50 ? "warn" : "");
        const havePi = Object.keys(pi).length > 0;
        setText("lh-pi", havePi ? "ONLINE" : (d.source_mode === "sim" ? "SIM" : "SILENT"), havePi ? "ok" : d.source_mode === "sim" ? "warn" : "bad");
        setText("lh-ros", havePi ? (pi.ros || (pi.serial ? "SERIAL" : "NONE")) : "--", havePi && (pi.ros || pi.serial) ? "ok" : "warn");
        setText("lh-subs", havePi ? `${pi.subs}` : "--", havePi && pi.subs > 0 ? "ok" : "warn");
        setText("lh-motors", havePi ? (!pi.motors ? "OFF (no-motors)" : pi.torque ? "TORQUE ON" : "TORQUE OFF") : "--", havePi && pi.motors && pi.torque ? "ok" : "warn");
        setText("lh-cmdage", havePi && pi.cmd_age != null ? `${(pi.cmd_age * 1000).toFixed(0)} ms` : "--", havePi && pi.cmd_age != null && pi.cmd_age < 0.2 ? "ok" : "warn");
        setText("lh-picam", havePi ? `${pi.cam_fps} fps${pi.raw ? " raw" : ""}` : "--");

        fpsHistory.push(d.fps || 0);
        if (fpsHistory.length > 90) fpsHistory.shift();
        if (spark) {
            const c = spark.getContext("2d");
            const w = spark.width, h = spark.height;
            c.clearRect(0, 0, w, h);
            c.strokeStyle = "#00f0ff";
            c.lineWidth = 1.5;
            c.beginPath();
            fpsHistory.forEach((v, i) => {
                const x = (i / 89) * w, y = h - Math.min(v, 35) / 35 * (h - 2) - 1;
                if (i) c.lineTo(x, y); else c.moveTo(x, y);
            });
            c.stroke();
        }

        // Mode buttons follow what the robot controller actually reports.
        const m = (d.robot_telemetry || {}).mode;
        if (m && !modeClickPending) {
            currentMode = m;
            btnModeAuto.classList.toggle("active", m === "AUTO");
            btnModeManual.classList.toggle("active", m === "MANUAL");
        }
    }

    // --- LiDAR sectors, fusion source, EKF readout ---
    function updateFusion(d) {
        const f = d.fusion || {};
        const sec = f.sectors;
        [["left", "sec-left"], ["front", "sec-front"], ["right", "sec-right"]].forEach(([k, id]) => {
            const el = document.getElementById(id);
            if (!el) return;
            const v = sec ? sec[k] : null;
            el.textContent = v == null ? "--" : `${v.toFixed(2)} m`;
            el.className = v == null ? "" : v < 0.35 ? "bad" : v < 0.85 ? "warn" : "";
        });
        setText("fu-mode", f.mode || "--", f.mode === "LIDAR+CAM" ? "ok" : f.mode === "CAM" ? "warn" : "");
        const nav = d.nav_status || "--";
        setText("fu-nav", nav, nav.includes("STOP") ? "bad" : nav.includes("AVOID") || nav.includes("SIDE") ? "warn" : "ok");
        const p = d.person;
        setText("fu-speed", p ? `${p.speed.toFixed(2)} m/s` : "--");
        setText("fu-head", p && p.speed > 0.05 ? `${(Math.atan2(p.vy, p.vx) * 180 / Math.PI).toFixed(0)}\u00b0` : "--");
        setText("fu-scale", f.vision_scale ? `x${f.vision_scale.toFixed(2)}  bearing ${f.bearing_bias_deg >= 0 ? "+" : ""}${f.bearing_bias_deg}°` : "--");
        const loc = d.localization;
        setText("loc-pose", loc ? `${loc.x.toFixed(2)}, ${loc.y.toFixed(2)} m  ${loc.yaw.toFixed(0)}\u00b0` : "no odometry");
        setText("loc-q", loc ? (loc.scans < 25 ? `mapping ${loc.scans}/25` : `match ${loc.score.toFixed(2)}  drift fixed ${loc.shift.toFixed(2)} m`) : "--", loc && loc.scans >= 25 && loc.score > 0.3 ? "ok" : "warn");
        const ps = d.plan_status || "--";
        setText("plan-status", ps, ps.includes("DETOUR") || ps.startsWith("MEMORY") ? "warn" : ps.includes("NO_PATH") ? "bad" : "ok");
        setText("loc-moving", loc ? `${loc.moving}` : "--");

        const r = d.reid;
        const rd = document.getElementById("reid-detail"), rf = document.getElementById("reid-fill"), rm = document.getElementById("reid-mark");
        if (rd && rf && rm) {
            if (!r) {
                rd.textContent = "Identity: no owner enrolled.";
                rf.style.width = "0";
            } else {
                const label = { TRACKING: "owner tracked", REACQUIRING: `re-checking ${r.reacq}/${r.confirm}`, REACQUIRED: "owner re-acquired", REJECTED: "not the owner" }[r.state] || r.state;
                rd.textContent = `Identity: ${label} | match ${r.best.toFixed(2)} | needs ${r.thr.toFixed(2)} | owner usually ${r.own.toFixed(2)}`;
                rf.style.width = `${Math.min(100, r.best * 100)}%`;
                rf.className = "reid-fill " + (r.best >= r.thr ? "" : r.state === "REJECTED" ? "bad" : "warn");
                rm.style.left = `${Math.min(100, r.thr * 100)}%`;
            }
        }
        const pi = d.pi || {};
        setText("lh-scan", pi.scan_hz != null ? (pi.scan_hz > 0 ? `${pi.scan_hz.toFixed(0)} Hz` : "none") : "--", pi.scan_hz > 3 ? "ok" : "warn");
    }

    // Follow distance: the robot waits at this range and resumes when the person moves 15 cm beyond it
    const followDist = document.getElementById("follow-dist");
    const followDistVal = document.getElementById("follow-dist-val");
    if (followDist) {
        let editing = false;
        fetch("/api/config").then((r) => r.json()).then((c) => {
            if (c.target_distance) { followDist.value = c.target_distance; followDistVal.textContent = `${Number(c.target_distance).toFixed(2)} m`; }
        }).catch(() => {});
        followDist.addEventListener("input", () => { editing = true; followDistVal.textContent = `${Number(followDist.value).toFixed(2)} m`; });
        followDist.addEventListener("change", async () => {
            await fetch("/api/config", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ target_distance: Number(followDist.value) }) });
            editing = false;
        });
    }

    const btnLidarReset = document.getElementById("btn-lidar-reset");
    if (btnLidarReset) btnLidarReset.addEventListener("click", () => fetch("/api/lidar/reset", { method: "POST" }));

    function getEventClass(type) {
        if (!type) return "system";
        const t = type.toLowerCase();
        if (t.includes("lock")) return "target_locked";
        if (t.includes("gesture")) return "gesture";
        if (t.includes("recovery") || t.includes("reacquired") || t.includes("rotating")) return "recovery";
        if (t.includes("stop") || t.includes("safety") || t.includes("hazard")) return "safety";
        return "system";
    }

    // --- Interactive Action Handlers ---

    async function setMode(mode) {
        modeClickPending = true;
        currentMode = mode;
        btnModeAuto.classList.toggle("active", mode === "AUTO");
        btnModeManual.classList.toggle("active", mode === "MANUAL");
        try {
            await fetch("/api/robot_mode", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ mode })
            });
        } catch (err) {}
        setTimeout(() => { modeClickPending = false; }, 400);
    }

    // Emergency Stop Button
    btnEstop.addEventListener("click", async () => {
        try {
            await fetch("/api/estop", { method: "POST" });
            btnModeAuto.classList.remove("active");
            btnModeManual.classList.remove("active");
        } catch (err) {
            console.error("E-Stop error:", err);
        }
    });

    // Robot Mode: Auto Follow
    btnModeAuto.addEventListener("click", async () => {
        await setMode("AUTO");
    });

    // Robot Mode: Manual Teleop
    btnModeManual.addEventListener("click", async () => {
        await setMode("MANUAL");
    });

    // Enroll / Recognize Me as Owner
    if (btnEnrollOwner) {
        btnEnrollOwner.addEventListener("click", async () => {
            btnEnrollOwner.textContent = "ENROLLING...";
            btnEnrollOwner.style.borderColor = "#00ffea";
            try {
                const res = await fetch("/api/enroll_owner", { method: "POST" });
                const json = await res.json();
                if (json.status === "success") {
                    btnEnrollOwner.textContent = "★ OWNER ENROLLED!";
                    setTimeout(() => {
                        btnEnrollOwner.textContent = "★ RE-ENROLL / UPDATE OWNER";
                        btnEnrollOwner.style.borderColor = "#00ff88";
                    }, 2000);
                } else {
                    btnEnrollOwner.textContent = "STAND IN VIEW & RETRY";
                    setTimeout(() => {
                        btnEnrollOwner.textContent = "★ ENROLL / RECOGNIZE ME AS OWNER";
                    }, 2000);
                }
            } catch (err) {
                console.error("Enroll error:", err);
                btnEnrollOwner.textContent = "★ ENROLL / RECOGNIZE ME AS OWNER";
            }
        });
    }

    // Start Following Button
    if (btnStartFollow) {
        btnStartFollow.addEventListener("click", async () => {
            try {
                await fetch("/api/start_following", { method: "POST" });
                btnStartFollow.textContent = "▶ FOLLOWING ACTIVE";
                setTimeout(() => { btnStartFollow.textContent = "▶ START FOLLOWING"; }, 2000);
            } catch (err) {
                console.error("Start follow error:", err);
            }
        });
    }

    // Pause Following Button
    if (btnPauseFollow) {
        btnPauseFollow.addEventListener("click", async () => {
            try {
                await fetch("/api/pause_following", { method: "POST" });
                btnPauseFollow.textContent = "⏸ PAUSED";
                setTimeout(() => { btnPauseFollow.textContent = "⏸ PAUSE"; }, 2000);
            } catch (err) {
                console.error("Pause follow error:", err);
            }
        });
    }

    // Release Owner Button
    if (btnReleaseOwner) {
        btnReleaseOwner.addEventListener("click", async () => {
            try {
                await fetch("/api/release_owner", { method: "POST" });
                if (btnEnrollOwner) btnEnrollOwner.textContent = "★ LOCK / ENROLL OWNER";
            } catch (err) {
                console.error("Release owner error:", err);
            }
        });
    }

    // Unlock / Reselect Target
    if (btnUnlockTarget) {
        btnUnlockTarget.addEventListener("click", async () => {
            try {
                await fetch("/api/unlock_target", { method: "POST" });
            } catch (err) {
                console.error("Unlock error:", err);
            }
        });
    }

    // Clear Owner Profile
    if (btnClearOwner) {
        btnClearOwner.addEventListener("click", async () => {
            try {
                await fetch("/api/clear_owner", { method: "POST" });
                if (btnEnrollOwner) btnEnrollOwner.textContent = "★ LOCK / ENROLL OWNER";
            } catch (err) {
                console.error("Clear owner error:", err);
            }
        });
    }

    // Test Motors Button
    if (btnTestMotors) {
        btnTestMotors.addEventListener("click", async () => {
            btnTestMotors.textContent = "TESTING...";
            btnTestMotors.style.borderColor = "var(--neon-crimson)";
            try {
                await fetch("/api/test_motors", { method: "POST" });
            } catch (err) {
                console.error("Test motor error:", err);
            }
            setTimeout(() => {
                btnTestMotors.textContent = "TEST MOTORS";
                btnTestMotors.style.borderColor = "var(--neon-amber)";
            }, 1500);
        });
    }

    // Teleop. The Pi stops the wheels 0.6 s after the last packet, so a held input must keep sending.
    const SPEED = { fwd: 0.20, back: -0.12, turn: 1.20 };
    const heldKeys = new Set();
    let padDir = null;
    let teleopTimer = null;
    let lastSent = [0, 0];

    function teleopVector() {
        const dirs = new Set(heldKeys);
        if (padDir) dirs.add(padDir);
        let lin = 0, ang = 0;
        if (dirs.has("up")) lin += SPEED.fwd;
        if (dirs.has("down")) lin += SPEED.back;
        if (dirs.has("left")) ang += SPEED.turn;
        if (dirs.has("right")) ang -= SPEED.turn;
        return [lin, ang];
    }

    async function pushTeleop(lin, ang) {
        lastSent = [lin, ang];
        try {
            await fetch("/api/teleop", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ linear: lin, angular: ang })
            });
        } catch (err) {
            console.error("Teleop error:", err);
        }
    }

    async function teleopChanged() {
        const [lin, ang] = teleopVector();
        const active = lin !== 0 || ang !== 0;
        if (active && currentMode !== "MANUAL") await setMode("MANUAL");
        if (active && !teleopTimer) {
            teleopTimer = setInterval(() => {
                const [l, a] = teleopVector();
                if (l !== 0 || a !== 0) pushTeleop(l, a);
            }, 100);
        }
        if (!active && teleopTimer) {
            clearInterval(teleopTimer);
            teleopTimer = null;
        }
        if (active || lastSent[0] !== 0 || lastSent[1] !== 0) pushTeleop(lin, ang);
    }

    const padButtons = { up: dpadUp, down: dpadDown, left: dpadLeft, right: dpadRight };
    Object.entries(padButtons).forEach(([dir, btn]) => {
        btn.addEventListener("pointerdown", (e) => {
            btn.setPointerCapture(e.pointerId);
            padDir = dir;
            teleopChanged();
        });
        const release = () => { if (padDir === dir) { padDir = null; teleopChanged(); } };
        btn.addEventListener("pointerup", release);
        btn.addEventListener("pointercancel", release);
        btn.addEventListener("lostpointercapture", release);
    });
    dpadCenter.addEventListener("click", () => {
        heldKeys.clear();
        padDir = null;
        teleopChanged();
    });

    const KEYMAP = { w: "up", arrowup: "up", s: "down", arrowdown: "down", a: "left", arrowleft: "left", d: "right", arrowright: "right" };
    window.addEventListener("keydown", (e) => {
        if (e.target.tagName === "INPUT") return;
        const key = e.key.toLowerCase();
        if (key === " " || key === "spacebar") {
            btnEstop.click();
            e.preventDefault();
        } else if (KEYMAP[key]) {
            e.preventDefault();
            if (!heldKeys.has(KEYMAP[key])) {
                heldKeys.add(KEYMAP[key]);
                teleopChanged();
            }
        }
    });
    window.addEventListener("keyup", (e) => {
        const key = e.key.toLowerCase();
        if (KEYMAP[key] && heldKeys.delete(KEYMAP[key])) teleopChanged();
    });
    window.addEventListener("blur", () => {
        if (heldKeys.size || padDir) { heldKeys.clear(); padDir = null; teleopChanged(); }
    });

    // Mode Toggle (Pi vs Sim)
    btnModeNetwork.addEventListener("click", async () => {
        btnModeNetwork.classList.add("active");
        btnModeSim.classList.remove("active");
        await fetch("/api/config", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ source_mode: "network" })
        });
    });

    btnModeSim.addEventListener("click", async () => {
        btnModeSim.classList.add("active");
        btnModeNetwork.classList.remove("active");
        await fetch("/api/config", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ source_mode: "sim" })
        });
    });

    // Live view. The server forwards raw camera JPEGs; the pose is drawn here, so nothing is re-encoded.
    // Reading the whole stream and keeping only the newest JPEG drops any backlog instead of playing it late.
    const canvas = document.getElementById("live-canvas");
    const ctx2d = canvas ? canvas.getContext("2d") : null;
    let pendingBitmap = null;
    let shownBitmap = null;
    const LINKS = [[11, 12], [11, 23], [12, 24], [23, 24], [11, 13], [13, 15], [12, 14], [14, 16],
                   [23, 25], [25, 27], [24, 26], [26, 28], [0, 1], [1, 2], [2, 3], [3, 7], [0, 4], [4, 5], [5, 6], [6, 8]];

    function drawOverlay(w, h) {
        if (performance.now() - overlayAt > 900) return;
        latestOverlay.forEach((p) => {
            const pts = {};
            (p.pose || []).forEach((it) => { if (it.length >= 4) pts[it[0]] = it; });
            ctx2d.lineWidth = 2;
            ctx2d.strokeStyle = "#00f0ff";
            LINKS.forEach(([a, b]) => {
                const pa = pts[a], pb = pts[b];
                if (!pa || !pb || pa[3] < 0.35 || pb[3] < 0.35) return;
                ctx2d.beginPath();
                ctx2d.moveTo(pa[1] * w, pa[2] * h);
                ctx2d.lineTo(pb[1] * w, pb[2] * h);
                ctx2d.stroke();
            });
            if (p.bbox && p.bbox.length === 4) {
                ctx2d.strokeStyle = p.target ? "#50dc3c" : "#ffb703";
                ctx2d.lineWidth = p.target ? 3 : 2;
                ctx2d.strokeRect(p.bbox[0] * w, p.bbox[1] * h, (p.bbox[2] - p.bbox[0]) * w, (p.bbox[3] - p.bbox[1]) * h);
            }
        });
    }

    function paint() {
        if (pendingBitmap) {
            if (shownBitmap) shownBitmap.close();
            shownBitmap = pendingBitmap;
            pendingBitmap = null;
        }
        if (shownBitmap && ctx2d) {
            if (canvas.width !== shownBitmap.width || canvas.height !== shownBitmap.height) {
                canvas.width = shownBitmap.width;
                canvas.height = shownBitmap.height;
            }
            ctx2d.drawImage(shownBitmap, 0, 0);
            drawOverlay(canvas.width, canvas.height);
        }
        requestAnimationFrame(paint);
    }

    async function pumpLive() {
        while (true) {
            try {
                const res = await fetch("/stream/video", { cache: "no-store" });
                const reader = res.body.getReader();
                let buf = new Uint8Array(0);
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) break;
                    const merged = new Uint8Array(buf.length + value.length);
                    merged.set(buf);
                    merged.set(value, buf.length);
                    buf = merged.length > 400000 ? merged.slice(merged.length - 250000) : merged;
                    let end = -1;
                    for (let i = buf.length - 2; i >= 1; i--) {
                        if (buf[i] === 0xFF && buf[i + 1] === 0xD9) { end = i; break; }
                    }
                    if (end < 0) continue;
                    let start = -1;
                    for (let i = end - 1; i >= 0; i--) {
                        if (buf[i] === 0xFF && buf[i + 1] === 0xD8) { start = i; break; }
                    }
                    if (start < 0) continue;
                    const jpeg = buf.slice(start, end + 2);
                    buf = buf.slice(end + 2);
                    createImageBitmap(new Blob([jpeg], { type: "image/jpeg" })).then((bmp) => {
                        if (pendingBitmap) pendingBitmap.close();
                        pendingBitmap = bmp;
                    }).catch(() => {});
                }
            } catch (err) {
                await new Promise((resolve) => setTimeout(resolve, 400));
            }
        }
    }
    if (canvas) {
        requestAnimationFrame(paint);
        pumpLive();
    }

    connectTelemetry();
});
