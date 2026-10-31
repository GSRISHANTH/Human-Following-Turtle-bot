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
    let isPolling = true;
    let currentMode = "AUTO";

    // --- Telemetry Poller Loop (10Hz) ---
    async function fetchTelemetry() {
        if (!isPolling) return;
        try {
            const res = await fetch("/api/telemetry");
            if (res.ok) {
                const data = await res.json();
                updateUI(data);
            }
        } catch (err) {
            console.warn("Telemetry fetch error:", err);
            connDot.className = "status-dot offline";
            connText.textContent = "Server Offline";
        }
        setTimeout(fetchTelemetry, 150);
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
                fpsVal.textContent = data.fps > 0 ? `${data.fps}` : "--";
            }

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
                if (data.owner_enrolled && data.is_owner_matched) {
                    targetPill.className = "target-indicator locked";
                    targetPillText.textContent = `★ OWNER #${targetId} [${targetState}]`;
                } else if (targetState === "TRACKING" || targetState === "REACQUIRED") {
                    targetPill.className = "target-indicator locked";
                    targetPillText.textContent = `TARGET #${targetId} [${targetState}]`;
                } else if (targetState === "OCCLUDED") {
                    targetPill.className = "target-indicator searching";
                    targetPillText.textContent = "OCCLUDED (PREDICTING)";
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
        currentMode = "AUTO";
        btnModeAuto.classList.add("active");
        btnModeManual.classList.remove("active");
        await fetch("/api/robot_mode", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ mode: "AUTO" })
        });
    });

    // Robot Mode: Manual Teleop
    btnModeManual.addEventListener("click", async () => {
        currentMode = "MANUAL";
        btnModeManual.classList.add("active");
        btnModeAuto.classList.remove("active");
        await fetch("/api/robot_mode", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ mode: "MANUAL" })
        });
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

    // D-Pad Helper Function
    async function sendTeleop(lin, ang) {
        // Auto-switch to MANUAL mode if user touches controls
        if (currentMode !== "MANUAL" && (lin !== 0 || ang !== 0)) {
            currentMode = "MANUAL";
            btnModeManual.classList.add("active");
            btnModeAuto.classList.remove("active");
            try {
                await fetch("/api/robot_mode", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ mode: "MANUAL" })
                });
            } catch (err) {}
        }
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

    // Virtual D-Pad Click Events
    dpadUp.addEventListener("mousedown", () => sendTeleop(0.18, 0.0));
    dpadDown.addEventListener("mousedown", () => sendTeleop(-0.12, 0.0));
    dpadLeft.addEventListener("mousedown", () => sendTeleop(0.0, 0.70));
    dpadRight.addEventListener("mousedown", () => sendTeleop(0.0, -0.70));
    dpadCenter.addEventListener("mousedown", () => sendTeleop(0.0, 0.0));

    // Stop motion on release
    [dpadUp, dpadDown, dpadLeft, dpadRight].forEach(btn => {
        btn.addEventListener("mouseup", () => sendTeleop(0.0, 0.0));
        btn.addEventListener("mouseleave", () => sendTeleop(0.0, 0.0));
    });

    // Keyboard Teleop Controls (WASD / Arrows / Space)
    window.addEventListener("keydown", (e) => {
        if (e.target.tagName === "INPUT") return;
        const key = e.key.toLowerCase();
        if (key === " " || key === "spacebar") {
            btnEstop.click();
            e.preventDefault();
        } else if (key === "w" || key === "arrowup") {
            sendTeleop(0.18, 0.0);
            e.preventDefault();
        } else if (key === "s" || key === "arrowdown") {
            sendTeleop(-0.12, 0.0);
            e.preventDefault();
        } else if (key === "a" || key === "arrowleft") {
            sendTeleop(0.0, 0.70);
            e.preventDefault();
        } else if (key === "d" || key === "arrowright") {
            sendTeleop(0.0, -0.70);
            e.preventDefault();
        }
    });

    window.addEventListener("keyup", (e) => {
        if (e.target.tagName === "INPUT") return;
        const key = e.key.toLowerCase();
        if (["w", "s", "a", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(key)) {
            sendTeleop(0.0, 0.0);
        }
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

    // Start background telemetry polling
    fetchTelemetry();
});
