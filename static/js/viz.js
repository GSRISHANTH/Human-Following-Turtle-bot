/**
 * Dashboard visualisations (no libraries):
 *  - drawOverlay():  LiDAR scan projected into the camera image (co-mapping), skeletons, person boxes with
 *                    camera range | LiDAR range | Re-ID match
 *  - TimeChart:      rolling 30 s line charts with a hover crosshair + tooltip (one y-axis per chart)
 *  - FSM:            the follow state machine, current state highlighted, with why it is there
 *  - Settings:       live tuning form backed by /api/config
 * Palette (validated on the #121826 card surface, dark mode): actual #3987e5, commanded #d95926 (dashed).
 */
(function () {
    "use strict";
    const INK = { primary: "#f0efec", secondary: "#c3c2b7", muted: "#898781", grid: "#2c2c2a", axis: "#383835" };
    const SERIES = { actual: "#3987e5", commanded: "#d95926" };
    const STATUS = { good: "#0ca30c", warning: "#fab219", serious: "#ec835a", critical: "#d03b3b" };

    // ---------- co-mapped camera overlay ----------
    // Range is a magnitude: one hue, bright (near) -> dim (far). Owner beams carry identity: green and larger.
    function rangeColor(r) {
        const t = Math.max(0, Math.min(1, (r - 0.3) / 3.7));
        const near = [159, 240, 216], far = [24, 96, 82];
        const c = near.map((n, i) => Math.round(n + (far[i] - n) * t));
        return `rgb(${c[0]},${c[1]},${c[2]})`;
    }
    const LINKS = [[11, 12], [11, 23], [12, 24], [23, 24], [11, 13], [13, 15], [12, 14], [14, 16],
                   [23, 25], [25, 27], [24, 26], [26, 28], [0, 1], [1, 2], [2, 3], [3, 7], [0, 4], [4, 5], [5, 6], [6, 8]];

    function label(ctx, text, x, y, fg, bg) {
        ctx.font = "600 12px 'JetBrains Mono', monospace";
        const w = ctx.measureText(text).width + 10;
        ctx.fillStyle = bg || "rgba(10,13,20,0.78)";
        ctx.fillRect(x, y - 15, w, 18);
        ctx.fillStyle = fg || INK.primary;
        ctx.fillText(text, x + 5, y - 2);
        return w;
    }

    function drawOverlay(ctx, w, h, overlay, lidarCam, layers) {
        if (layers.lidar && lidarCam && lidarCam.length) {
            // the "laser curtain": consecutive beams joined where the range is continuous
            ctx.lineWidth = 2;
            for (let i = 1; i < lidarCam.length; i++) {
                const a = lidarCam[i - 1], b = lidarCam[i];
                if (Math.abs(a[2] - b[2]) > 0.25 || Math.abs(a[0] - b[0]) > 0.08) continue;
                ctx.strokeStyle = rangeColor((a[2] + b[2]) / 2);
                ctx.globalAlpha = 0.55;
                ctx.beginPath();
                ctx.moveTo(a[0] * w, Math.min(h - 2, a[1] * h));
                ctx.lineTo(b[0] * w, Math.min(h - 2, b[1] * h));
                ctx.stroke();
            }
            ctx.globalAlpha = 1;
            lidarCam.forEach((p) => {
                const x = p[0] * w, y = Math.min(h - 3, p[1] * h);
                const owner = p[3] === 1;
                ctx.beginPath();
                ctx.arc(x, y, owner ? 4.5 : 3, 0, Math.PI * 2);
                ctx.fillStyle = owner ? STATUS.good : rangeColor(p[2]);
                ctx.fill();
                ctx.lineWidth = 1.5;
                ctx.strokeStyle = "rgba(10,13,20,0.9)";   // surface ring keeps dots legible on any background
                ctx.stroke();
            });
        }
        (overlay || []).forEach((p) => {
            if (layers.pose) {
                const pts = {};
                (p.pose || []).forEach((it) => { if (it.length >= 4) pts[it[0]] = it; });
                ctx.lineWidth = 2;
                ctx.strokeStyle = p.owner ? "#7dffe0" : "rgba(255,255,255,0.55)";
                LINKS.forEach(([a, b]) => {
                    const pa = pts[a], pb = pts[b];
                    if (!pa || !pb || pa[3] < 0.35 || pb[3] < 0.35) return;
                    ctx.beginPath();
                    ctx.moveTo(pa[1] * w, pa[2] * h);
                    ctx.lineTo(pb[1] * w, pb[2] * h);
                    ctx.stroke();
                });
            }
            if (layers.boxes && p.bbox && p.bbox.length === 4) {
                const [x1, y1, x2, y2] = [p.bbox[0] * w, p.bbox[1] * h, p.bbox[2] * w, p.bbox[3] * h];
                ctx.strokeStyle = p.owner ? STATUS.good : STATUS.warning;
                ctx.lineWidth = p.owner ? 3 : 2;
                ctx.setLineDash(p.owner ? [] : [6, 4]);        // bystander: dashed, not colour alone
                ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
                ctx.setLineDash([]);
                const who = p.owner ? "OWNER" : "PERSON";
                let x = x1;
                x += label(ctx, who, x, y1, p.owner ? "#0b0b0b" : "#0b0b0b", p.owner ? STATUS.good : STATUS.warning);
                const parts = [`cam ${p.cam_m != null ? p.cam_m.toFixed(2) : "--"} m`];
                if (p.lidar_m != null) parts.push(`lidar ${p.lidar_m.toFixed(2)} m`);
                if (p.match != null) parts.push(`match ${(p.match * 100).toFixed(0)}%`);
                label(ctx, parts.join(" | "), x + 2, y1);
            }
        });
    }

    // ---------- rolling time charts ----------
    const WINDOW_S = 30;

    class TimeChart {
        /** opts: {series:[{key,label,color,dash}], refs:[{key,label}], band:{lo,hi,label}, min, max, unit, digits} */
        constructor(canvas, opts) {
            this.c = canvas;
            this.o = opts;
            this.hoverX = null;
            canvas.addEventListener("mousemove", (e) => { const r = canvas.getBoundingClientRect(); this.hoverX = e.clientX - r.left; });
            canvas.addEventListener("mouseleave", () => { this.hoverX = null; });
        }

        draw(buf, now) {
            const c = this.c, dpr = window.devicePixelRatio || 1;
            const cw = c.clientWidth || 300, ch = c.clientHeight || 120;
            if (c.width !== Math.round(cw * dpr) || c.height !== Math.round(ch * dpr)) {
                c.width = Math.round(cw * dpr);
                c.height = Math.round(ch * dpr);
            }
            const g = c.getContext("2d");
            g.setTransform(dpr, 0, 0, dpr, 0, 0);
            g.clearRect(0, 0, cw, ch);
            const L = 38, R = 64, T = 8, B = 18;
            const pw = cw - L - R, ph = ch - T - B;
            const o = this.o;
            const keys = o.series.map((s) => s.key).concat((o.refs || []).map((r) => r.key));
            let lo = o.min, hi = o.max;
            buf.forEach((d) => keys.forEach((k) => {
                const v = d[k];
                if (v == null || !isFinite(v)) return;
                if (o.autoMin !== false) lo = Math.min(lo, v);
                hi = Math.max(hi, v);
            }));
            if (hi - lo < 1e-6) hi = lo + 1;
            const pad = (hi - lo) * 0.08;
            lo -= o.autoMin === false ? 0 : pad;
            hi += pad;
            const X = (t) => L + (1 - (now - t) / WINDOW_S) * pw;
            const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;

            // recessive grid + axis labels
            g.font = "11px 'JetBrains Mono', monospace";
            g.fillStyle = INK.muted;
            g.strokeStyle = INK.grid;
            g.lineWidth = 1;
            for (let i = 0; i <= 3; i++) {
                const v = lo + (hi - lo) * i / 3, y = Y(v);
                g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
                g.fillText(v.toFixed(o.digits ?? 1), 2, y + 4);
            }
            if (lo < 0 && hi > 0) {
                g.strokeStyle = INK.axis;
                g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(L + pw, Y(0)); g.stroke();
            }
            g.fillText(`-${WINDOW_S}s`, L, ch - 4);
            g.fillText("now", L + pw - 22, ch - 4);

            // band (e.g. stop..resume hysteresis) from the newest sample
            const last = buf.length ? buf[buf.length - 1] : null;
            if (o.band && last && last[o.band.lo] != null && last[o.band.hi] != null) {
                const y1 = Y(last[o.band.hi]), y2 = Y(last[o.band.lo]);
                g.fillStyle = "rgba(240,239,236,0.07)";
                g.fillRect(L, y1, pw, y2 - y1);
                g.fillStyle = INK.muted;
                g.fillText(o.band.label, L + 4, y1 - 3);
            }
            // reference lines (setpoint, threshold), labelled directly at the right edge
            (o.refs || []).forEach((r) => {
                if (!last || last[r.key] == null) return;
                g.strokeStyle = INK.secondary;
                g.setLineDash([2, 3]);
                g.beginPath();
                let started = false;
                buf.forEach((d) => {
                    if (d[r.key] == null) { started = false; return; }
                    const x = X(d.t), y = Y(d[r.key]);
                    if (started) g.lineTo(x, y); else { g.moveTo(x, y); started = true; }
                });
                g.stroke();
                g.setLineDash([]);
                g.fillStyle = INK.secondary;
                g.fillText(`${r.label} ${last[r.key].toFixed(o.digits ?? 2)}`, L + pw + 4, Y(last[r.key]) + 4);
            });
            // series: 2px lines, gaps where the value is missing
            o.series.forEach((s) => {
                g.strokeStyle = s.color;
                g.lineWidth = 2;
                g.setLineDash(s.dash || []);
                g.lineJoin = "round";
                g.beginPath();
                let started = false;
                buf.forEach((d) => {
                    const v = d[s.key];
                    if (v == null || !isFinite(v)) { started = false; return; }
                    const x = X(d.t), y = Y(v);
                    if (started) g.lineTo(x, y); else { g.moveTo(x, y); started = true; }
                });
                g.stroke();
                g.setLineDash([]);
            });
            // hover: crosshair + tooltip at the nearest sample
            if (this.hoverX != null && buf.length) {
                const tx = now - (1 - (this.hoverX - L) / pw) * WINDOW_S;
                let best = null;
                buf.forEach((d) => { if (!best || Math.abs(d.t - tx) < Math.abs(best.t - tx)) best = d; });
                if (best && this.hoverX >= L && this.hoverX <= L + pw) {
                    const x = X(best.t);
                    g.strokeStyle = INK.secondary;
                    g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke();
                    const lines = [`${(best.t - now).toFixed(1)} s`];
                    o.series.forEach((s) => {
                        if (best[s.key] != null) {
                            lines.push(`${s.label}: ${best[s.key].toFixed(o.digits ?? 2)}${o.unit || ""}`);
                            g.beginPath(); g.arc(x, Y(best[s.key]), 4, 0, Math.PI * 2);
                            g.fillStyle = s.color; g.fill();
                            g.strokeStyle = "#121826"; g.lineWidth = 2; g.stroke(); g.lineWidth = 1;
                        }
                    });
                    (o.refs || []).forEach((r) => { if (best[r.key] != null) lines.push(`${r.label}: ${best[r.key].toFixed(o.digits ?? 2)}`); });
                    if (best.state) lines.push(best.state);
                    g.font = "11px 'JetBrains Mono', monospace";
                    const tw = Math.max(...lines.map((l) => g.measureText(l).width)) + 12;
                    const bx = x + 10 + tw > cw ? x - 10 - tw : x + 10;
                    g.fillStyle = "rgba(10,13,20,0.92)";
                    g.fillRect(bx, T, tw, lines.length * 14 + 8);
                    g.strokeStyle = "rgba(255,255,255,0.10)";
                    g.strokeRect(bx, T, tw, lines.length * 14 + 8);
                    g.fillStyle = INK.primary;
                    lines.forEach((l, i) => g.fillText(l, bx + 6, T + 16 + i * 14));
                }
            }
        }
    }

    // ---------- state machine diagram ----------
    // 4 columns x 3 rows; 72 px between columns so every edge label sits in clear space
    const NODES = {
        IDLE: [4, 56, "IDLE"], OWNER_LOCKED: [194, 56, "OWNER LOCKED"], FOLLOWING: [384, 56, "FOLLOWING"],
        WAIT_FOR_OWNER_MOVEMENT: [574, 56, "WAIT FOR OWNER"],
        OWNER_RELEASED: [4, 170, "RELEASED"], TARGET_TEMPORARILY_LOST: [384, 170, "TEMP. LOST"],
        RECALLING_LAST_POSITION: [574, 170, "RECALL LAST POS."],
        TARGET_AMBIGUOUS: [4, 280, "AMBIGUOUS"], SAFETY_STOP: [194, 280, "SAFETY STOP"], SEARCH_ROTATING: [574, 280, "SEARCH 360"],
    };
    const NW = 118, NH = 34;
    // [from, to, label, bend (+ = bow left of travel), label offset dx, dy, text-anchor]
    const EDGES = [
        ["IDLE", "OWNER_LOCKED", "FOLLOW ME", 0, 0, -8], ["OWNER_LOCKED", "FOLLOWING", "start", 0, 0, -8],
        ["FOLLOWING", "WAIT_FOR_OWNER_MOVEMENT", "d ≤ stop", 20, 0, -12], ["WAIT_FOR_OWNER_MOVEMENT", "FOLLOWING", "d ≥ resume", 20, 0, 20],
        ["FOLLOWING", "TARGET_TEMPORARILY_LOST", "lost", 0, -6, 4, "end"], ["TARGET_TEMPORARILY_LOST", "RECALLING_LAST_POSITION", "path", 0, 0, -8],
        ["RECALLING_LAST_POSITION", "SEARCH_ROTATING", "empty / 25 s", 0, 6, 4, "start"], ["RECALLING_LAST_POSITION", "FOLLOWING", "relock", 0, 8, 2, "start"],
        ["OWNER_RELEASED", "IDLE", "", 0, 0, 0],
    ];
    const WHY = {
        IDLE: "No owner enrolled. Raise one hand (FOLLOW ME) or press ENROLL.",
        OWNER_LOCKED: "Owner profile ready. Raise one hand or press START to follow.",
        FOLLOWING: "Driving: PID on the EKF range/bearing, LiDAR avoidance on top.",
        WAIT_FOR_OWNER_MOVEMENT: "Owner within the stop distance, or paused (WAIT). Stationary; only turns to face them.",
        TARGET_TEMPORARILY_LOST: "Owner not seen by camera or LiDAR. Holding, waiting for a path to the last-seen spot.",
        RECALLING_LAST_POSITION: "Driving to where the owner was last seen (A* on the LiDAR map).",
        SEARCH_ROTATING: "Last-seen spot was empty: predicted turn, widening sweep, then a full 360.",
        TARGET_AMBIGUOUS: "Two people match the owner equally. Holding still until they separate.",
        SAFETY_STOP: "Latched stop (STOP gesture). Owner raises one hand, or press START, to resume.",
        OWNER_RELEASED: "Owner released. Profile deleted.",
    };

    function center(n) { const [x, y] = NODES[n]; return [x + NW / 2, y + NH / 2]; }

    function buildFSM(svg) {
        const ns = "http://www.w3.org/2000/svg";
        svg.innerHTML = `<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="${INK.muted}"/></marker></defs>`;
        EDGES.forEach(([a, b, lab, bend, ldx, ldy, anchor]) => {
            const [x1, y1] = center(a), [x2, y2] = center(b);
            const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy);
            // clip to node borders
            const k1 = Math.min(NW / 2 / Math.max(Math.abs(dx), 1e-6), NH / 2 / Math.max(Math.abs(dy), 1e-6)) * 1.06;
            const sx = x1 + dx * Math.min(k1, 0.5), sy = y1 + dy * Math.min(k1, 0.5);
            const ex = x2 - dx * Math.min(k1, 0.5), ey = y2 - dy * Math.min(k1, 0.5);
            const off = bend || 0;
            const mx = (sx + ex) / 2 - (dy / len) * off, my = (sy + ey) / 2 + (dx / len) * off;
            const path = document.createElementNS(ns, "path");
            path.setAttribute("d", `M${sx},${sy} Q${mx},${my} ${ex},${ey}`);
            path.setAttribute("class", "fsm-edge");
            path.setAttribute("marker-end", "url(#arr)");
            svg.appendChild(path);
            if (lab) {
                const t = document.createElementNS(ns, "text");
                t.setAttribute("x", mx + (ldx || 0)); t.setAttribute("y", my + (ldy || 0));
                t.setAttribute("text-anchor", anchor || "middle");
                t.setAttribute("class", "fsm-edge-label");
                t.textContent = lab;
                svg.appendChild(t);
            }
        });
        ["From any state:  STOP gesture → SAFETY STOP   ·   two look-alikes → AMBIGUOUS   ·   RELEASE → RELEASED",
         "SEARCH 360 → FOLLOWING when Re-ID passes   ·   SAFETY STOP → FOLLOWING on the owner's FOLLOW ME / START"]
            .forEach((line, i) => {
                const note = document.createElementNS(ns, "text");
                note.setAttribute("x", 4); note.setAttribute("y", 14 + i * 15); note.setAttribute("class", "fsm-edge-label");
                note.textContent = line;
                svg.appendChild(note);
            });
        Object.entries(NODES).forEach(([key, [x, y, name]]) => {
            const g = document.createElementNS(ns, "g");
            g.setAttribute("class", "fsm-node");
            g.dataset.state = key;
            g.innerHTML = `<rect x="${x}" y="${y}" width="${NW}" height="${NH}" rx="6"/><text x="${x + NW / 2}" y="${y + NH / 2 + 4}" text-anchor="middle">${name}</text>`;
            svg.appendChild(g);
        });
    }

    let fsmState = null, fsmSince = 0;
    function updateFSM(svg, state, extra) {
        if (state !== fsmState) { fsmState = state; fsmSince = performance.now(); }
        svg.querySelectorAll(".fsm-node").forEach((g) => g.classList.toggle("active", g.dataset.state === state));
        const why = document.getElementById("fsm-why");
        if (why) why.textContent = (WHY[state] || state || "--") + (extra ? `  ${extra}` : "");
        const dwell = document.getElementById("fsm-dwell");
        if (dwell) dwell.textContent = `${state || "--"} for ${((performance.now() - fsmSince) / 1000).toFixed(1)} s`;
    }

    // ---------- live settings ----------
    const GROUPS = [
        ["Following", [["DESIRED_DISTANCE", "Follow distance", "m", 0.05], ["MAX_LINEAR_VEL", "Max speed", "m/s", 0.01],
                       ["MAX_ANGULAR_VEL", "Max turn rate", "rad/s", 0.1]]],
        ["PID", [["PID_DIST_KP", "Range Kp"], ["PID_DIST_KI", "Range Ki"], ["PID_DIST_KD", "Range Kd"],
                 ["PID_TURN_KP", "Turn Kp"], ["PID_TURN_KI", "Turn Ki"], ["PID_TURN_KD", "Turn Kd"], ["PID_FF_GAIN", "Feed-forward"]]],
        ["Camera ↔ LiDAR", [["CAMERA_HFOV_DEG", "HFOV", "°", 0.5], ["CAM_X_M", "Cam ahead of LiDAR", "m", 0.01],
                            ["CAM_Y_M", "Cam left of LiDAR", "m", 0.01], ["CAM_HEIGHT_M", "Cam height", "m", 0.01],
                            ["LIDAR_HEIGHT_M", "LiDAR height", "m", 0.01], ["CAM_YAW_DEG", "Cam yaw (+left)", "°", 0.5],
                            ["CAM_PITCH_DEG", "Cam pitch (+down)", "°", 0.5], ["DISTANCE_SCALE", "Camera range scale", "×", 0.01]]],
        ["Identity & link", [["IDENTITY_MATCH_THRESHOLD", "Match threshold", "", 0.01], ["REACQ_THRESHOLD", "Re-acquire threshold", "", 0.01],
                             ["PI_IP", "Raspberry Pi IP", "", null]]],
    ];

    function buildSettings(form, values) {
        form.innerHTML = "";
        GROUPS.forEach(([title, fields]) => {
            const fs = document.createElement("fieldset");
            fs.innerHTML = `<legend>${title}</legend>`;
            fields.forEach(([key, name, unit, step]) => {
                const v = values[key];
                const row = document.createElement("label");
                row.className = "set-row";
                const isText = typeof v === "string";
                row.innerHTML = `<span>${name}${unit ? ` <em>${unit}</em>` : ""}</span>` +
                    `<input name="${key}" ${isText ? 'type="text" inputmode="decimal"' : `type="number" step="${step || 0.01}"`} value="${v ?? ""}">`;
                fs.appendChild(row);
            });
            form.appendChild(fs);
        });
    }

    function readSettings(form, original) {
        const out = {};
        form.querySelectorAll("input").forEach((inp) => {
            const k = inp.name, o = original[k];
            const v = typeof o === "string" ? inp.value.trim() : Number(inp.value);
            if (v !== o && !(typeof v === "number" && isNaN(v))) out[k] = v;
        });
        return out;
    }

    window.HFViz = { drawOverlay, TimeChart, SERIES, buildFSM, updateFSM, buildSettings, readSettings };
})();
