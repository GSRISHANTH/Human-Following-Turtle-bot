/**
 * RViz-style 2-D view (top-down orthographic, fixed frame "map", x right, y up).
 * Displays, like RViz's: Grid, Map (OccupancyGrid), LaserScan, Odometry trail, Path + goal Pose, TF axes,
 * robot footprint, camera FOV, person marker with velocity arrow and 2-sigma covariance.
 * Data: /api/scene (SSE, 10 Hz) and /api/map.png (occupancy grid, origin in the response headers).
 * Mouse: drag = pan (stops following), wheel = zoom at the cursor, double-click = reset.
 */
(function () {
    "use strict";
    const C = {
        bg: "#303030", grid: "rgba(160,160,164,0.22)", gridAxis: "rgba(160,160,164,0.45)",
        scanStatic: "#ff3b30", scanWall: "#ff6b5e", scanMoving: "#ffa500", scanPerson: "#19e619",
        robot: "#e6e6e6", tfX: "#ff2020", tfY: "#20d020", trail: "#00c8ff", path: "#19ff00", goal: "#ff00ff",
        person: "#19e619", vel: "#ffd400", cov: "rgba(255,0,255,0.85)", fov: "rgba(255,214,0,0.55)",
        ink: "#f0efec", muted: "#a9a8a0",
    };
    const ROBOT_R = 0.105;          // TurtleBot3 Burger footprint radius

    function init() {
        const canvas = document.getElementById("rviz");
        if (!canvas) return;
        const ctx = canvas.getContext("2d");
        const status = document.getElementById("rviz-status");
        const cursorEl = document.getElementById("rviz-cursor");
        const layer = (id) => { const el = document.getElementById("rv-" + id); return !el || el.checked; };
        const followBtn = document.getElementById("rv-follow"), upBtn = document.getElementById("rv-up");

        let scene = null, mapImg = null, mapMeta = null, mapScans = -1, mapBusy = false;
        let view = { x: 0, y: 0, ppm: 60, follow: true, robotUp: false };    // ppm = pixels per metre
        let drag = null, mouse = null;

        const setBtn = (b, on) => { if (b) b.classList.toggle("active", on); };
        setBtn(followBtn, view.follow);
        setBtn(upBtn, view.robotUp);
        if (followBtn) followBtn.addEventListener("click", () => { view.follow = !view.follow; setBtn(followBtn, view.follow); });
        if (upBtn) upBtn.addEventListener("click", () => { view.robotUp = !view.robotUp; setBtn(upBtn, view.robotUp); });
        const resetBtn = document.getElementById("rv-reset");
        const reset = () => { view.ppm = 60; view.follow = true; setBtn(followBtn, true); };
        if (resetBtn) resetBtn.addEventListener("click", reset);

        // ---- data ----
        function connect() {
            const es = new EventSource("/api/scene");
            es.onmessage = (ev) => {
                try { scene = JSON.parse(ev.data); } catch (e) { return; }
                if (scene.ok && scene.map && scene.map.scans !== mapScans && !mapBusy) loadMap(scene.map.scans);
            };
            es.onerror = () => { if (status) status.textContent = "scene stream offline"; };
        }
        async function loadMap(scans) {
            mapBusy = true;
            try {
                const res = await fetch("/api/map.png?s=" + scans, { cache: "no-store" });
                const o = (res.headers.get("X-Map-Origin") || "0,0").split(",").map(Number);
                const meta = { ox: o[0], oy: o[1], res: Number(res.headers.get("X-Map-Res") || 0.05), n: Number(res.headers.get("X-Map-N") || 240) };
                const bmp = await createImageBitmap(await res.blob());
                if (mapImg && mapImg.close) mapImg.close();
                mapImg = bmp; mapMeta = meta; mapScans = scans;
            } catch (e) { /* next scene retries */ }
            setTimeout(() => { mapBusy = false; }, 400);   // at most ~2 map images per second
        }

        // ---- transforms ----
        function frame() {
            const dpr = window.devicePixelRatio || 1;
            const w = canvas.clientWidth || 400, h = canvas.clientHeight || 400;
            if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
                canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
            }
            return { dpr, w, h };
        }
        function rot() { return view.robotUp && scene && scene.pose ? Math.PI / 2 - scene.pose[2] : 0; }
        function toScreen(x, y, F) {
            const a = rot(), dx = x - view.x, dy = y - view.y;
            const rx = Math.cos(a) * dx - Math.sin(a) * dy, ry = Math.sin(a) * dx + Math.cos(a) * dy;
            return [F.w / 2 + rx * view.ppm, F.h / 2 - ry * view.ppm];
        }
        function toWorld(sx, sy, F) {
            const a = -rot(), rx = (sx - F.w / 2) / view.ppm, ry = -(sy - F.h / 2) / view.ppm;
            return [view.x + Math.cos(a) * rx - Math.sin(a) * ry, view.y + Math.sin(a) * rx + Math.cos(a) * ry];
        }

        // ---- interaction ----
        canvas.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; canvas.setPointerCapture(e.pointerId); });
        canvas.addEventListener("pointerup", () => { drag = null; });
        canvas.addEventListener("pointermove", (e) => {
            const r = canvas.getBoundingClientRect();
            mouse = [e.clientX - r.left, e.clientY - r.top];
            if (!drag) return;
            const F = frame();
            const [ax, ay] = toWorld(drag.x - r.left, drag.y - r.top, F), [bx, by] = toWorld(mouse[0], mouse[1], F);
            view.x -= bx - ax; view.y -= by - ay;
            drag = { x: e.clientX, y: e.clientY };
            if (view.follow) { view.follow = false; setBtn(followBtn, false); }
        });
        canvas.addEventListener("pointerleave", () => { mouse = null; });
        canvas.addEventListener("wheel", (e) => {
            e.preventDefault();
            const r = canvas.getBoundingClientRect(), F = frame();
            const before = toWorld(e.clientX - r.left, e.clientY - r.top, F);
            view.ppm = Math.max(12, Math.min(400, view.ppm * Math.exp(-e.deltaY * 0.0015)));
            const after = toWorld(e.clientX - r.left, e.clientY - r.top, F);
            if (!view.follow) { view.x += before[0] - after[0]; view.y += before[1] - after[1]; }
        }, { passive: false });
        canvas.addEventListener("dblclick", reset);

        // ---- drawing ----
        function line(pts, F, color, width, dash) {
            if (pts.length < 2) return;
            ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []);
            ctx.beginPath();
            pts.forEach((p, i) => { const [sx, sy] = toScreen(p[0], p[1], F); if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy); });
            ctx.stroke(); ctx.setLineDash([]);
        }
        function arrow(x0, y0, x1, y1, F, color, width) {
            const [a, b] = toScreen(x0, y0, F), [c, d] = toScreen(x1, y1, F);
            const ang = Math.atan2(d - b, c - a), L = Math.hypot(c - a, d - b);
            if (L < 2) return;
            ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
            ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
            const hs = Math.min(10, L * 0.35);
            ctx.beginPath(); ctx.moveTo(c, d);
            ctx.lineTo(c - hs * Math.cos(ang - 0.45), d - hs * Math.sin(ang - 0.45));
            ctx.lineTo(c - hs * Math.cos(ang + 0.45), d - hs * Math.sin(ang + 0.45));
            ctx.closePath(); ctx.fill();
        }
        function text(t, sx, sy, color) {
            ctx.font = "600 11px 'JetBrains Mono', monospace";
            ctx.lineWidth = 3; ctx.strokeStyle = "rgba(0,0,0,0.75)"; ctx.strokeText(t, sx, sy);
            ctx.fillStyle = color || C.ink; ctx.fillText(t, sx, sy);
        }

        function draw() {
            const F = frame();
            ctx.setTransform(F.dpr, 0, 0, F.dpr, 0, 0);
            ctx.fillStyle = C.bg; ctx.fillRect(0, 0, F.w, F.h);
            const ok = scene && scene.ok;
            if (ok && view.follow) { view.x = scene.pose[0]; view.y = scene.pose[1]; }

            // Map (OccupancyGrid) - drawn as an image in world coordinates
            if (layer("map") && mapImg && mapMeta) {
                const half = mapMeta.n * mapMeta.res / 2;
                const [ax, ay] = toScreen(mapMeta.ox - half, mapMeta.oy + half, F);
                ctx.save();
                ctx.translate(ax, ay); ctx.rotate(-rot());
                ctx.imageSmoothingEnabled = false;
                const sz = mapMeta.n * mapMeta.res * view.ppm;
                ctx.drawImage(mapImg, 0, 0, sz, sz);
                ctx.restore();
            }
            // Grid (1 m cells, axes through the map origin)
            if (layer("grid")) {
                const span = Math.ceil(Math.max(F.w, F.h) / view.ppm) + 2;
                const gx = Math.round(view.x), gy = Math.round(view.y);
                for (let i = -span; i <= span; i++) {
                    const isAxis = (gx + i) === 0, isAxisY = (gy + i) === 0;
                    line([[gx + i, gy - span], [gx + i, gy + span]], F, isAxis ? C.gridAxis : C.grid, 1);
                    line([[gx - span, gy + i], [gx + span, gy + i]], F, isAxisY ? C.gridAxis : C.grid, 1);
                }
            }
            if (!ok) {
                text(scene ? "waiting for odometry / first scans..." : "connecting to /api/scene...", 12, 22, C.muted);
                requestAnimationFrame(draw);
                return;
            }
            const [px, py, yaw] = scene.pose;
            // Odometry trail
            if (layer("trail")) line(scene.trail, F, C.trail, 1.5);
            // Camera FOV
            if (layer("fov") && scene.fov) {
                const hf = scene.fov.hfov_deg * Math.PI / 360, cy = yaw + (scene.fov.yaw_deg || 0) * Math.PI / 180, L = 4.0;
                line([[px + L * Math.cos(cy + hf), py + L * Math.sin(cy + hf)], [px, py], [px + L * Math.cos(cy - hf), py + L * Math.sin(cy - hf)]], F, C.fov, 1.5, [6, 4]);
            }
            // Path + goal Pose
            if (layer("path")) {
                line(scene.path, F, C.path, 2.5);
                if (scene.goal) {
                    const g = scene.goal, last = scene.path.length > 1 ? scene.path[scene.path.length - 2] : [px, py];
                    const a = Math.atan2(g[1] - last[1], g[0] - last[0]);
                    arrow(g[0] - 0.25 * Math.cos(a), g[1] - 0.25 * Math.sin(a), g[0], g[1], F, C.goal, 3);
                }
                if (scene.last_seen) {
                    const [sx, sy] = toScreen(scene.last_seen[0], scene.last_seen[1], F);
                    ctx.strokeStyle = C.goal; ctx.lineWidth = 2;
                    ctx.beginPath(); ctx.moveTo(sx - 7, sy - 7); ctx.lineTo(sx + 7, sy + 7); ctx.moveTo(sx + 7, sy - 7); ctx.lineTo(sx - 7, sy + 7); ctx.stroke();
                    text("last seen", sx + 9, sy - 6, C.goal);
                }
            }
            // LaserScan
            if (layer("scan")) {
                const sz = Math.max(2, Math.min(5, 0.03 * view.ppm));
                const col = [C.scanStatic, C.scanWall, C.scanMoving, C.scanPerson];
                scene.scan.forEach((p) => {
                    const [sx, sy] = toScreen(p[0], p[1], F);
                    if (sx < -5 || sy < -5 || sx > F.w + 5 || sy > F.h + 5) return;
                    ctx.fillStyle = col[p[2]] || C.scanStatic;
                    ctx.fillRect(sx - sz / 2, sy - sz / 2, sz, sz);
                });
            }
            // Person marker + velocity + 2-sigma covariance
            if (layer("person") && scene.person) {
                const P = scene.person, [sx, sy] = toScreen(P.x, P.y, F);
                if (P.cov) {
                    const a = P.cov[0], b = P.cov[1], d = P.cov[2];
                    const tr = (a + d) / 2, det = Math.sqrt(Math.max(0, ((a - d) / 2) ** 2 + b * b));
                    const l1 = tr + det, l2 = Math.max(1e-6, tr - det), th = 0.5 * Math.atan2(2 * b, a - d);
                    ctx.save(); ctx.translate(sx, sy); ctx.rotate(-(th + rot()));
                    ctx.strokeStyle = C.cov; ctx.lineWidth = 1.5;
                    ctx.beginPath(); ctx.ellipse(0, 0, Math.max(3, 2 * Math.sqrt(l1) * view.ppm), Math.max(3, 2 * Math.sqrt(l2) * view.ppm), 0, 0, Math.PI * 2); ctx.stroke();
                    ctx.restore();
                }
                ctx.fillStyle = "rgba(25,230,25,0.25)"; ctx.strokeStyle = C.person; ctx.lineWidth = 2;
                ctx.beginPath(); ctx.arc(sx, sy, Math.max(6, 0.22 * view.ppm), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
                arrow(P.x, P.y, P.x + P.vx, P.y + P.vy, F, C.vel, 2.5);
                text(`owner ${P.range.toFixed(2)} m  ${Math.hypot(P.vx, P.vy).toFixed(2)} m/s`, sx + 12, sy - 10, C.person);
            }
            // Robot footprint + TF axes (base_link)
            {
                const [sx, sy] = toScreen(px, py, F);
                ctx.fillStyle = "rgba(230,230,230,0.18)"; ctx.strokeStyle = C.robot; ctx.lineWidth = 2;
                ctx.beginPath(); ctx.arc(sx, sy, Math.max(5, ROBOT_R * view.ppm), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
                if (layer("tf")) {
                    const L = 0.3;
                    arrow(px, py, px + L * Math.cos(yaw), py + L * Math.sin(yaw), F, C.tfX, 2.5);
                    arrow(px, py, px - L * Math.sin(yaw), py + L * Math.cos(yaw), F, C.tfY, 2.5);
                    text("base_link", sx + 8, sy + 16, C.muted);
                }
            }
            // HUD: frame, scale bar, source
            const bar = Math.pow(10, Math.floor(Math.log10(120 / view.ppm)));
            const barM = [1, 2, 5].map((k) => k * bar).filter((m) => m * view.ppm <= 140).pop() || bar;
            ctx.strokeStyle = C.ink; ctx.lineWidth = 2;
            ctx.beginPath(); ctx.moveTo(14, F.h - 16); ctx.lineTo(14 + barM * view.ppm, F.h - 16); ctx.stroke();
            text(`${barM} m`, 14, F.h - 22);
            text(`Fixed Frame: map   ${scene.hz} Hz   ${scene.source}${scene.source === "slam" ? "" : "  match " + scene.score.toFixed(2)}`, 12, 20, scene.loc_ok || scene.source === "slam" ? C.ink : C.scanMoving);
            if (status) status.textContent = `pose ${px.toFixed(2)}, ${py.toFixed(2)} m  ${(yaw * 180 / Math.PI).toFixed(0)}°  |  ${scene.scan.length} pts  |  plan ${scene.plan_status || "--"}`;
            if (cursorEl) cursorEl.textContent = mouse ? (() => { const w = toWorld(mouse[0], mouse[1], F); return `cursor ${w[0].toFixed(2)}, ${w[1].toFixed(2)} m`; })() : "";
            requestAnimationFrame(draw);
        }
        connect();
        requestAnimationFrame(draw);
    }
    document.addEventListener("DOMContentLoaded", init);
})();
