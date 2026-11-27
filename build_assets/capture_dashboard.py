import os, json, time, base64, subprocess, urllib.request
import websocket
from PIL import Image

edge_candidates = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
]
browser = next((p for p in edge_candidates if os.path.exists(p)), None)
assert browser, "No Edge/Chrome found"

tmp_dir = os.path.abspath("build_assets/edge_cdp_tmp")
os.makedirs(tmp_dir, exist_ok=True)

proc = subprocess.Popen(
    [
        browser,
        "--headless=new",
        "--disable-gpu",
        "--remote-debugging-port=9229",
        "--remote-allow-origins=*",
        f"--user-data-dir={tmp_dir}",
        "--window-size=1520,960",
        "about:blank",
    ],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)

try:
    ws_url = None
    for _ in range(30):
        try:
            targets = json.loads(urllib.request.urlopen("http://127.0.0.1:9229/json/list", timeout=1).read().decode())
            page = next((t for t in targets if t.get("type") == "page"), None)
            if page:
                ws_url = page["webSocketDebuggerUrl"]
                break
        except Exception:
            time.sleep(0.25)
    assert ws_url, "Failed to connect to Edge CDP"

    ws = websocket.create_connection(ws_url, timeout=15)
    msg_id = 0

    def cdp(method, params=None):
        global msg_id
        msg_id += 1
        cid = msg_id
        ws.send(json.dumps({"id": cid, "method": method, "params": params or {}}))
        while True:
            resp = json.loads(ws.recv())
            if resp.get("id") == cid:
                return resp.get("result", {})

    cdp("Page.enable")
    cdp("Runtime.enable")
    cdp("Emulation.setDeviceMetricsOverride", {"width": 1520, "height": 960, "deviceScaleFactor": 1, "mobile": False})
    cdp("Page.navigate", {"url": "http://localhost:8000"})
    time.sleep(2.5)

    # Click Simulation Mode so the AI feed renders live skeleton/tracking instead of offline placeholder
    cdp("Runtime.evaluate", {"expression": "document.getElementById('btn-mode-sim')?.click();"})
    time.sleep(5.0)

    # 1. Capture top dashboard viewport (1520x960)
    shot1 = cdp("Page.captureScreenshot", {"format": "png"})
    with open("build_assets/dashboard_top.png", "wb") as f:
        f.write(base64.b64decode(shot1["data"]))
    print("Saved build_assets/dashboard_top.png")

    # 2. Scroll to State Machine section and capture
    cdp("Runtime.evaluate", {"expression": "document.getElementById('fsm-svg')?.scrollIntoView({behavior: 'instant', block: 'center'});"})
    time.sleep(1.0)
    shot2 = cdp("Page.captureScreenshot", {"format": "png"})
    with open("build_assets/dashboard_fsm_view.png", "wb") as f:
        f.write(base64.b64decode(shot2["data"]))

    # Also capture tightly cropped State Machine card via document coordinates (add window.scrollY)
    box_res = cdp("Runtime.evaluate", {
        "expression": "(() => { const svg = document.getElementById('fsm-svg'); const el = svg.parentElement; const r = el.getBoundingClientRect(); return JSON.stringify({x: r.left + window.scrollX, y: r.top + window.scrollY, w: r.width, h: r.height}); })()"
    })
    box = json.loads(box_res["result"]["value"])
    shot_fsm = cdp("Page.captureScreenshot", {
        "format": "png",
        "captureBeyondViewport": True,
        "clip": {"x": max(0, box["x"] - 4), "y": max(0, box["y"] - 4), "width": box["w"] + 8, "height": box["h"] + 8, "scale": 2}
    })
    with open("build_assets/dashboard_fsm.png", "wb") as f:
        f.write(base64.b64decode(shot_fsm["data"]))
    print("Saved build_assets/dashboard_fsm.png and dashboard_fsm_view.png")
    ws.close()
finally:
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except Exception:
        proc.kill()
