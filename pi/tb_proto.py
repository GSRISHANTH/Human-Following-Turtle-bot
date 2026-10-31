"""UDP wire format shared by the Pi node, the laptop, and the HIL virtual Pi.

Nothing is hosted on the Pi. The laptop owns both directions:

    laptop UDP :VIDEO_PORT  --HELLO-->   Pi UDP :CMD_PORT     (every 0.5 s, from the video socket)
    laptop (any port)       --V:..,W:..> Pi UDP :CMD_PORT     (20 Hz velocity, doubles as heartbeat)
    Pi                      --video/status--> source address of the last HELLO

Video is one JPEG per frame, cut into datagrams that fit one Ethernet/Wi-Fi MTU.
A frame with a missing datagram is dropped. Nothing is retransmitted, so one
lost packet costs one frame instead of stalling the stream the way TCP does.
"""

import json
import struct

CMD_PORT = 8888
VIDEO_PORT = 5000

MAGIC = b"TB"
T_VIDEO = 1
T_STATUS = 2
T_SCAN = 3

SCAN_BINS = 360   # 1 degree per bin, bin 0 straight ahead, counter-clockwise

# scan payload: odom x, y, yaw (float32), odom_valid, bin count, then uint16 millimetres per bin (0 = no return)
_SCAN = struct.Struct("!fffBH")

# magic, type, frame_id, chunk_index, chunk_count, capture_time
_HDR = struct.Struct("!2sBIHHd")
HEADER_SIZE = _HDR.size
MAX_PAYLOAD = 1200

HELLO = b"HELLO\n"


def encode_command(linear, angular, seq):
    return f"V:{linear:.3f},W:{angular:.3f},S:{seq}\n".encode("ascii")


def parse_command(data):
    """Return (linear, angular, seq) or None. seq is None for old senders without S:."""
    text = data.decode("utf-8", errors="ignore").strip()
    if text.startswith("{"):
        try:
            obj = json.loads(text)
            return float(obj.get("linear", 0.0)), float(obj.get("angular", 0.0)), obj.get("seq")
        except (ValueError, TypeError):
            return None
    if "V:" not in text or "W:" not in text:
        return None
    lin = ang = 0.0
    seq = None
    try:
        for part in text.split(","):
            key, _, val = part.partition(":")
            if key == "V":
                lin = float(val)
            elif key == "W":
                ang = float(val)
            elif key == "S":
                seq = int(val)
    except ValueError:
        return None
    return lin, ang, seq


def seq_is_newer(seq, last):
    """True when seq is not older than the last one. A restarted sender is handled by the receiver's silence timeout."""
    if seq is None or last is None:
        return True
    return ((seq - last) & 0xFFFFFFFF) < 0x80000000


def pack_frame(jpeg, frame_id, capture_time):
    """Split one JPEG into datagrams."""
    count = max(1, -(-len(jpeg) // MAX_PAYLOAD))
    out = []
    for i in range(count):
        part = jpeg[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD]
        out.append(_HDR.pack(MAGIC, T_VIDEO, frame_id & 0xFFFFFFFF, i, count, capture_time) + part)
    return out


def pack_status(status, capture_time):
    return _HDR.pack(MAGIC, T_STATUS, 0, 0, 1, capture_time) + json.dumps(status).encode("utf-8")


def pack_scan(bins_mm, pose, capture_time):
    """bins_mm: SCAN_BINS ints (0 = none). pose: (x, y, yaw) or None."""
    x, y, yaw = pose if pose is not None else (0.0, 0.0, 0.0)
    n = len(bins_mm)
    body = _SCAN.pack(x, y, yaw, 1 if pose is not None else 0, n) + struct.pack(f"!{n}H", *bins_mm)
    return _HDR.pack(MAGIC, T_SCAN, 0, 0, 1, capture_time) + body


class Reassembler:
    """Rebuilds the newest frame. Anything older than a frame already started is dropped."""

    def __init__(self):
        self.frame_id = None
        self.count = 0
        self.parts = {}
        self.capture_time = 0.0
        self.last_done = None
        self.stale_run = 0
        self.frames_ok = 0
        self.frames_dropped = 0

    def feed(self, data):
        """Returns ('video', jpeg, capture_time), ('status', dict, t), or None."""
        if len(data) < HEADER_SIZE:
            return None
        magic, ptype, fid, idx, count, ts = _HDR.unpack_from(data)
        if magic != MAGIC:
            return None
        body = data[HEADER_SIZE:]
        if ptype == T_STATUS:
            try:
                return "status", json.loads(body.decode("utf-8")), ts
            except ValueError:
                return None
        if ptype == T_SCAN:
            if len(body) < _SCAN.size:
                return None
            x, y, yaw, has_odom, n = _SCAN.unpack_from(body)
            if n == 0 or len(body) < _SCAN.size + 2 * n:
                return None
            bins = list(struct.unpack_from(f"!{n}H", body, _SCAN.size))
            return "scan", {"bins": bins, "pose": (x, y, yaw) if has_odom else None}, ts
        if ptype != T_VIDEO or count == 0 or idx >= count:
            return None

        if fid != self.frame_id:
            if self.last_done is not None and ((fid - self.last_done) & 0xFFFFFFFF) >= 0x80000000:
                # Late packet from a frame already shown. But a Pi that restarted counts from 1 again,
                # so a long run of "old" ids means the sender reset: accept it.
                self.stale_run += 1
                if self.stale_run < 20:
                    return None
                self.last_done = None
            self.stale_run = 0
            if self.parts:
                self.frames_dropped += 1
            self.frame_id = fid
            self.count = count
            self.parts = {}
            self.capture_time = ts
        if count != self.count:
            return None
        self.parts[idx] = body
        if len(self.parts) < self.count:
            return None

        jpeg = b"".join(self.parts[i] for i in range(self.count))
        self.last_done = fid
        self.parts = {}
        self.frame_id = None
        self.frames_ok += 1
        return "video", jpeg, self.capture_time
