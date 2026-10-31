"""UDP link: wire format, loss handling, and the real Pi node talking to the laptop receiver.

The end-to-end test starts pi/pi_turtlebot_node.py --no-motors as a subprocess (no camera
needed: it serves its "camera not detected" frame) and drives it with the laptop classes.
"""

import os
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pi"))
sys.path.insert(0, str(ROOT / "laptop"))

import tb_proto  # noqa: E402


class WireFormat(unittest.TestCase):
    def test_command_roundtrip(self):
        lin, ang, seq = tb_proto.parse_command(tb_proto.encode_command(0.18, -0.7, 42))
        self.assertAlmostEqual(lin, 0.18)
        self.assertAlmostEqual(ang, -0.7)
        self.assertEqual(seq, 42)

    def test_legacy_and_json_commands(self):
        self.assertEqual(tb_proto.parse_command(b"V:0.10,W:0.20\n"), (0.10, 0.20, None))
        self.assertEqual(tb_proto.parse_command(b'{"linear": 0.1, "angular": 0.3}'), (0.1, 0.3, None))
        self.assertIsNone(tb_proto.parse_command(b"garbage"))
        self.assertIsNone(tb_proto.parse_command(b"V:x,W:y"))

    def test_seq_order_and_wrap(self):
        self.assertTrue(tb_proto.seq_is_newer(11, 10))
        self.assertFalse(tb_proto.seq_is_newer(9, 10))
        self.assertTrue(tb_proto.seq_is_newer(2, 0xFFFFFFFE))
        self.assertTrue(tb_proto.seq_is_newer(None, 5))

    def test_restarted_sender_is_not_ignored_by_the_pi_node(self):
        """Pi node rule: an older seq is accepted after 0.5 s of silence. Checked against the real node source."""
        src = (ROOT / "pi" / "pi_turtlebot_node.py").read_text(encoding="utf-8")
        self.assertIn("(time.monotonic() - last_cmd_time) < 0.5", src)

    def test_frame_roundtrip_in_any_order(self):
        jpeg = os.urandom(30000)
        pkts = tb_proto.pack_frame(jpeg, 7, 123.5)
        self.assertGreater(len(pkts), 20)
        self.assertTrue(all(len(p) <= tb_proto.HEADER_SIZE + tb_proto.MAX_PAYLOAD for p in pkts))
        asm = tb_proto.Reassembler()
        out = None
        for p in reversed(pkts):
            out = asm.feed(p) or out
        self.assertEqual(out[0], "video")
        self.assertEqual(out[1], jpeg)
        self.assertEqual(out[2], 123.5)

    def test_lost_packet_drops_only_that_frame(self):
        asm = tb_proto.Reassembler()
        a = tb_proto.pack_frame(os.urandom(5000), 1, 0.0)
        b = tb_proto.pack_frame(os.urandom(5000), 2, 0.0)
        results = [asm.feed(p) for p in a[:2] + a[3:]]  # frame 1 loses a datagram
        self.assertTrue(all(r is None for r in results))
        results = [asm.feed(p) for p in b]
        self.assertEqual(results[-1][0], "video")
        self.assertEqual(asm.frames_dropped, 1)

    def test_late_packet_of_old_frame_is_ignored(self):
        asm = tb_proto.Reassembler()
        old = tb_proto.pack_frame(os.urandom(5000), 1, 0.0)
        new = tb_proto.pack_frame(os.urandom(5000), 2, 0.0)
        for p in new:
            asm.feed(p)
        self.assertIsNone(asm.feed(old[0]))

    def test_sender_restart_resyncs(self):
        asm = tb_proto.Reassembler()
        for p in tb_proto.pack_frame(os.urandom(3000), 500, 0.0):
            asm.feed(p)
        got = None
        for fid in range(1, 30):  # restarted Pi counts from 1 again
            for p in tb_proto.pack_frame(os.urandom(3000), fid, 0.0):
                got = asm.feed(p) or got
        self.assertEqual(got[0], "video")

    def test_status_packet(self):
        asm = tb_proto.Reassembler()
        kind, payload, _ = asm.feed(tb_proto.pack_status({"ros": "ROS2", "subs": 1}, 0.0))
        self.assertEqual(kind, "status")
        self.assertEqual(payload["subs"], 1)

    def test_garbage_is_ignored(self):
        asm = tb_proto.Reassembler()
        for junk in (b"", b"xx", b"HELLO\n", os.urandom(64)):
            self.assertIsNone(asm.feed(junk))


class EndToEnd(unittest.TestCase):
    """Real Pi script <-> laptop VideoStreamer + RobotController over loopback."""

    @classmethod
    def setUpClass(cls):
        for port in (tb_proto.CMD_PORT, tb_proto.VIDEO_PORT):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.bind(("0.0.0.0", port))
            except OSError:
                raise unittest.SkipTest(f"UDP {port} is in use; stop the dashboard first")
            finally:
                s.close()
        cls.pi = subprocess.Popen(
            [sys.executable, "-u", str(ROOT / "pi" / "pi_turtlebot_node.py"), "--no-motors"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        os.environ["PI_IP"] = "127.0.0.1"
        from config import Config
        Config.PI_IP = "127.0.0.1"
        from video_streamer import VideoStreamer
        cls.streamer = VideoStreamer()

    @classmethod
    def tearDownClass(cls):
        cls.streamer.stop()
        cls.pi.terminate()
        try:
            cls.pi.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cls.pi.kill()

    def test_video_status_and_commands(self):
        deadline = time.time() + 15
        while time.time() < deadline and (self.streamer.frame_count < 10 or not self.streamer.pi_status):
            time.sleep(0.1)
        self.assertGreaterEqual(self.streamer.frame_count, 3, "no video reached the laptop")
        self.assertTrue(self.streamer.latest_jpeg.startswith(b"\xff\xd8"))
        self.assertTrue(self.streamer.pi_status, "no status packet from the Pi")
        self.assertFalse(self.streamer.pi_status["motors"])

        from robot_controller import RobotController
        from config import Config
        Config.COMMAND_MODE = "udp"            # this one test really sends, to the Pi node it started on loopback
        try:
            rc = RobotController()
        finally:
            Config.COMMAND_MODE = "mock"
        rc.set_mode("MANUAL")
        rc.update_manual_teleop(0.1, 0.0)
        time.sleep(1.0)
        rc.stop()
        base = self.streamer.pi_status["cmds"]
        self.assertGreater(base, 5, "Pi counted almost no command packets")


if __name__ == "__main__":
    unittest.main()
