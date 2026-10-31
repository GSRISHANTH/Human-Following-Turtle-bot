"""
Gesture Recognizer Module (Human-Robot Interaction):
Recognizes natural hand/arm gestures using MediaPipe Pose landmarks:
- 'FOLLOW_ME' (Raise hand above head): Lock onto person & start following
- 'STOP_HALT' (Both hands raised): Emergency stop
- 'RELEASE' (Crossed arms): Release lock / disengage
"""

class GestureRecognizer:
    def __init__(self, debounce_frames=5):
        self.debounce_frames = debounce_frames
        self.current_candidate = None
        self.candidate_count = 0
        self.active_gesture = "NONE"

    def analyze_pose(self, landmarks):
        """
        Analyzes 33 MediaPipe pose landmarks to detect commands.
        Returns: active_gesture (str), is_new_event (bool)
        """
        if not landmarks or len(landmarks) < 25:
            self._reset_candidate()
            return "NONE", False

        # Key Landmarks
        nose = landmarks[0]
        sh_l, sh_r = landmarks[11], landmarks[12]
        el_l, el_r = landmarks[13], landmarks[14]
        wr_l, wr_r = landmarks[15], landmarks[16]

        detected = "NONE"

        # Check visibility
        wr_l_vis = wr_l.visibility > 0.4
        wr_r_vis = wr_r.visibility > 0.4
        el_l_vis = el_l.visibility > 0.35
        el_r_vis = el_r.visibility > 0.35
        sh_vis = (sh_l.visibility > 0.4) and (sh_r.visibility > 0.4)

        if sh_vis:
            # Gesture 1: STOP_HALT (Both hands raised high above shoulders)
            if wr_l_vis and wr_r_vis:
                if (wr_l.y < sh_l.y - 0.08) and (wr_r.y < sh_r.y - 0.08):
                    detected = "STOP_HALT"

            # Gesture 2: RELEASE (Arms deliberately crossed in an 'X' over chest)
            if detected == "NONE" and wr_l_vis and wr_r_vis:
                # Wrists must cross past each other at chest level.
                # Invariant vector test: (wr_l.x - wr_r.x) * (sh_l.x - sh_r.x) < 0
                # In normal posture, wrists align with shoulders (product > 0).
                # When crossed in an 'X', wrists invert relative to shoulders (product < 0).
                avg_sh_y = (sh_l.y + sh_r.y) / 2.0
                chest_y_min = avg_sh_y - 0.05
                chest_y_max = avg_sh_y + 0.30
                delta_sh = sh_l.x - sh_r.x
                delta_wr = wr_l.x - wr_r.x
                if (delta_wr * delta_sh < -0.005) and (chest_y_min < wr_l.y < chest_y_max) and (chest_y_min < wr_r.y < chest_y_max):
                    detected = "RELEASE"

            # Gesture 3: FOLLOW_ME (One hand raised high above head/nose)
            if detected == "NONE":
                if wr_r_vis and (wr_r.y < nose.y - 0.04):
                    detected = "FOLLOW_ME"
                elif wr_l_vis and (wr_l.y < nose.y - 0.04):
                    detected = "FOLLOW_ME"

            # Gesture 4: WAIT (FollowMe paper: Open palm / hand held upright at chest/shoulder level)
            if detected == "NONE":
                if wr_r_vis and el_r_vis and (wr_r.y >= nose.y - 0.04) and (wr_r.y < sh_r.y + 0.05) and (wr_r.y < el_r.y):
                    detected = "WAIT"
                elif wr_l_vis and el_l_vis and (wr_l.y >= nose.y - 0.04) and (wr_l.y < sh_l.y + 0.05) and (wr_l.y < el_l.y):
                    detected = "WAIT"

        # Debounce to prevent false triggers from noisy frames
        is_new_event = False
        if detected != "NONE":
            if detected == self.current_candidate:
                self.candidate_count += 1
                if self.candidate_count >= self.debounce_frames and self.active_gesture != detected:
                    self.active_gesture = detected
                    is_new_event = True
            else:
                self.current_candidate = detected
                self.candidate_count = 1
        else:
            self._reset_candidate()
            if self.active_gesture != "NONE":
                self.active_gesture = "NONE"

        return self.active_gesture, is_new_event

    def _reset_candidate(self):
        self.current_candidate = None
        self.candidate_count = 0
