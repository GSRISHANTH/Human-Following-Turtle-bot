"""
Safety Layer Module:
Monitors critical conditions:
- Excessive proximity (< 0.5m)
- Ground-level visual obstacle / path collision hazards
- Prolonged tracking loss timeout
- Emergency halt states
"""

import cv2
import numpy as np

class SafetyMonitor:
    def __init__(self, min_safe_dist=0.55, caution_dist=0.85):
        self.min_safe_dist = min_safe_dist
        self.caution_dist = caution_dist
        self.emergency_stop_triggered = False

    def evaluate_safety(self, frame, distance_m, target_status, manual_stop_gesture=False):
        """
        Evaluates CV-based safety metrics.
        Returns safety report dictionary.
        """
        h, w = frame.shape[:2]
        alerts = []
        status = "SAFE"
        color = (0, 255, 120) # Green

        # 1. Gesture emergency stop check
        if manual_stop_gesture:
            status = "EMERGENCY_STOP_GESTURE"
            alerts.append("HRI: Human issued STOP gesture")
            color = (0, 50, 255) # Red
            return {"status": status, "color": color, "alerts": alerts, "can_move": False}

        # 2. Excessive human proximity check
        if distance_m > 0.0:
            if distance_m < self.min_safe_dist:
                status = "CRITICAL_PROXIMITY_STOP"
                alerts.append(f"Proximity Hazard: Human too close ({distance_m:.2f}m < {self.min_safe_dist}m)")
                color = (0, 50, 255)
            elif distance_m < self.caution_dist:
                status = "CAUTION_PROXIMITY"
                alerts.append(f"Caution: Approaching boundary ({distance_m:.2f}m)")
                color = (0, 200, 255)

        # 3. Visual ground-level collision hazard check (bottom 20% of frame)
        ground_roi = frame[int(h * 0.80):h, int(w * 0.25):int(w * 0.75)]
        if ground_roi.size > 0:
            gray_ground = cv2.cvtColor(ground_roi, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_ground, 80, 160)
            edge_density = float(np.sum(edges > 0)) / float(edges.size)
            # High edge concentration directly in front of camera indicates obstacle
            if edge_density > 0.16 and distance_m < 1.0:
                alerts.append("Visual Hazard: Immediate low-profile obstacle in drive path")
                if status == "SAFE":
                    status = "OBSTACLE_WARNING"
                    color = (0, 180, 255)

        # 4. Tracking state check
        if target_status == "WAIT":
            status = "WAIT_STATE_HOLD"
            alerts.append("HRI: Robot paused in WAIT state by human gesture")
            color = (255, 180, 0)
            return {"status": status, "color": color, "alerts": alerts, "can_move": False, "min_safe_dist": self.min_safe_dist}
        elif target_status in ["LOST", "SEARCHING"]:
            if status == "SAFE":
                status = "TARGET_LOST_HOLD"
                alerts.append("Target Lost: Robot stopped, running recovery")
                color = (255, 180, 0)

        can_move = status in ["SAFE", "CAUTION_PROXIMITY", "OBSTACLE_WARNING"]

        return {
            "status": status,
            "color": color,
            "alerts": alerts,
            "can_move": can_move,
            "min_safe_dist": self.min_safe_dist
        }
