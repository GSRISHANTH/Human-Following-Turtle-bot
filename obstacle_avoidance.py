"""
Obstacle Avoidance & Path Deflection Module (Phase 2):
Implements Vision-Based Artificial Potential Fields (APF) and reactive deflection.
Scans floor-plane driving corridors (Left, Center, Right) for obstacles (chairs, furniture, walls)
while masking out the tracked human to ensure uninterrupted following.
"""

import math
import cv2
import numpy as np

class ObstacleAvoidance:
    def __init__(self, safe_margin=0.90, critical_dist=0.50):
        self.safe_margin = safe_margin
        self.critical_dist = critical_dist
        self.last_bias = 1.0  # Remember deflection direction to avoid rapid oscillating

    def scan_visual_corridors(self, frame, human_bboxes=None):
        """
        Scans ground plane in front of the robot for physical obstacles (chairs, boxes, walls).
        Masks out all detected humans so their body/legs/shoes are not treated as an obstacle.
        Uses vertical edge filtering (Sobel X) to detect 3D upright obstacles while
        rejecting flat horizontal floor tile seams and reflections.
        Returns:
            obstacle_distances = {'front': float, 'left': float, 'right': float}
            corridor_status = {'left': str, 'center': str, 'right': str}
            corridor_boxes = list of (x1, y1, x2, y2, color, label)
        """
        h, w = frame.shape[:2]
        
        # Floor inspection region (bottom 38% of the camera view)
        y_top = int(h * 0.60)
        y_bottom = int(h * 0.95)
        floor_roi = frame[y_top:y_bottom, 0:w]
        
        if floor_roi.size == 0:
            return {"front": 2.5, "left": 2.5, "right": 2.5}, {}, []

        # Convert to grayscale and blur slightly to suppress fine surface grain
        gray = cv2.cvtColor(floor_roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Sobel vertical gradient: Isolates upright 3D structures (chair legs, boxes, trash bins, walls)
        # while rejecting flat horizontal tile grout lines, carpet seams, and floor lighting glare.
        sobel_v = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        sobel_abs = np.uint8(np.clip(np.absolute(sobel_v), 0, 255))
        _, edges = cv2.threshold(sobel_abs, 32, 255, cv2.THRESH_BINARY)

        # Mask out ALL detected human positions so legs/shoes are NEVER treated as obstacles
        if human_bboxes is not None:
            bboxes_to_mask = human_bboxes if isinstance(human_bboxes, list) else [human_bboxes]
            for hb in bboxes_to_mask:
                if hb is not None and len(hb) == 4:
                    hx1, hy1, hx2, hy2 = hb
                    mask_x1 = max(0, int(hx1 - 25))
                    mask_x2 = min(w, int(hx2 + 25))
                    edges[:, mask_x1:mask_x2] = 0

        # Divide floor region into 3 spatial corridors
        x_c1 = int(w * 0.33)  # Left | Center boundary
        x_c2 = int(w * 0.67)  # Center | Right boundary

        edges_left = edges[:, 0:x_c1]
        edges_center = edges[:, x_c1:x_c2]
        edges_right = edges[:, x_c2:w]

        dens_l = float(np.sum(edges_left > 0)) / max(1.0, float(edges_left.size))
        dens_c = float(np.sum(edges_center > 0)) / max(1.0, float(edges_center.size))
        dens_r = float(np.sum(edges_right > 0)) / max(1.0, float(edges_right.size))

        # Convert edge density into approximate distance (higher density = closer obstacle)
        def _density_to_dist(dens):
            if dens < 0.025:
                return 2.50
            return max(0.35, min(2.50, 0.12 / max(0.01, dens)))

        d_left = _density_to_dist(dens_l)
        d_front = _density_to_dist(dens_c)
        d_right = _density_to_dist(dens_r)

        # Corridor visual status
        corridor_boxes = []
        for name, x1, x2, dist in [
            ("LEFT", 0, x_c1, d_left),
            ("CENTER", x_c1, x_c2, d_front),
            ("RIGHT", x_c2, w, d_right)
        ]:
            if dist < self.critical_dist:
                color = (0, 0, 255)      # Red = Blocked
            elif dist < self.safe_margin:
                color = (0, 180, 255)    # Amber = Warning
            else:
                color = (0, 255, 100)    # Green = Clear
            corridor_boxes.append((x1, y_top, x2, y_bottom, color, f"{name}: {dist:.1f}m"))

        obstacle_distances = {"front": d_front, "left": d_left, "right": d_right}
        return obstacle_distances, corridor_boxes

    def compute_safe_steering(self, target_offset_norm, obstacle_distances=None):
        """
        Computes safe angular steering and linear speed adjustment.
        target_offset_norm: Normalized horizontal offset to human (-1.0 to +1.0)
        obstacle_distances: dict {'front': float, 'left': float, 'right': float}
        Returns: (safe_angular_command, speed_scale, status_string)
        """
        # Base steering towards the human:
        # Boosted gain & friction-breaking impulse for immediate, snappy reaction to human movement
        # ROS 2 convention: +w_z = Turn Left (CCW), -w_z = Turn Right (CW).
        # When target is to the right (target_offset_norm > 0), robot must turn Right (-w_z).
        # When target is to the left (target_offset_norm < 0), robot must turn Left (+w_z).
        if abs(target_offset_norm) < 0.04:  # Centered deadzone (±4%)
            attraction_w = 0.0
        else:
            turn_mag = max(0.24, min(1.40, abs(target_offset_norm) * 2.2))
            attraction_w = -math.copysign(turn_mag, target_offset_norm)

        if obstacle_distances is None:
            obstacle_distances = {"front": 2.5, "left": 2.5, "right": 2.5}

        d_front = obstacle_distances.get("front", 2.5)
        d_left = obstacle_distances.get("left", 2.5)
        d_right = obstacle_distances.get("right", 2.5)

        # 1. Critical proximity condition (Obstacle directly ahead < 0.50m)
        if d_front < self.critical_dist:
            # Pick direction with more clearance
            if d_left > d_right + 0.15:
                self.last_bias = 1.0   # Deflect Left (+w_z in ROS)
            elif d_right > d_left + 0.15:
                self.last_bias = -1.0  # Deflect Right (-w_z in ROS)
            deflection_w = self.last_bias * 0.85
            # Stop forward movement to prevent collision
            return deflection_w, 0.0, "OBSTACLE_CRITICAL_STOP"

        # 2. Warning zone (Obstacle ahead < 0.90m)
        if d_front < self.safe_margin:
            # Repulsive force proportional to proximity
            repulsion_gain = (self.safe_margin - d_front) / (self.safe_margin - self.critical_dist)
            
            # Choose deflection away from the obstacle towards open space
            if d_left >= d_right:
                bias = 1.0   # Steer Left (+w_z in ROS)
            else:
                bias = -1.0  # Steer Right (-w_z in ROS)
            self.last_bias = bias

            repulsion_w = bias * repulsion_gain * 0.90
            
            # Blend attraction towards human with obstacle repulsion
            safe_w = 0.40 * attraction_w + 0.60 * repulsion_w
            
            # Slow down forward velocity while maneuvering around obstacle
            speed_scale = max(0.35, (d_front - self.critical_dist) / (self.safe_margin - self.critical_dist))
            return safe_w, speed_scale, "OBSTACLE_AVOIDING"

        # 3. Lateral clearance protection (Obstacles to the left or right side)
        lateral_margin = 0.75
        lateral_w = 0.0
        if d_left < lateral_margin and d_left < d_right:
            # Obstacle on left -> push right (-w_z in ROS)
            repulse = (lateral_margin - d_left) / lateral_margin
            lateral_w = -0.65 * repulse
        elif d_right < lateral_margin and d_right < d_left:
            # Obstacle on right -> push left (+w_z in ROS)
            repulse = (lateral_margin - d_right) / lateral_margin
            lateral_w = +0.65 * repulse

        if abs(lateral_w) > 0.05:
            safe_w = 0.50 * attraction_w + 0.50 * lateral_w
            return safe_w, 0.75, "OBSTACLE_CLEARING_SIDE"

        # 4. Path clear -> Direct human following
        return attraction_w, 1.0, "PATH_CLEAR"
