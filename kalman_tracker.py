"""
Kalman Filter State Estimator & Motion Predictor:
Estimates person 2D trajectory [cx, cy, vx, vy], smooths bounding box,
and projects motion during temporary occlusion / lost-human recovery.
"""

import cv2
import numpy as np

class PersonKalmanTracker:
    def __init__(self, initial_bbox):
        """
        initial_bbox: [x1, y1, x2, y2]
        State vector: [cx, cy, vx, vy, w, h]^T (6 variables)
        Measurement vector: [cx, cy, w, h]^T (4 variables)
        """
        self.kf = cv2.KalmanFilter(6, 4)
        
        # State Transition Matrix (A) with constant velocity model
        dt = 1.0 / 25.0 # ~25 FPS
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  0, 0],
            [0, 1, 0,  dt, 0, 0],
            [0, 0, 1,  0,  0, 0],
            [0, 0, 0,  1,  0, 0],
            [0, 0, 0,  0,  1, 0],
            [0, 0, 0,  0,  0, 1]
        ], dtype=np.float32)

        # Measurement Matrix (H)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)

        # Process Noise Covariance (Q)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-2
        self.kf.processNoiseCov[2, 2] = 5.0 # velocity x variance
        self.kf.processNoiseCov[3, 3] = 5.0 # velocity y variance

        # Measurement Noise Covariance (R)
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 1e-1

        # Error Covariance Post (P)
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)

        # Initialize state with first detection
        x1, y1, x2, y2 = initial_bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = max(10.0, float(x2 - x1))
        h = max(10.0, float(y2 - y1))

        self.kf.statePost = np.array([[cx], [cy], [0.0], [0.0], [w], [h]], dtype=np.float32)

        self.missed_frames = 0
        self.total_tracked_frames = 1
        self.last_predicted_bbox = initial_bbox

    def predict(self):
        """Advances state prediction step (used in normal tracking and during occlusion)"""
        pred = self.kf.predict()
        cx = float(pred[0, 0])
        cy = float(pred[1, 0])
        vx = float(pred[2, 0])
        vy = float(pred[3, 0])
        w = max(10.0, float(pred[4, 0]))
        h = max(10.0, float(pred[5, 0]))

        x1 = int(cx - w / 2.0)
        y1 = int(cy - h / 2.0)
        x2 = int(cx + w / 2.0)
        y2 = int(cy + h / 2.0)
        self.last_predicted_bbox = [x1, y1, x2, y2]
        return self.last_predicted_bbox, (vx, vy)

    def update(self, measurement_bbox):
        """Updates Kalman Filter with fresh detection measurement"""
        x1, y1, x2, y2 = measurement_bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = max(10.0, float(x2 - x1))
        h = max(10.0, float(y2 - y1))

        measurement = np.array([[cx], [cy], [w], [h]], dtype=np.float32)
        corrected = self.kf.correct(measurement)

        cx_c = float(corrected[0, 0])
        cy_c = float(corrected[1, 0])
        vx_c = float(corrected[2, 0])
        vy_c = float(corrected[3, 0])
        w_c = max(10.0, float(corrected[4, 0]))
        h_c = max(10.0, float(corrected[5, 0]))

        self.missed_frames = 0
        self.total_tracked_frames += 1
        corrected_bbox = [
            int(cx_c - w_c / 2.0),
            int(cy_c - h_c / 2.0),
            int(cx_c + w_c / 2.0),
            int(cy_c + h_c / 2.0)
        ]
        self.last_predicted_bbox = corrected_bbox
        return corrected_bbox, (vx_c, vy_c)

    def mark_missed(self):
        self.missed_frames += 1
