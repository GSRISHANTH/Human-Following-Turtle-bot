"""
Appearance Model & Biometric Re-Identification Module:
Extracts normalized HSV color histograms from person torso and head/face patches,
along with anatomical proportions for owner recognition, multi-person disambiguation,
and re-identifying the designated owner after occlusion or tracking loss.
"""

import os
import pickle
import cv2
import numpy as np


class AppearanceModel:
    def __init__(self, h_bins=18, s_bins=16):
        self.h_bins = h_bins
        self.s_bins = s_bins

    def extract_signature(self, image, bbox, landmarks=None):
        """
        Extracts appearance and biometric features (Torso HSV + Head HSV + Proportions).
        bbox: [x1, y1, x2, y2]
        landmarks: dictionary of points or list of landmarks
        """
        if image is None or image.size == 0:
            return None

        h, w = image.shape[:2]
        x1, y1, x2, y2 = bbox
        bw = max(10, x2 - x1)
        bh = max(10, y2 - y1)

        # Convert landmarks to dictionary format if passed as a list
        lm_dict = None
        if landmarks is not None:
            if isinstance(landmarks, dict):
                lm_dict = landmarks
            elif hasattr(landmarks, "__iter__"):
                lm_dict = {}
                for idx, pt in enumerate(landmarks):
                    if hasattr(pt, "x") and hasattr(pt, "y"):
                        lm_dict[idx] = (int(pt.x * w), int(pt.y * h), getattr(pt, "visibility", 1.0))

        # --- 1. TORSO PATCH EXTRACTION ---
        torso_patch = None
        torso_ratio = 1.0
        if lm_dict:
            sh_l = lm_dict.get(11)  # Left shoulder
            sh_r = lm_dict.get(12)  # Right shoulder
            hip_l = lm_dict.get(23) # Left hip
            hip_r = lm_dict.get(24) # Right hip
            if sh_l and sh_r and sh_l[2] > 0.35 and sh_r[2] > 0.35:
                sx1 = min(sh_l[0], sh_r[0])
                sx2 = max(sh_l[0], sh_r[0])
                sy1 = min(sh_l[1], sh_r[1])
                pad_x = max(8, int((sx2 - sx1) * 0.15))
                torso_x1 = max(0, sx1 - pad_x)
                torso_x2 = min(w, sx2 + pad_x)
                torso_y1 = max(0, sy1)
                if hip_l and hip_r and hip_l[2] > 0.25 and hip_r[2] > 0.25:
                    torso_y2 = min(h, max(hip_l[1], hip_r[1]))
                else:
                    torso_y2 = min(h, int(sy1 + (sx2 - sx1) * 1.35))

                if torso_y2 > torso_y1 and torso_x2 > torso_x1:
                    torso_patch = image[torso_y1:torso_y2, torso_x1:torso_x2]
                    raw_ratio = (torso_y2 - torso_y1) / max(10, torso_x2 - torso_x1)
                    torso_ratio = round(max(0.75, min(2.20, raw_ratio)), 2)

        if torso_patch is None or torso_patch.size == 0:
            torso_y1 = max(0, int(y1 + bh * 0.18))
            torso_y2 = min(h, int(y1 + bh * 0.62))
            torso_x1 = max(0, int(x1 + bw * 0.15))
            torso_x2 = min(w, int(x2 - bw * 0.15))
            if torso_y2 <= torso_y1 or torso_x2 <= torso_x1:
                torso_patch = image[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
            else:
                torso_patch = image[torso_y1:torso_y2, torso_x1:torso_x2]

        if torso_patch is None or torso_patch.size == 0:
            return None

        # Torso HSV Histogram
        torso_hsv = cv2.cvtColor(torso_patch, cv2.COLOR_BGR2HSV)
        torso_hist = cv2.calcHist(
            [torso_hsv],
            [0, 1],
            None,
            [self.h_bins, self.s_bins],
            [0, 180, 25, 256]
        )
        cv2.normalize(torso_hist, torso_hist, alpha=1.0, norm_type=cv2.NORM_L1)

        avg_bgr = np.mean(torso_patch, axis=(0, 1))
        r, g, b = int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0])
        color_hex = f"#{r:02x}{g:02x}{b:02x}"

        # --- 2. HEAD / FACE PATCH EXTRACTION ---
        head_patch = None
        head_hist = None
        head_hex = "#ffcc99"
        if lm_dict:
            nose = lm_dict.get(0)
            sh_l = lm_dict.get(11)
            sh_r = lm_dict.get(12)
            if nose and nose[2] > 0.35 and sh_l and sh_r:
                head_w = int(abs(sh_r[0] - sh_l[0]) * 0.65)
                head_x1 = max(0, nose[0] - head_w // 2)
                head_x2 = min(w, nose[0] + head_w // 2)
                head_y2 = min(h, int((sh_l[1] + sh_r[1]) / 2))
                head_y1 = max(0, head_y2 - int(head_w * 1.3))
                if head_y2 > head_y1 and head_x2 > head_x1:
                    head_patch = image[head_y1:head_y2, head_x1:head_x2]

        if head_patch is None or head_patch.size == 0:
            # Fallback to top 22% of bounding box
            top_y2 = min(h, int(y1 + bh * 0.22))
            top_x1 = max(0, int(x1 + bw * 0.25))
            top_x2 = min(w, int(x2 - bw * 0.25))
            if top_y2 > y1 and top_x2 > top_x1:
                head_patch = image[max(0, y1):top_y2, top_x1:top_x2]

        if head_patch is not None and head_patch.size > 20:
            head_hsv = cv2.cvtColor(head_patch, cv2.COLOR_BGR2HSV)
            head_hist = cv2.calcHist(
                [head_hsv],
                [0, 1],
                None,
                [self.h_bins, self.s_bins],
                [0, 180, 20, 256]
            )
            cv2.normalize(head_hist, head_hist, alpha=1.0, norm_type=cv2.NORM_L1)
            avg_head = np.mean(head_patch, axis=(0, 1))
            hr, hg, hb = int(avg_head[2]), int(avg_head[1]), int(avg_head[0])
            head_hex = f"#{hr:02x}{hg:02x}{hb:02x}"

        return {
            "hist": torso_hist,
            "head_hist": head_hist,
            "color_hex": color_hex,
            "head_hex": head_hex,
            "aspect_ratio": round(bw / max(1, bh), 2),
            "torso_ratio": torso_ratio,
            "area": bw * bh
        }

    def compute_similarity(self, sig1, sig2):
        """
        Computes comprehensive similarity between two signatures.
        Combines torso color, head/face color, and bodily proportions.
        Returns score from 0.0 to 1.0.
        """
        if sig1 is None or sig2 is None:
            return 0.0

        hist1 = sig1.get("hist")
        hist2 = sig2.get("hist")
        if hist1 is None or hist2 is None:
            return 0.0

        # 1. Torso color similarity (Dual Bhattacharyya + Correlation metric)
        bhatt_torso = cv2.compareHist(hist1, hist2, cv2.HISTCMP_BHATTACHARYYA)
        correl_torso = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
        color_sim = 0.50 * max(0.0, 1.0 - bhatt_torso) + 0.50 * max(0.0, correl_torso)

        # 2. Head/face color similarity
        head_sim = 0.50
        h_hist1 = sig1.get("head_hist")
        h_hist2 = sig2.get("head_hist")
        if h_hist1 is not None and h_hist2 is not None:
            bhatt_head = cv2.compareHist(h_hist1, h_hist2, cv2.HISTCMP_BHATTACHARYYA)
            correl_head = cv2.compareHist(h_hist1, h_hist2, cv2.HISTCMP_CORREL)
            head_sim = 0.50 * max(0.0, 1.0 - bhatt_head) + 0.50 * max(0.0, correl_head)
        else:
            # If head is not visible, use torso similarity
            head_sim = color_sim

        # 3. Body proportion consistency
        ar1 = sig1.get("aspect_ratio", 0.4)
        ar2 = sig2.get("aspect_ratio", 0.4)
        ar_ratio = min(ar1, ar2) / max(0.01, max(ar1, ar2))

        tr1 = sig1.get("torso_ratio", 1.0)
        tr2 = sig2.get("torso_ratio", 1.0)
        tr_ratio = min(tr1, tr2) / max(0.01, max(tr1, tr2))
        geom_sim = 0.60 * ar_ratio + 0.40 * tr_ratio

        # Weighted combination: 70% torso clothing, 18% head/face, 12% geometry
        total_sim = 0.70 * color_sim + 0.18 * head_sim + 0.12 * geom_sim
        # If torso clothing is a strong match, don't let head turn or aspect ratio drag it down
        if color_sim >= 0.45:
            total_sim = max(total_sim, 0.88 * color_sim)
        return round(float(total_sim), 3)

    def blend_signatures(self, target_sig, new_sig, alpha=0.10):
        """
        Gently updates target appearance model using EMA to adapt to lighting
        changes while avoiding identity drift.
        """
        if target_sig is None:
            return new_sig
        if new_sig is None:
            return target_sig

        blended_hist = (1.0 - alpha) * target_sig["hist"] + alpha * new_sig["hist"]
        cv2.normalize(blended_hist, blended_hist, alpha=1.0, norm_type=cv2.NORM_L1)

        blended_head = target_sig.get("head_hist")
        if target_sig.get("head_hist") is not None and new_sig.get("head_hist") is not None:
            blended_head = (1.0 - alpha) * target_sig["head_hist"] + alpha * new_sig["head_hist"]
            cv2.normalize(blended_head, blended_head, alpha=1.0, norm_type=cv2.NORM_L1)
        elif new_sig.get("head_hist") is not None:
            blended_head = new_sig.get("head_hist")

        return {
            "hist": blended_hist,
            "head_hist": blended_head,
            "color_hex": new_sig.get("color_hex", target_sig.get("color_hex")),
            "head_hex": new_sig.get("head_hex", target_sig.get("head_hex")),
            "aspect_ratio": (1.0 - alpha) * target_sig["aspect_ratio"] + alpha * new_sig["aspect_ratio"],
            "torso_ratio": (1.0 - alpha) * target_sig.get("torso_ratio", 1.0) + alpha * new_sig.get("torso_ratio", 1.0),
            "area": new_sig.get("area", target_sig.get("area"))
        }

    def create_owner_profile(self, signature, owner_id=1):
        """Creates an immutable-baseline multi-template owner identity structure"""
        import time
        if signature is None:
            return None
        return {
            "owner_id": owner_id,
            "protected_template": signature.copy(),
            "templates": [signature.copy()],
            "color_hex": signature.get("color_hex", "#00f0ff"),
            "head_hex": signature.get("head_hex", "#ffcc99"),
            "enrollment_time": time.time(),
            "locked": True,
            "following_enabled": False
        }

    def compute_owner_match(self, owner_profile, candidate_sig):
        """
        Matches a candidate signature against the protected enrollment baseline
        and the multi-template bank. Returns the highest similarity score.
        """
        if owner_profile is None or candidate_sig is None:
            return 0.0

        # Legacy fallback if profile is a single signature dictionary
        if "protected_template" not in owner_profile:
            return self.compute_similarity(owner_profile, candidate_sig)

        best_score = self.compute_similarity(owner_profile["protected_template"], candidate_sig)
        for t in owner_profile.get("templates", []):
            score = self.compute_similarity(t, candidate_sig)
            if score > best_score:
                best_score = score
        return best_score

    def add_owner_template(self, owner_profile, candidate_sig, match_score, min_conf=0.75, max_templates=5):
        """
        Adds a new multi-view appearance template without corrupting the protected baseline.
        """
        if owner_profile is None or candidate_sig is None:
            return
        if match_score < min_conf:
            return

        templates = owner_profile.setdefault("templates", [])
        if len(templates) < max_templates:
            templates.append(candidate_sig.copy())
        else:
            # Gently blend into the oldest secondary template (never the protected baseline)
            templates[1] = self.blend_signatures(templates[1], candidate_sig, alpha=0.08)

    def check_ambiguity(self, scores, match_thresh=0.40, delta_thresh=0.08):
        """
        Determines if two or more candidates have indistinguishable similarity to the Owner.
        Returns (is_ambiguous, top_score).
        """
        if not scores or len(scores) < 2:
            return False, (scores[0] if scores else 0.0)
        sorted_s = sorted(scores, reverse=True)
        if sorted_s[0] >= match_thresh and sorted_s[1] >= match_thresh:
            if (sorted_s[0] - sorted_s[1]) < delta_thresh:
                return True, sorted_s[0]
        return False, sorted_s[0]

    def save_profile(self, profile, filepath):
        """Persists owner profile to disk"""
        try:
            with open(filepath, "wb") as f:
                pickle.dump(profile, f)
            return True
        except Exception as e:
            print(f"[AppearanceModel] Error saving profile to {filepath}: {e}")
            return False

    def load_profile(self, filepath):
        """Loads owner profile from disk"""
        if not os.path.exists(filepath):
            return None
        try:
            with open(filepath, "rb") as f:
                return pickle.load(f)
        except Exception as e:
            print(f"[AppearanceModel] Error loading profile from {filepath}: {e}")
            return None
