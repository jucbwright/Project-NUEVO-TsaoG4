from __future__ import annotations

import cv2
import numpy as np

from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject


def detect_zombie_green(
    frame_bgr: np.ndarray,
) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect zombie-green blobs and return detections plus debug contours."""
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Zombie green HSV range — matches classify_person_color in vision_node.py.
    # High saturation floor rejects muted teal walls and blue clothing.
    green_hsv_low  = np.array([45, 120,  60])
    green_hsv_high = np.array([75, 255, 255])
    mask = cv2.inRange(hsv, green_hsv_low, green_hsv_high)

    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    # Slightly looser fill ratio than yellow block — zombie silhouette is irregular
    min_area_px    = 500
    min_fill_ratio = 0.15

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area_px:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        bounding_box_area = float(max(1, width * height))
        fill_ratio = contour_area / bounding_box_area
        if fill_ratio < min_fill_ratio:
            continue

        confidence = _green_detection_score(contour_area, min_area_px, fill_ratio)
        detection = DetectedObject(
            class_name="zombie",
            confidence=confidence,
            x=int(x),
            y=int(y),
            width=int(width),
            height=int(height),
        )
        detection.add_attribute("color", "green", 1.0)
        detections.append(detection)
        debug_overlays.append(
            DebugOverlay(
                color=(30, 180, 30),  # BGR — zombie shade of green
                contour=contour,
                label="zombie",
                x=int(x),
                y=int(y),
            )
        )

    return detections, debug_overlays


def _green_detection_score(contour_area: float, min_area_px: int, fill_ratio: float) -> float:
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 4)))
    score = 0.55 * area_score + 0.45 * max(0.0, min(1.0, fill_ratio))
    return max(0.0, min(1.0, score))


def classify_person_color(person_crop: np.ndarray) -> tuple[str, float]:
    """
    Check if a person crop is predominantly green.
    Returns ("green", score) or ("not_green", 0.0).
    25% green pixels threshold — tunable for your lighting conditions.
    """
    if person_crop.size == 0:
        return "unknown", 0.0

    blurred = cv2.GaussianBlur(person_crop, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    green_hsv_low  = (40, 60, 60)
    green_hsv_high = (80, 255, 255)
    mask = cv2.inRange(hsv, green_hsv_low, green_hsv_high)

    green_pixels  = float(np.count_nonzero(mask))
    total_pixels  = float(max(1, mask.shape[0] * mask.shape[1]))
    green_fraction = green_pixels / total_pixels

    if green_fraction >= 0.25:
        return "green", min(1.0, green_fraction * 3.0)
    return "not_green", 0.0
