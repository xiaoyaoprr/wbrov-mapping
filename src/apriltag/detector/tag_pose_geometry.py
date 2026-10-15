"""Stateless square-tag pose estimation with an explicit planar ambiguity gate.

No ROS, temporal reference, device access, pose smoothing or fallback to an old
pose.  Pixel/angle thresholds below are conservative trial settings, not an
underwater accuracy calibration.  IPPE exposes ambiguity; it cannot resolve it
without additional information.  An ambiguous observation is withheld.
"""
import math

import cv2
import numpy as np


def square_object_points(tag_size):
    """OpenCV IPPE_SQUARE order, matching ArUco's decoded corner order."""
    half = float(tag_size) * 0.5
    if not math.isfinite(half) or half <= 0.0:
        raise ValueError("tag_size must be finite and positive")
    return np.asarray([[-half, half, 0.0], [half, half, 0.0],
                       [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)


def rotation_distance_deg(first, second):
    cosine = (float(np.trace(first.T @ second)) - 1.0) * 0.5
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def solve_square_pose(image_points, camera_matrix, distortion, tag_size, *,
                      max_reprojection_px=3.0, min_side_px=18.0,
                      ambiguity_angle_deg=15.0, ambiguity_error_gap_px=0.5,
                      ambiguity_error_ratio=2.0):
    """Return accepted/reason/best plus independently recomputed diagnostics.

    A significantly different second rotation is ambiguous if its reprojection
    error is within the absolute noise allowance OR within the error ratio.
    Ratios are compared by multiplication, including zero-error observations.
    Cheirality uses all four corners, not just the marker centre.  Distortion
    and intrinsics must belong to the supplied image; neither is modified.
    """
    config = (max_reprojection_px, min_side_px, ambiguity_angle_deg,
              ambiguity_error_gap_px, ambiguity_error_ratio)
    if (not all(math.isfinite(float(value)) for value in config) or
            max_reprojection_px <= 0 or min_side_px <= 0 or
            not 0 < ambiguity_angle_deg < 180 or ambiguity_error_gap_px < 0 or
            ambiguity_error_ratio < 1):
        raise ValueError("invalid square-pose gate configuration")
    result = {"accepted": False, "reason": "invalid_square_geometry", "best": None,
              "candidate_count": 0, "solution_count": 0}
    try:
        points = np.ascontiguousarray(image_points, dtype=np.float64).reshape(4, 2)
        matrix = np.ascontiguousarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        coefficients = (None if distortion is None else
                        np.ascontiguousarray(distortion, dtype=np.float64).reshape(-1, 1))
        objects = square_object_points(tag_size)
    except (ValueError, TypeError, OverflowError):
        return result
    if (not np.isfinite(points).all() or not np.isfinite(matrix).all() or
            (coefficients is not None and not np.isfinite(coefficients).all()) or
            matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or
            not np.allclose(matrix[2], [0., 0., 1.], rtol=0.0, atol=1e-9)):
        return result
    edges = np.roll(points, -1, axis=0) - points
    if float(np.min(np.linalg.norm(edges, axis=1))) < min_side_px:
        result["reason"] = "tag_too_small"
        return result
    # Decoded ArUco corners are clockwise in image coordinates (+y down).
    # Reject reversed, collinear, repeated and bow-tie quadrilaterals.
    following = np.roll(edges, -1, axis=0)
    crosses = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not np.all(crosses > 1e-9):
        return result
    try:
        solutions = cv2.solvePnPGeneric(
            objects, points, matrix, coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except (cv2.error, ValueError, TypeError, OverflowError):
        result["reason"] = "square_solver_failed"
        return result
    if not solutions or len(solutions) < 3 or not solutions[0]:
        result["reason"] = "square_solver_failed"
        return result
    rvecs, tvecs = solutions[1], solutions[2]
    result["solution_count"] = len(rvecs)
    if len(rvecs) != 2 or len(tvecs) != 2:
        # A single root does not establish that a planar alternative is absent.
        result["reason"] = "square_solver_incomplete"
        return result
    candidates = []
    for raw_rvec, raw_tvec in zip(rvecs, tvecs):
        try:
            rvec = np.asarray(raw_rvec, dtype=np.float64).reshape(3).copy()
            translation = np.asarray(raw_tvec, dtype=np.float64).reshape(3).copy()
            if not (np.isfinite(rvec).all() and np.isfinite(translation).all()):
                result["reason"] = "square_solver_nonfinite"
                return result
            rotation, _ = cv2.Rodrigues(rvec)
            if (not np.isfinite(rotation).all() or
                    not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0) or
                    not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-7)):
                result["reason"] = "square_solver_invalid_rotation"
                return result
            transformed = objects @ rotation.T + translation
            if not np.all(transformed[:, 2] > 1e-9):
                continue
            projected, _ = cv2.projectPoints(objects, rvec, translation, matrix, coefficients)
            residuals = projected.reshape(4, 2) - points
            reprojection = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
            if not math.isfinite(reprojection):
                result["reason"] = "square_solver_nonfinite"
                return result
            candidates.append({"rvec": rvec, "translation": translation,
                               "rotation": rotation, "reprojection": reprojection})
        except (cv2.error, ValueError, TypeError, OverflowError):
            result["reason"] = "square_solver_failed"
            return result
    candidates.sort(key=lambda candidate: candidate["reprojection"])
    result["candidate_count"] = len(candidates)
    if not candidates:
        result["reason"] = "square_behind_camera"
        return result
    best = candidates[0]
    result["best"] = best
    if best["reprojection"] > max_reprojection_px:
        result["reason"] = "square_reprojection_rejected"
        return result
    if len(candidates) > 1:
        other = candidates[1]
        angle = rotation_distance_deg(best["rotation"], other["rotation"])
        gap = other["reprojection"] - best["reprojection"]
        result.update(second_reprojection_px=other["reprojection"],
                      solution_angle_deg=angle, reprojection_gap_px=gap)
        close_error = (gap <= ambiguity_error_gap_px + 1e-12 or
                       other["reprojection"] <= best["reprojection"] * ambiguity_error_ratio + 1e-12)
        if angle + 1e-8 >= ambiguity_angle_deg and close_error:
            result["reason"] = "ambiguous_square_pose"
            return result
    result.update(accepted=True, reason="square_pose_unambiguous")
    return result
