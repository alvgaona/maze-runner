import numpy as np
import cv2


def extract_map_lines(binary_map: np.ndarray, resolution: float = 10.0) -> np.ndarray:
    """
    Extract horizontal and vertical lines from a binary map using Hough transform.

    Args:
        binary_map: 2D binary image (numpy array)
        resolution: Pixels per meter (default: 10)

    Returns:
        Array of shape (N, 2) with [alpha, distance] for each line in Hesse form
    """
    # Edge Detection
    edges = cv2.Canny(binary_map.astype(np.uint8) * 255, 50, 150)

    # Hough Transform using probabilistic method (similar to MATLAB's houghlines)
    # minLineLength and maxLineGap to match MATLAB's 'MinLength', 30, 'FillGap', 20
    hough_lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=50,
        minLineLength=30,
        maxLineGap=20
    )

    if hough_lines is None:
        print("Found 0 line segments")
        return np.array([]).reshape(0, 2)

    print(f"Found {len(hough_lines)} line segments")

    # Convert line segments to Hesse form and filter horizontal/vertical only
    map_lines = []
    angle_tolerance = np.deg2rad(1)  # 1 degree tolerance for horizontal/vertical

    for line in hough_lines:
        x1, y1, x2, y2 = line[0]

        # Calculate angle of the line
        dx = x2 - x1
        dy = y2 - y1

        # Angle perpendicular to the line (Hesse form convention)
        theta_rad = np.arctan2(dy, dx) + np.pi / 2

        # Normalize to [0, pi]
        alpha = theta_rad % np.pi

        # Filter: Keep only horizontal or vertical lines
        # Horizontal lines: α near 0 or π
        # Vertical lines: α near π/2
        is_horizontal = abs(alpha) < angle_tolerance or abs(alpha - np.pi) < angle_tolerance
        is_vertical = abs(alpha - np.pi / 2) < angle_tolerance

        # Skip diagonal lines
        if not (is_horizontal or is_vertical):
            continue

        # Snap angle to exactly 0 or π/2 for consistent merging
        if is_horizontal:
            alpha = 0.0
        else:  # is_vertical
            alpha = np.pi / 2

        # Calculate distance from origin to line (Hesse form)
        # Using point (x1, y1) on the line: d = x1*cos(alpha) + y1*sin(alpha)
        rho_pix = abs(x1 * np.cos(alpha) + y1 * np.sin(alpha))

        # Convert distance from pixels to meters
        d = rho_pix / resolution

        map_lines.append([alpha, d])

    print(f"Extracted {len(map_lines)} horizontal/vertical line segments")

    if len(map_lines) == 0:
        return np.array([]).reshape(0, 2)

    map_lines = np.array(map_lines)
    map_lines = merge_similar_lines(map_lines, np.deg2rad(1), 0.2)

    return map_lines


def angdiff(a: float, b: float) -> float:
    """Compute the smallest signed angle difference between two angles."""
    diff = a - b
    return np.arctan2(np.sin(diff), np.cos(diff))


def merge_similar_lines(lines: np.ndarray, angle_threshold: float, dist_threshold: float) -> np.ndarray:
    """
    Merge lines that are similar in parameter space.

    Args:
        lines: Array of shape (N, 2) with [alpha, distance] for each line
        angle_threshold: Maximum angle difference to consider lines similar (radians)
        dist_threshold: Maximum distance difference to consider lines similar (meters)

    Returns:
        Merged lines array of shape (M, 2)
    """
    if lines.size == 0:
        return np.array([]).reshape(0, 2)

    lines = lines.copy()

    # Normalize angles to [0, pi]
    lines[:, 0] = lines[:, 0] % np.pi

    keep = np.ones(len(lines), dtype=bool)

    for i in range(len(lines)):
        if not keep[i]:
            continue

        for j in range(i + 1, len(lines)):
            if not keep[j]:
                continue

            # Angle difference
            angle_diff = abs(angdiff(lines[i, 0], lines[j, 0]))

            # Check flipped version too
            angle_diff_flip = abs(angdiff(lines[i, 0], (lines[j, 0] + np.pi) % (2 * np.pi)))

            angle_similar = min(angle_diff, angle_diff_flip) < angle_threshold
            dist_similar = abs(lines[i, 1] - lines[j, 1]) < dist_threshold

            if angle_similar and dist_similar:
                # Average and merge
                lines[i, :] = (lines[i, :] + lines[j, :]) / 2
                keep[j] = False

    return lines[keep]


if __name__ == "__main__":
    import argparse
    import pandas as pd
    from pathlib import Path

    parser = argparse.ArgumentParser(description="Extract map lines from a binary image")
    parser.add_argument("image", help="Path to binary map image")
    parser.add_argument("--resolution", type=float, default=10.0, help="Pixels per meter")
    parser.add_argument("-o", "--output", help="Output CSV file (default: <image>_lines.csv)")
    args = parser.parse_args()

    # Load image as grayscale
    img = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {args.image}")

    # Convert to binary (assuming white = free, black = obstacle)
    binary_map = (img > 127).astype(np.uint8)

    lines = extract_map_lines(binary_map, args.resolution)

    # Save to CSV
    output_path = args.output or str(Path(args.image).with_suffix("")) + "_lines.csv"
    df = pd.DataFrame(lines, columns=["alpha", "distance"]).round(6)
    df.to_csv(output_path, index=False)

    print(f"\nExtracted {len(lines)} unique lines -> {output_path}")
    for i, (alpha, d) in enumerate(lines):
        orientation = "horizontal" if abs(alpha) < 0.1 or abs(alpha - np.pi) < 0.1 else "vertical"
        print(f"  Line {i + 1}: α={np.rad2deg(alpha):.1f}°, d={d:.2f}m ({orientation})")
