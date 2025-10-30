import cv2
import numpy as np
from pathlib import Path

def visualize_rgb_depth_alignment(
    rgb_path: str,
    depth_path: str,
    save_dir: str = ".",
    alpha_rgb: float = 0.6,
    alpha_depth: float = 0.4
):
    """
    Visualize the alignment between an RGB image and its corresponding depth map.
    Generates two visualizations:
        1. Heatmap overlay (RGB + depth color map)
        2. Edge overlay (RGB edges vs. depth edges)

    Args:
        rgb_path (str): Path to the RGB image (e.g., "rgb_undist.png")
        depth_path (str): Path to the depth image (e.g., "depth_on_rgb_mm.png", uint16 in mm)
        save_dir (str): Output directory for visualization images
        alpha_rgb (float): Opacity of the RGB image in overlay (0–1)
        alpha_depth (float): Opacity of the depth heatmap in overlay (0–1)
    """

    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    # --- Load RGB and depth images ---
    rgb = cv2.imread(str(rgb_path))
    depth_mm = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)

    if rgb is None or depth_mm is None:
        raise ValueError("❌ Cannot read input images. Please check file paths.")

    # ==============================================================
    # [1] Heatmap overlay (Method 1)
    # ==============================================================
    # Normalize depth to 0–255 and convert to color heatmap
    depth_vis = cv2.normalize(depth_mm, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

    # Blend RGB and heatmap
    overlay_heatmap = cv2.addWeighted(rgb, alpha_rgb, depth_color, alpha_depth, 0)
    out_path1 = save_dir / "overlay_heatmap.png"
    cv2.imwrite(str(out_path1), overlay_heatmap)
    print(f"✅ Saved heatmap overlay: {out_path1}")

    # ==============================================================
    # [2] Edge overlay (Method 2)
    # ==============================================================
    # Compute RGB edges
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    edges_rgb = cv2.Canny(gray, 100, 200)
    edges_rgb_colored = cv2.cvtColor(edges_rgb, cv2.COLOR_GRAY2BGR)

    # Compute depth edges (scaled for visibility)
    depth_abs = cv2.convertScaleAbs(depth_mm, alpha=0.03)  # Adjust alpha if depth too bright/dark
    edges_depth = cv2.Canny(depth_abs, 50, 150)
    edges_depth_colored = cv2.cvtColor(edges_depth, cv2.COLOR_GRAY2BGR)
    edges_depth_colored[:, :, 1:] = 0  # Keep only red channel for depth edges

    # Combine RGB edges (white) and depth edges (red)
    overlay_edges = cv2.addWeighted(edges_rgb_colored, 1, edges_depth_colored, 1, 0)
    out_path2 = save_dir / "overlay_edges.png"
    cv2.imwrite(str(out_path2), overlay_edges)
    print(f"✅ Saved edge overlay: {out_path2}")

    return overlay_heatmap, overlay_edges


# ---------------- Example usage ----------------
if __name__ == "__main__":
    visualize_rgb_depth_alignment(
        rgb_path="rgb_undist.png",
        depth_path="depth_on_rgb_mm.png",
        save_dir="alignment_vis"
    )
