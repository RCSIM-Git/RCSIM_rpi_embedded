import argparse
import json
import os
import statistics
from collections import Counter


def analyze_log(log_path):
    if not os.path.exists(log_path):
        print(f"Error: File {log_path} not found.")
        return

    print(f"Analyzing {log_path}...")

    entries = []
    with open(log_path, "r") as f:
        for line in f:
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue

    if not entries:
        print("No valid entries found.")
        return

    total_frames = len(entries)
    total_detections = 0
    class_counts = Counter()
    conf_scores = []

    # Heatmap (640x640 normalized to 64x64 grid)
    grid_size = 64
    heatmap = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

    for entry in entries:
        dets = entry.get("detections", [])
        total_detections += len(dets)

        for det in dets:
            cls_name = det.get("class_name", "unknown")
            conf = det.get("con", 0.0)
            class_counts[cls_name] += 1
            conf_scores.append(conf)

            # Heatmap
            bbox = det.get("bbox")  # [x1, y1, x2, y2] normalized
            if bbox:
                cx = (bbox[0] + bbox[2]) / 2.0
                cy = (bbox[1] + bbox[3]) / 2.0
                gx = int(cx * (grid_size - 1))
                gy = int(cy * (grid_size - 1))
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    heatmap[gy][gx] += 1

    # Stats
    avg_dets = total_detections / total_frames if total_frames > 0 else 0
    avg_conf = statistics.mean(conf_scores) if conf_scores else 0.0

    print("-" * 40)
    print(f"Total Frames Analyzed: {total_frames}")
    print(f"Total Detections: {total_detections}")
    print(f"Average Detections/Frame: {avg_dets:.2f}")
    print(f"Average Confidence: {avg_conf:.2f}")
    print("-" * 40)
    print("Class Distribution:")
    for cls, count in class_counts.most_common():
        print(f"  {cls}: {count} ({count/total_detections*100:.1f}%)")

    # Visualize Heatmap (ASCII)
    print("-" * 40)
    print("Detection Heatmap (ASCII 64x64 Aggregated):")
    # Downsample to 16x16 for print
    down_size = 16
    block_size = grid_size // down_size

    max_val = 0
    mini_map = [[0 for _ in range(down_size)] for _ in range(down_size)]

    for y in range(down_size):
        for x in range(down_size):
            sum_val = 0
            for by in range(block_size):
                for bx in range(block_size):
                    sum_val += heatmap[y * block_size + by][x * block_size + bx]
            mini_map[y][x] = sum_val
            max_val = max(max_val, sum_val)

    chars = " .:-=+*#%@"
    for y in range(down_size):
        line = ""
        for x in range(down_size):
            val = mini_map[y][x]
            if max_val > 0:
                idx = int((val / max_val) * (len(chars) - 1))
                line += chars[idx]
            else:
                line += chars[0]
        print(line)

    # Optional Matplotlib
    try:
        import matplotlib.pyplot as plt

        print("\nGenerating 'heatmap_detections.png'...")
        plt.figure(figsize=(10, 8))

        # Reconstruct coordinates for hexbin or scatter
        x_coords = []
        y_coords = []
        c_scores = []

        for entry in entries:
            for det in entry.get("detections", []):
                bbox = det.get("bbox")
                if bbox:
                    cx = (bbox[0] + bbox[2]) / 2.0
                    cy = (bbox[1] + bbox[3]) / 2.0
                    x_coords.append(cx)
                    y_coords.append(
                        cy
                    )  # y is typically 0 at top, plt is 0 at bottom usually
                    c_scores.append(det.get("con", 0.0))

        if x_coords:
            # Invert Y to match image coordinates (0 at top)
            y_coords = [1.0 - y for y in y_coords]

            plt.hexbin(x_coords, y_coords, gridsize=20, cmap="inferno", mincnt=1)
            plt.colorbar(label="Detection Count")
            plt.title("Detections Heatmap (Normalized Coords)")
            plt.xlim(0, 1)
            plt.ylim(0, 1)
            plt.savefig("heatmap_detections.png")
            print("Saved.")
        else:
            print("No detections to plot.")

    except ImportError:
        print("\nMatplotlib not found. Skipping plot generation.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze AI Detection Logs")
    parser.add_argument(
        "--log",
        type=str,
        default="debug_logs/detections.jsonl",
        help="Path to detections.jsonl",
    )
    args = parser.parse_args()

    analyze_log(args.log)
