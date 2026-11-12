#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    default_ply =  "dense_map.ply"

    parser = argparse.ArgumentParser(
        description="Visualize a PLY point cloud with Open3D."
    )
    parser.add_argument(
        "ply",
        nargs="?",
        type=Path,
        default=default_ply,
        help=f"PLY file to open (default: {default_ply})",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Rendered point size (default: 2.0)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ply_path = args.ply.expanduser().resolve()

    if not ply_path.is_file():
        print(f"PLY file not found: {ply_path}")
        return 1

    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    if point_cloud.is_empty():
        print(f"PLY contains no readable points: {ply_path}")
        return 1

    if not point_cloud.has_colors():
        point_cloud.paint_uniform_color([0.85, 0.85, 0.85])

    print(f"Loaded {len(point_cloud.points):,} points from {ply_path}")

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=f"Dense map - {ply_path.name}")
    visualizer.add_geometry(point_cloud)

    render_options = visualizer.get_render_option()
    render_options.background_color = np.asarray([0.02, 0.02, 0.02])
    render_options.point_size = args.point_size

    try:
        visualizer.run()
    finally:
        visualizer.destroy_window()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
