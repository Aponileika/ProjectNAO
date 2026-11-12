#!/usr/bin/env python3

from pathlib import Path
import os
import struct
import sys
import time

import numpy as np
import viser


POINT_CLOUD_MAGIC = 0x50414E544F50434C
HEADER = struct.Struct("<QQQ")


def parent_is_alive(parent_pid: int) -> bool:
    try:
        os.kill(parent_pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def read_point_cloud(path: Path):
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise RuntimeError("point-cloud file is shorter than its header")

    magic, sequence, count = HEADER.unpack_from(data)
    if magic != POINT_CLOUD_MAGIC:
        raise RuntimeError("point-cloud file has the wrong magic value")

    positions_size = count * 3 * 4
    colors_size = count * 3
    expected_size = HEADER.size + positions_size + colors_size
    if len(data) != expected_size:
        raise RuntimeError(
            f"point-cloud frame expected {expected_size} bytes, got {len(data)}"
        )

    positions_offset = HEADER.size
    colors_offset = positions_offset + positions_size
    positions = np.frombuffer(
        data, dtype="<f4", count=count * 3, offset=positions_offset
    ).reshape((-1, 3)).copy()
    colors = np.frombuffer(
        data, dtype=np.uint8, count=count * 3, offset=colors_offset
    ).reshape((-1, 3)).copy()
    return sequence, positions, colors


def main(output_directory: str, parent_pid_text: str):
    output_path = Path(output_directory) / "points.bin"
    parent_pid = int(parent_pid_text)
    server = viser.ViserServer()
    point_size = server.gui.add_slider(
        "Point size", min=0.001, max=0.1, step=0.001, initial_value=0.008
    )
    point_count = server.gui.add_markdown("**Points:** waiting for first frame")
    server.gui.add_markdown(
        "**Metric scale:** axis length = 1 m; grid cells = 0.5 m; "
        "camera forward = −Z"
    )
    server.scene.add_grid(
        "/reference_grid",
        width=10.0,
        height=10.0,
        cell_size=0.5,
        cell_thickness=1.0,
        section_size=2.0,
        section_thickness=2.0,
        plane="xz",
    )
    axis_points = np.asarray(
        [
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
        ],
        dtype=np.float32,
    )
    axis_colors = np.asarray(
        [
            [[255, 60, 60], [255, 60, 60]],
            [[60, 255, 60], [60, 255, 60]],
            [[60, 120, 255], [60, 120, 255]],
        ],
        dtype=np.uint8,
    )
    server.scene.add_line_segments(
        "/metric_axes", axis_points, axis_colors, line_width=5.0
    )
    server.scene.add_label(
        "/metric_axes/x_label", "X  1 m", position=(1.0, 0.0, 0.0)
    )
    server.scene.add_label(
        "/metric_axes/y_label", "Y  1 m", position=(0.0, 1.0, 0.0)
    )
    server.scene.add_label(
        "/metric_axes/z_label", "Forward  1 m", position=(0.0, 0.0, -1.0)
    )
    for metres in range(1, 6):
        server.scene.add_label(
            f"/metric_scale/{metres}m",
            f"{metres} m",
            position=(0.0, 0.0, -float(metres)),
            font_screen_scale=0.8,
        )

    cloud_handle = None
    last_sequence = None

    @server.on_client_connect
    def _(client):
        client.camera.position = np.array([0.0, 0.0, 0.0])
        client.camera.look_at = np.array([0.0, 0.0, -3.0])
        client.camera.up_direction = np.array([0.0, 1.0, 0.0])

    print("Waiting for demo point-cloud frames...", flush=True)
    print("Open the Viser URL shown above in your browser.", flush=True)

    while parent_is_alive(parent_pid):
        if not output_path.exists():
            time.sleep(0.02)
            continue

        try:
            sequence, positions, colors = read_point_cloud(output_path)
        except (OSError, RuntimeError) as error:
            print(f"[DEMO VISER] Could not read frame: {error}", flush=True)
            time.sleep(0.02)
            continue

        if sequence == last_sequence:
            time.sleep(0.01)
            continue

        if cloud_handle is not None:
            cloud_handle.remove()
            cloud_handle = None

        if len(positions) > 0:
            cloud_handle = server.scene.add_point_cloud(
                "/demo/current_points",
                points=positions,
                colors=colors,
                point_size=point_size.value,
            )

        point_count.content = f"**Points:** {len(positions):,}"
        last_sequence = sequence

    print("Receiver exited; stopping demo viewer.", flush=True)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: demo_pointcloud_viewer.py OUTPUT_DIRECTORY PARENT_PID"
        )
    main(sys.argv[1], sys.argv[2])
