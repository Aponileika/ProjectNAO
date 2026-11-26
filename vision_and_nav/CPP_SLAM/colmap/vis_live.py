#!/usr/bin/env python3

from pathlib import Path
import argparse
import time

import numpy as np
import viser
import viser.transforms as vtf
from viser.extras.colmap import read_cameras_binary, read_images_binary

from vis_colmap import (
    initialize_view_from_first_camera,
    read_latest_snapshot,
    read_tracking_binary,
)


def make_segments(points):
    points = np.asarray(points, dtype=np.float32)
    if len(points) < 2:
        return None
    return np.stack((points[:-1], points[1:]), axis=1)


def make_tracking_colors(num_points: int):
    interpolation = np.linspace(
        0.0, 1.0, num_points, dtype=np.float32
    )
    start_color = np.array([0, 120, 255], dtype=np.float32)
    end_color = np.array([255, 80, 80], dtype=np.float32)
    point_colors = (
        (1.0 - interpolation[:, None]) * start_color[None, :]
        + interpolation[:, None] * end_color[None, :]
    ).astype(np.uint8)
    return np.stack((point_colors[:-1], point_colors[1:]), axis=1)


def read_keyframe_mode(root: Path, snapshot_id: int):
    path = root / "sparse" / "snapshots" / str(snapshot_id) / "keyframes_mode.txt"
    # Existing exports contain complete snapshots without a mode file.
    mode = path.read_text().strip() if path.is_file() else "replace"
    if mode not in ("append", "replace"):
        raise ValueError(f"unknown keyframe snapshot mode: {mode}")
    return mode


def append_tracking_frames(displayed, incoming, replace):
    if replace or displayed is None:
        return np.array(incoming, copy=True)
    # The producer's history may contain BA corrections. Accept only frames
    # committed since our previous poll; never rewrite displayed positions.
    if len(incoming) <= len(displayed):
        return displayed
    return np.concatenate((displayed, incoming[len(displayed):]), axis=0)


def live_snapshot_replaces_history(root: Path, previous_id, latest_id: int):
    if read_keyframe_mode(root, latest_id) == "replace":
        return True
    if previous_id is None:
        return False
    if latest_id < previous_id:
        return True
    # A missed map reset must still clear the old run. Inspect only its mode;
    # do not load or replay any of the missed keyframes.
    return any(read_keyframe_mode(root, snapshot_id) == "replace"
               for snapshot_id in range(previous_id + 1, latest_id))


def load_live_snapshot(root: Path, snapshot_id: int):
    snapshot_path = root / "sparse" / "snapshots" / str(snapshot_id)
    tracks = read_tracking_binary(snapshot_path / "tracking.bin")

    cameras_path = snapshot_path / "cameras.bin"
    images_path = snapshot_path / "images.bin"
    cameras = (
        read_cameras_binary(cameras_path)
        if cameras_path.is_file()
        else {}
    )
    images = (
        read_images_binary(images_path)
        if images_path.is_file()
        else {}
    )
    return tracks, cameras, images


def main(root_path: str):
    root = Path(root_path)
    print(f"root path = {root.resolve()}")
    print(
        f"live snapshot path = "
        f"{(root / 'sparse' / 'snapshots').resolve()}"
    )

    server = viser.ViserServer()
    gui_frustum_scale = server.gui.add_slider(
        "Frustum scale",
        min=0.01,
        max=5.0,
        step=0.01,
        initial_value=0.2,
    )
    gui_tracking_thickness = server.gui.add_slider(
        "Trajectory thickness",
        min=1.0,
        max=10.0,
        step=0.5,
        initial_value=3.0,
    )
    gui_show_frustums = server.gui.add_checkbox(
        "Show keyframe frustums",
        initial_value=True,
    )
    gui_show_tracking = server.gui.add_checkbox(
        "Show tracking trajectory",
        initial_value=True,
    )
    gui_show_ground_truth = server.gui.add_checkbox(
        "Show ground-truth trajectory",
        initial_value=True,
    )
    server.gui.add_markdown(
        "Live mode displays only keyframe frustums, tracking, and "
        "ground truth."
    )

    handles = {
        "frustums": {},
        "tracking": None,
        "ground_truth": None,
        "camera_initialized": False,
        "tracking_positions": None,
    }

    @gui_frustum_scale.on_update
    def _(_):
        for frustum in handles["frustums"].values():
            frustum.scale = gui_frustum_scale.value

    @gui_tracking_thickness.on_update
    def _(_):
        if handles["tracking"] is not None:
            handles["tracking"].line_width = gui_tracking_thickness.value
        if handles["ground_truth"] is not None:
            handles["ground_truth"].line_width = (
                gui_tracking_thickness.value
            )

    @gui_show_frustums.on_update
    def _(_):
        for frustum in handles["frustums"].values():
            frustum.visible = gui_show_frustums.value

    @gui_show_tracking.on_update
    def _(_):
        if handles["tracking"] is not None:
            handles["tracking"].visible = gui_show_tracking.value

    @gui_show_ground_truth.on_update
    def _(_):
        if handles["ground_truth"] is not None:
            handles["ground_truth"].visible = gui_show_ground_truth.value

    def update_ground_truth(ground_truth):
        if handles["ground_truth"] is not None:
            handles["ground_truth"].remove()
            handles["ground_truth"] = None

        segments = make_segments(ground_truth)
        if segments is None:
            return

        colors = np.empty(segments.shape, dtype=np.uint8)
        colors[:] = np.array([40, 230, 80], dtype=np.uint8)
        handles["ground_truth"] = server.scene.add_line_segments(
            "/ground_truth/trajectory",
            points=segments,
            colors=colors,
            line_width=gui_tracking_thickness.value,
            visible=gui_show_ground_truth.value,
        )

    def update_tracking(tracks, replace):
        handles["tracking_positions"] = append_tracking_frames(
            handles["tracking_positions"], tracks, replace
        )
        tracks = handles["tracking_positions"]
        if handles["tracking"] is not None:
            handles["tracking"].remove()
            handles["tracking"] = None

        segments = make_segments(tracks)
        if segments is None:
            return

        handles["tracking"] = server.scene.add_line_segments(
            "/tracking/trajectory",
            points=segments,
            colors=make_tracking_colors(len(tracks)),
            line_width=gui_tracking_thickness.value,
            visible=gui_show_tracking.value,
        )

    def update_frustums(cameras, images, replace):
        if replace:
            removed_image_ids = handles["frustums"].keys() - images.keys()
            for image_id in removed_image_ids:
                handles["frustums"].pop(image_id).remove()

        for image_id in sorted(images):
            image = images[image_id]
            camera = cameras.get(image.camera_id)
            if camera is None or camera.model != "PINHOLE":
                continue

            world_from_camera = (
                vtf.SE3.from_rotation_and_translation(
                    vtf.SO3(image.qvec), image.tvec
                ).inverse()
            )

            if image_id in handles["frustums"]:
                frustum = handles["frustums"][image_id]
                frustum.wxyz = world_from_camera.rotation().wxyz
                frustum.position = world_from_camera.translation()
                frustum.visible = gui_show_frustums.value
                continue

            _, fy, _, _ = camera.params[:4]
            handles["frustums"][image_id] = (
                server.scene.add_camera_frustum(
                    f"/keyframes/frustum_{image_id}",
                    fov=2.0 * np.arctan2(camera.height / 2.0, fy),
                    aspect=camera.width / camera.height,
                    scale=gui_frustum_scale.value,
                    image=None,
                    wxyz=world_from_camera.rotation().wxyz,
                    position=world_from_camera.translation(),
                    visible=gui_show_frustums.value,
                )
            )

        initialize_view_from_first_camera(
            server,
            images,
            handles,
        )

    last_snapshot_id = None
    ground_truth_mtime = None

    print("Waiting for live SLAM snapshots...")
    print("Open the viser URL shown above in your browser.")

    while True:
        ground_truth_path = root / "sparse" / "ground_truth.bin"
        current_ground_truth_mtime = (
            ground_truth_path.stat().st_mtime_ns
            if ground_truth_path.is_file()
            else None
        )
        if current_ground_truth_mtime != ground_truth_mtime:
            ground_truth = read_tracking_binary(ground_truth_path)
            update_ground_truth(ground_truth)
            ground_truth_mtime = current_ground_truth_mtime

        snapshot_id = read_latest_snapshot(root)
        if snapshot_id is not None and snapshot_id != last_snapshot_id:
            try:
                # Each normal snapshot contains only the just-processed KF.
                # Skipped KF snapshots are deliberately not replayed.
                tracks, cameras, images = load_live_snapshot(root, snapshot_id)
                replace = live_snapshot_replaces_history(root, last_snapshot_id, snapshot_id)
                update_frustums(cameras, images, replace=replace)
                update_tracking(tracks, replace=replace)
                last_snapshot_id = snapshot_id
                print(
                    f"[VISER] Live snapshot {snapshot_id}: "
                    f"tracking={len(tracks)}, keyframes={len(handles['frustums'])}"
                )
            except Exception as error:
                print(
                    f"[VISER] Failed to load live snapshot "
                    f"{snapshot_id}: {error}"
                )

        time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Minimal live PantoSLAM Viser viewer."
    )
    parser.add_argument("root", help="Path to the colmap output directory")
    arguments = parser.parse_args()
    main(arguments.root)
