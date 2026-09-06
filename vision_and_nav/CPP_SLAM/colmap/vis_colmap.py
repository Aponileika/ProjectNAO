from pathlib import Path
import struct
import time

import imageio.v3 as iio
import numpy as np
import viser
import viser.transforms as vtf

from viser.extras.colmap import (
    read_cameras_binary,
    read_images_binary,
    read_points3d_binary,
)


def read_viser_image(path: Path):
    image = iio.imread(path)

    if image.ndim == 2:
        image = np.repeat(image[:, :, None], 3, axis=2)
    elif image.ndim == 3 and image.shape[2] == 1:
        image = np.repeat(image, 3, axis=2)

    if image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError(
            f"unsupported image shape {image.shape} for {path}"
        )

    return np.ascontiguousarray(image)


def read_tracking_binary(path: Path):
    if not path.exists():
        print(f"[VISER] tracking file does not exist: {path}")
        return np.empty((0, 3), dtype=np.float64)

    with path.open("rb") as f:
        num_points_data = f.read(8)

        if len(num_points_data) != 8:
            raise RuntimeError(
                "tracking.bin is too short"
            )

        num_points = struct.unpack("<Q", num_points_data)[0]

        print(
            f"[VISER] tracking.bin contains "
            f"{num_points} points"
        )

        point_data = f.read(num_points * 3 * 8)

        if len(point_data) != num_points * 3 * 8:
            raise RuntimeError(
                f"tracking.bin expected "
                f"{num_points * 3 * 8} bytes, "
                f"got {len(point_data)}"
            )

        if num_points == 0:
            return np.empty(
                (0, 3),
                dtype=np.float64,
            )

        points = np.frombuffer(
            point_data,
            dtype="<f8",
        ).reshape(num_points, 3).copy()

        print(
            f"[VISER] tracking first = {points[0]}, "
            f"last = {points[-1]}"
        )

        return points


def read_timestamp_binary(path: Path):
    if not path.exists():
        return np.empty((0,), dtype=np.float64)

    with path.open("rb") as f:
        count_data = f.read(8)
        if len(count_data) != 8:
            raise RuntimeError(f"{path} is too short")

        count = struct.unpack("<Q", count_data)[0]
        timestamp_data = f.read(count * 8)
        if len(timestamp_data) != count * 8:
            raise RuntimeError(
                f"{path} expected {count * 8} timestamp bytes, "
                f"got {len(timestamp_data)}"
            )

    return np.frombuffer(timestamp_data, dtype="<f8").copy()


def read_latest_snapshot(root: Path):
    latest_path = root / "sparse" / "latest.txt"

    if not latest_path.exists():
        return None

    try:
        return int(latest_path.read_text().strip())
    except (ValueError, OSError):
        return None


def load_snapshot(root: Path, snapshot_id: int):
    sparse_path = root / "sparse" / "snapshots" / str(snapshot_id)

    print("[VISER] Reading tracking.bin")
    tracks = read_tracking_binary(sparse_path / "tracking.bin")
    print(f"[VISER] tracking.bin OK: {len(tracks)}")

    cameras_path = sparse_path / "cameras.bin"

    if cameras_path.exists():
        print("[VISER] Reading cameras.bin")
        cameras = read_cameras_binary(cameras_path)
        print(f"[VISER] cameras.bin OK: {len(cameras)}")
    else:
        cameras = {}
        print("[VISER] cameras.bin omitted")

    images_path = sparse_path / "images.bin"

    if images_path.exists():
        print("[VISER] Reading images.bin")
        images = read_images_binary(images_path)
        print(f"[VISER] images.bin OK: {len(images)}")
    else:
        images = {}
        print("[VISER] images.bin omitted")

    points_path = sparse_path / "points3D.bin"

    if points_path.exists():
        print("[VISER] Reading points3D.bin")
        points3d = read_points3d_binary(points_path)
        print(f"[VISER] points3D.bin OK: {len(points3d)}")
    else:
        points3d = {}
        print("[VISER] points3D.bin omitted")

    ground_truth_path = root / "sparse" / "ground_truth.bin"

    if ground_truth_path.exists():
        print("[VISER] Reading ground-truth trajectory")
        ground_truth = read_tracking_binary(ground_truth_path)
        print(f"[VISER] Ground truth OK: {len(ground_truth)}")
    else:
        ground_truth = np.empty((0, 3), dtype=np.float64)

    ground_truth_timestamps = read_timestamp_binary(
        root / "sparse" / "ground_truth_timestamps.bin"
    )

    if len(ground_truth_timestamps) not in (0, len(ground_truth)):
        raise RuntimeError(
            "ground-truth position and timestamp counts differ: "
            f"{len(ground_truth)} != {len(ground_truth_timestamps)}"
        )

    return (
        tracks,
        ground_truth,
        ground_truth_timestamps,
        cameras,
        images,
        points3d,
    )


def initialize_view_from_first_camera(
    server,
    images,
    handles,
):
    if handles["camera_initialized"]:
        return

    if len(images) == 0:
        return

    first_img_id = min(images.keys())
    first_img = images[first_img_id]

    T_world_camera = vtf.SE3.from_rotation_and_translation(
        vtf.SO3(first_img.qvec),
        first_img.tvec,
    ).inverse()

    camera_position = np.array(
        T_world_camera.translation(),
        dtype=np.float64,
    )

    Rwc = T_world_camera.rotation().as_matrix()

    camera_forward = Rwc @ np.array(
        [0.0, 0.0, 1.0],
        dtype=np.float64,
    )

    camera_up = Rwc @ np.array(
        [0.0, -1.0, 0.0],
        dtype=np.float64,
    )

    viewer_position = (
        camera_position
        - 2.0 * camera_forward
        + 0.5 * camera_up
    )

    viewer_look_at = (
        camera_position
        + 2.0 * camera_forward
    )

    server.initial_camera.position = tuple(viewer_position)
    server.initial_camera.look_at = tuple(viewer_look_at)

    for client in server.get_clients().values():
        client.camera.position = viewer_position
        client.camera.look_at = viewer_look_at

    handles["camera_initialized"] = True


def get_track_follow_pose(
    tracks,
    images,
    distance: float,
    height: float,
    look_ahead: float,
):
    """Return a third-person viewer pose at the newest tracked position."""
    if len(tracks) == 0 or len(images) == 0:
        return None

    target_position = np.asarray(tracks[-1], dtype=np.float64)

    if not np.all(np.isfinite(target_position)):
        return None

    latest_img = images[max(images.keys())]
    T_world_camera = vtf.SE3.from_rotation_and_translation(
        vtf.SO3(latest_img.qvec),
        latest_img.tvec,
    ).inverse()
    Rwc = T_world_camera.rotation().as_matrix()

    # COLMAP/OpenCV cameras look along +Z with -Y as camera-up.
    forward = Rwc @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    up = Rwc @ np.array([0.0, -1.0, 0.0], dtype=np.float64)
    forward /= np.linalg.norm(forward)
    up /= np.linalg.norm(up)

    viewer_position = target_position - distance * forward + height * up
    viewer_look_at = target_position + look_ahead * forward

    return viewer_position, viewer_look_at, up


def update_track_follow_camera(
    server,
    handles,
    gui_follow_track,
    gui_follow_distance,
    gui_follow_height,
    gui_follow_look_ahead,
    client=None,
):
    if not gui_follow_track.value:
        return

    pose = get_track_follow_pose(
        handles["latest_tracks"],
        handles["latest_images"],
        gui_follow_distance.value,
        gui_follow_height.value,
        gui_follow_look_ahead.value,
    )

    if pose is None:
        return

    viewer_position, viewer_look_at, viewer_up = pose

    server.initial_camera.position = viewer_position
    server.initial_camera.look_at = viewer_look_at
    server.initial_camera.up = viewer_up

    clients = [client] if client is not None else server.get_clients().values()

    for current_client in clients:
        with current_client.atomic():
            current_client.camera.position = viewer_position
            current_client.camera.look_at = viewer_look_at
            current_client.camera.up_direction = viewer_up


def update_expected_ground_truth_marker(
    server,
    images,
    ground_truth,
    ground_truth_timestamps,
    image_id,
    visible,
    handles,
):
    marker = handles["expected_gt_marker"]
    label = handles["expected_gt_label"]

    if handles["expected_gt_error"] is not None:
        handles["expected_gt_error"].remove()
        handles["expected_gt_error"] = None

    if (
        not visible
        or image_id not in images
        or len(ground_truth) == 0
        or len(ground_truth_timestamps) != len(ground_truth)
    ):
        if marker is not None:
            marker.visible = False
        if label is not None:
            label.visible = False
        return

    try:
        image_timestamp = int(Path(images[image_id].name).stem) * 1e-9
    except ValueError:
        print(
            f"[VISER] Cannot parse timestamp from image "
            f"{images[image_id].name}"
        )
        return

    insertion_index = int(
        np.searchsorted(ground_truth_timestamps, image_timestamp)
    )
    candidate_indices = []
    if insertion_index < len(ground_truth_timestamps):
        candidate_indices.append(insertion_index)
    if insertion_index > 0:
        candidate_indices.append(insertion_index - 1)

    if not candidate_indices:
        return

    gt_index = min(
        candidate_indices,
        key=lambda index: abs(
            ground_truth_timestamps[index] - image_timestamp
        ),
    )
    gt_position = np.asarray(ground_truth[gt_index], dtype=np.float64)
    time_error = ground_truth_timestamps[gt_index] - image_timestamp

    if marker is None:
        marker = server.scene.add_icosphere(
            "/ground_truth/expected_position",
            radius=0.045,
            color=(255, 40, 220),
        )
        handles["expected_gt_marker"] = marker

    if label is None:
        label = server.scene.add_label(
            "/ground_truth/expected_position_label",
            "",
            anchor="bottom-center",
        )
        handles["expected_gt_label"] = label

    marker.position = gt_position
    marker.visible = True
    label.position = gt_position
    label.text = (
        f"Expected GT for KF {image_id - 1} "
        f"(dt={time_error * 1e3:+.3f} ms)"
    )
    label.visible = True

    image = images[image_id]
    estimated_pose = vtf.SE3.from_rotation_and_translation(
        vtf.SO3(image.qvec),
        image.tvec,
    ).inverse()
    estimated_position = np.asarray(
        estimated_pose.translation(),
        dtype=np.float32,
    )
    error_segment = np.stack(
        (estimated_position, gt_position.astype(np.float32)),
        axis=0,
    )[None, :, :]
    error_colors = np.array(
        [[[255, 210, 0], [255, 40, 220]]],
        dtype=np.uint8,
    )
    handles["expected_gt_error"] = server.scene.add_line_segments(
        "/ground_truth/expected_position_error",
        points=error_segment,
        colors=error_colors,
        line_width=3.0,
    )
    handles["selected_gt_image_id"] = image_id


def update_visualization(
    server,
    root: Path,
    tracks,
    ground_truth,
    ground_truth_timestamps,
    cameras,
    images,
    points3d,
    gui_point_size,
    gui_frustum_scale,
    gui_tracking_thickness,
    gui_show_keyframes,
    gui_show_expected_gt,
    gui_follow_track,
    gui_follow_distance,
    gui_follow_height,
    gui_follow_look_ahead,
    handles,
):
    images_path = root / "images"

    ground_truth_changed = not np.array_equal(
        handles["latest_ground_truth"],
        ground_truth,
    )

    handles["latest_tracks"] = tracks
    handles["latest_images"] = images
    handles["latest_ground_truth"] = ground_truth
    handles["latest_ground_truth_timestamps"] = ground_truth_timestamps
    handles["camera_image_preview"].visible = len(images) > 0

    if len(images) > 0:
        latest_img_id = max(images.keys())
        latest_image_file = images_path / images[latest_img_id].name

        if latest_image_file.is_file():
            try:
                handles["camera_image_preview"].image = read_viser_image(
                    latest_image_file
                )
            except Exception as error:
                print(
                    f"[VISER] Failed to read camera image "
                    f"{latest_img_id} from {latest_image_file}: {error}"
                )
        else:
            print(
                f"[VISER] Missing camera image {latest_img_id}: "
                f"{latest_image_file}"
            )

    if handles["point_cloud"] is not None:
        handles["point_cloud"].remove()
        handles["point_cloud"] = None

    if len(points3d) > 0:
        points = np.array(
            [p.xyz for p in points3d.values()],
            dtype=np.float32,
        )

        colors = np.array(
            [p.rgb for p in points3d.values()],
            dtype=np.uint8,
        )

        handles["point_cloud"] = server.scene.add_point_cloud(
            name="/colmap/points",
            points=points,
            colors=colors,
            point_size=gui_point_size.value,
        )

    if handles["tracking_trajectory"] is not None:
        handles["tracking_trajectory"].remove()
        handles["tracking_trajectory"] = None

    if len(tracks) >= 2:
        tracking_points = np.asarray(
            tracks,
            dtype=np.float32,
        )

        tracking_segments = np.stack(
            (
                tracking_points[:-1],
                tracking_points[1:],
            ),
            axis=1,
        )

        num_points = tracking_points.shape[0]

        t = np.linspace(0.0, 1.0, num_points, dtype=np.float32)

        start_color = np.array([0, 120, 255], dtype=np.float32)
        end_color = np.array([255, 80, 80], dtype=np.float32)

        point_colors = (
            (1.0 - t[:, None]) * start_color[None, :]
            + t[:, None] * end_color[None, :]
        ).astype(np.uint8)

        segment_colors = np.stack(
            (
                point_colors[:-1],
                point_colors[1:],
            ),
            axis=1,
        )

        print(
            f"[VISER] Adding tracking trajectory: "
            f"{tracking_segments.shape}, colors: {segment_colors.shape}"
        )

        handles["tracking_trajectory"] = server.scene.add_line_segments(
            name="/tracking/trajectory",
            points=tracking_segments,
            colors=segment_colors,
            line_width=gui_tracking_thickness.value,
        )
    else:
        handles["tracking_trajectory"] = None

    if (
        handles["ground_truth_size"] != len(ground_truth)
        or ground_truth_changed
    ):
        if handles["ground_truth_trajectory"] is not None:
            handles["ground_truth_trajectory"].remove()
            handles["ground_truth_trajectory"] = None

        if len(ground_truth) >= 2:
            ground_truth_points = np.asarray(
                ground_truth,
                dtype=np.float32,
            )
            ground_truth_segments = np.stack(
                (
                    ground_truth_points[:-1],
                    ground_truth_points[1:],
                ),
                axis=1,
            )
            ground_truth_colors = np.empty(
                ground_truth_segments.shape,
                dtype=np.uint8,
            )
            ground_truth_colors[:] = np.array(
                [40, 230, 80],
                dtype=np.uint8,
            )

            handles["ground_truth_trajectory"] = (
                server.scene.add_line_segments(
                    name="/ground_truth/trajectory",
                    points=ground_truth_segments,
                    colors=ground_truth_colors,
                    line_width=gui_tracking_thickness.value,
                )
            )

        handles["ground_truth_size"] = len(ground_truth)

    initialize_view_from_first_camera(
        server,
        images,
        handles,
    )

    # Camera images are by far the most expensive scene objects to decode,
    # encode, and send. Keep existing handles alive between snapshots and only
    # create frustums for newly seen keyframes. Iterating in descending ID order
    # makes the latest images appear first during initial scene loading.
    removed_img_ids = handles["camera_frames"].keys() - images.keys()

    for img_id in removed_img_ids:
        handles["camera_frames"].pop(img_id).remove()
        handles["camera_frustums"].pop(img_id, None)

    for img_id in sorted(images.keys(), reverse=True):
        img = images[img_id]
        cam = cameras[img.camera_id]

        T_world_camera = vtf.SE3.from_rotation_and_translation(
            vtf.SO3(img.qvec),
            img.tvec,
        ).inverse()

        if img_id in handles["camera_frames"]:
            frame = handles["camera_frames"][img_id]
            frame.wxyz = T_world_camera.rotation().wxyz
            frame.position = T_world_camera.translation()
            frame.visible = gui_show_keyframes.value
            if img_id in handles["camera_frustums"]:
                handles["camera_frustums"][img_id].visible = (
                    gui_show_keyframes.value
                )
            continue

        handles["camera_frames"][img_id] = server.scene.add_frame(
            f"/colmap/frame_{img_id}",
            wxyz=T_world_camera.rotation().wxyz,
            position=T_world_camera.translation(),
            axes_length=gui_frustum_scale.value * 0.5,
            axes_radius=gui_frustum_scale.value * 0.025,
            visible=gui_show_keyframes.value,
        )

        if cam.model == "PINHOLE":
            fx, fy, cx, cy = cam.params

            H = cam.height
            W = cam.width
            image_file = images_path / img.name

            if image_file.is_file():
                try:
                    image = read_viser_image(image_file)
                except Exception as error:
                    image = None
                    print(
                        f"[VISER] Failed to read camera image "
                        f"{img_id} from {image_file}: {error}"
                    )
            else:
                image = None
                print(
                    f"[VISER] Missing camera image {img_id}: "
                    f"{image_file}"
                )

            frustum = server.scene.add_camera_frustum(
                f"/colmap/frame_{img_id}/frustum",
                fov=2 * np.arctan2(H / 2.0, fy),
                aspect=W / H,
                scale=gui_frustum_scale.value,
                image=image,
                visible=gui_show_keyframes.value,
            )

            handles["camera_frustums"][img_id] = frustum

            @frustum.on_click
            def _(_, selected_image_file=image_file, selected_id=img_id):
                update_expected_ground_truth_marker(
                    server,
                    handles["latest_images"],
                    handles["latest_ground_truth"],
                    handles["latest_ground_truth_timestamps"],
                    selected_id,
                    gui_show_expected_gt.value,
                    handles,
                )

                if not selected_image_file.is_file():
                    print(
                        f"[VISER] Missing camera image {selected_id}: "
                        f"{selected_image_file}"
                    )
                    return

                try:
                    handles["camera_image_preview"].image = read_viser_image(
                        selected_image_file
                    )
                except Exception as error:
                    print(
                        f"[VISER] Failed to read camera image "
                        f"{selected_id} from {selected_image_file}: {error}"
                    )

        else:
            print(
                f"[VISER] Skipping frustum for image {img_id}: "
                f"camera model is {cam.model}"
            )

    if len(images) > 0:
        update_expected_ground_truth_marker(
            server,
            images,
            ground_truth,
            ground_truth_timestamps,
            max(images.keys()),
            gui_show_expected_gt.value,
            handles,
        )

    update_track_follow_camera(
        server,
        handles,
        gui_follow_track,
        gui_follow_distance,
        gui_follow_height,
        gui_follow_look_ahead,
    )


def main(root: str):
    root = Path(root)

    print(f"root path = {root.resolve()}")
    print(f"images path = {(root / 'images').resolve()}")
    print(
        f"snapshot path = "
        f"{(root / 'sparse' / 'snapshots').resolve()}"
    )

    server = viser.ViserServer()

    initial_point_size = 0.02
    initial_frustum_scale = initial_point_size * 10.0

    gui_point_size = server.gui.add_slider(
        "Point size",
        min=0.001,
        max=1.0,
        step=0.001,
        initial_value=initial_point_size,
    )

    gui_frustum_scale = server.gui.add_slider(
        "Frustum scale",
        min=0.01,
        max=5.0,
        step=0.01,
        initial_value=initial_frustum_scale,
    )

    gui_tracking_thickness = server.gui.add_slider(
        "Tracking thickness",
        min=1.0,
        max=10.0,
        step=0.5,
        initial_value=3.0,
    )

    gui_show_keyframes = server.gui.add_checkbox(
        "Show keyframes",
        initial_value=True,
        hint="Show or hide all keyframe axes and image frustums.",
    )

    gui_show_expected_gt = server.gui.add_checkbox(
        "Show expected GT position",
        initial_value=True,
        hint=(
            "Highlight the ground-truth position matching the latest or "
            "clicked keyframe timestamp."
        ),
    )

    camera_image_preview = server.gui.add_image(
        np.zeros((2, 2, 3), dtype=np.uint8),
        label="Camera image (latest; click a frustum to inspect)",
        format="jpeg",
        jpeg_quality=90,
        visible=False,
    )

    server.gui.add_markdown(
        "**Free camera:** W/A/S/D move, Q/E move down/up, "
        "arrow keys rotate, and the mouse orbits/pans/zooms."
    )

    gui_follow_track = server.gui.add_checkbox(
        "Follow latest track",
        initial_value=False,
        hint="Toggle a third-person view behind the newest tracked frame.",
    )

    gui_follow_distance = server.gui.add_slider(
        "Follow distance",
        min=0.05,
        max=20.0,
        step=0.05,
        initial_value=2.0,
    )

    gui_follow_height = server.gui.add_slider(
        "Follow height",
        min=0.0,
        max=10.0,
        step=0.05,
        initial_value=0.5,
    )

    gui_follow_look_ahead = server.gui.add_slider(
        "Follow look-ahead",
        min=0.05,
        max=20.0,
        step=0.05,
        initial_value=2.0,
    )

    handles = {
        "point_cloud": None,
        "tracking_trajectory": None,
        "ground_truth_trajectory": None,
        "ground_truth_size": 0,
        "expected_gt_marker": None,
        "expected_gt_label": None,
        "expected_gt_error": None,
        "selected_gt_image_id": None,
        "camera_frames": {},
        "camera_frustums": {},
        "camera_initialized": False,
        "latest_tracks": np.empty((0, 3), dtype=np.float64),
        "latest_images": {},
        "latest_ground_truth": np.empty((0, 3), dtype=np.float64),
        "latest_ground_truth_timestamps": np.empty(
            (0,), dtype=np.float64
        ),
        "camera_image_preview": camera_image_preview,
    }

    @gui_point_size.on_update
    def _(_):
        if handles["point_cloud"] is not None:
            handles["point_cloud"].point_size = gui_point_size.value

    @gui_frustum_scale.on_update
    def _(_):
        for frustum in handles["camera_frustums"].values():
            frustum.scale = gui_frustum_scale.value

    @gui_show_keyframes.on_update
    def _(_):
        for frame in handles["camera_frames"].values():
            frame.visible = gui_show_keyframes.value
        for frustum in handles["camera_frustums"].values():
            frustum.visible = gui_show_keyframes.value

    @gui_show_expected_gt.on_update
    def _(_):
        image_id = handles["selected_gt_image_id"]
        if image_id is None and handles["latest_images"]:
            image_id = max(handles["latest_images"].keys())

        update_expected_ground_truth_marker(
            server,
            handles["latest_images"],
            handles["latest_ground_truth"],
            handles["latest_ground_truth_timestamps"],
            image_id,
            gui_show_expected_gt.value,
            handles,
        )

    @gui_tracking_thickness.on_update
    def _(_):
        if handles["tracking_trajectory"] is not None:
            handles["tracking_trajectory"].line_width = (
                gui_tracking_thickness.value
            )
        if handles["ground_truth_trajectory"] is not None:
            handles["ground_truth_trajectory"].line_width = (
                gui_tracking_thickness.value
            )

    def refresh_track_follow():
        update_track_follow_camera(
            server,
            handles,
            gui_follow_track,
            gui_follow_distance,
            gui_follow_height,
            gui_follow_look_ahead,
        )

    @gui_follow_track.on_update
    def _(_):
        refresh_track_follow()

    @gui_follow_distance.on_update
    def _(_):
        refresh_track_follow()

    @gui_follow_height.on_update
    def _(_):
        refresh_track_follow()

    @gui_follow_look_ahead.on_update
    def _(_):
        refresh_track_follow()

    @server.on_client_connect
    def _(client):
        update_track_follow_camera(
            server,
            handles,
            gui_follow_track,
            gui_follow_distance,
            gui_follow_height,
            gui_follow_look_ahead,
            client=client,
        )

    last_snapshot_id = None

    print("Waiting for SLAM snapshots...")
    print("Open the viser URL shown above in your browser.")

    while True:
        snapshot_id = read_latest_snapshot(root)

        if snapshot_id is not None and snapshot_id != last_snapshot_id:
            try:
                (
                    tracks,
                    ground_truth,
                    ground_truth_timestamps,
                    cameras,
                    images,
                    points3d,
                ) = load_snapshot(root, snapshot_id)

                update_visualization(
                    server,
                    root,
                    tracks,
                    ground_truth,
                    ground_truth_timestamps,
                    cameras,
                    images,
                    points3d,
                    gui_point_size,
                    gui_frustum_scale,
                    gui_tracking_thickness,
                    gui_show_keyframes,
                    gui_show_expected_gt,
                    gui_follow_track,
                    gui_follow_distance,
                    gui_follow_height,
                    gui_follow_look_ahead,
                    handles,
                )

                last_snapshot_id = snapshot_id

            except Exception as Error:
                print(
                    f"[VISER] Failed to load snapshot "
                    f"{snapshot_id}: {Error}"
                )

        time.sleep(0.1)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "root",
        help="Path to COLMAP root folder",
    )

    args = parser.parse_args()

    main(args.root)
