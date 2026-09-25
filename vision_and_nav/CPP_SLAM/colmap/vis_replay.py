#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import argparse
import struct
import threading
import time

import numpy as np
import viser

from vis_colmap import (
    read_timestamp_binary,
    read_tracking_binary,
    update_expected_ground_truth_marker,
    update_track_follow_camera,
    update_visualization,
)


FILE_MAGIC = b"PANTOVIZ"
BLOCK_MAGIC = b"PVBLOCK1"
INDEX_MAGIC = b"PVINDEX1"
FOOTER_MAGIC = b"PVEND001"
FORMAT_VERSION = 1

FILE_HEADER = struct.Struct("<8sIIQ")
BLOCK_PREFIX = struct.Struct("<8sQ")
BLOCK_DATA_HEADER = struct.Struct("<QdQQiiifQQQ")
CAMERA_HEADER = struct.Struct("<iQQ")
INDEX_HEADER = struct.Struct("<8sQ")
INDEX_ENTRY = struct.Struct("<QdQQ")
FOOTER = struct.Struct("<8sQ")

CAMERA_MODEL_NAMES = {
    0: "SIMPLE_PINHOLE",
    1: "PINHOLE",
    2: "SIMPLE_RADIAL",
    3: "RADIAL",
    4: "OPENCV",
    5: "OPENCV_FISHEYE",
    6: "FULL_OPENCV",
    7: "FOV",
    8: "SIMPLE_RADIAL_FISHEYE",
    9: "RADIAL_FISHEYE",
    10: "THIN_PRISM_FISHEYE",
}


def read_exact(file, size: int, description: str) -> bytes:
    data = file.read(size)
    if len(data) != size:
        raise RuntimeError(
            f"replay ended while reading {description}: "
            f"expected {size} bytes, got {len(data)}"
        )
    return data


@dataclass(frozen=True)
class ReplayIndexEntry:
    keyframe_id: int
    timestamp: float
    offset: int
    size: int


@dataclass
class ReplayBlockMetadata:
    keyframe_id: int
    timestamp: float
    num_dense_points: int
    origin_key: np.ndarray
    voxel_size: float
    voxels_per_side: int
    occupied_observation_threshold: int
    num_nonzero_occupancy: int
    camera: object
    image: object
    distortion: np.ndarray
    tracking_timestamps: np.ndarray
    tracking_points: np.ndarray
    dense_offset: int
    occupancy_offset: int


class ReplayFile:
    def __init__(self, path: Path):
        self.path = path
        self.index = self._read_index()
        self.blocks = [
            self._read_block_metadata(entry) for entry in self.index
        ]

        if self.blocks:
            tracking_parts = [
                block.tracking_points
                for block in self.blocks
                if len(block.tracking_points) > 0
            ]
            self.all_tracking_points = (
                np.concatenate(tracking_parts, axis=0)
                if tracking_parts
                else np.empty((0, 3), dtype=np.float64)
            )
        else:
            self.all_tracking_points = np.empty((0, 3), dtype=np.float64)

        cumulative_count = 0
        self.tracking_end_indices = []
        for block in self.blocks:
            cumulative_count += len(block.tracking_points)
            self.tracking_end_indices.append(cumulative_count)

    def _read_index(self) -> list[ReplayIndexEntry]:
        file_size = self.path.stat().st_size
        if file_size < FILE_HEADER.size + FOOTER.size:
            raise RuntimeError(f"{self.path} is too short")

        with self.path.open("rb") as file:
            magic, version, flags, num_blocks = FILE_HEADER.unpack(
                read_exact(file, FILE_HEADER.size, "file header")
            )
            if magic != FILE_MAGIC:
                raise RuntimeError(f"invalid replay magic {magic!r}")
            if version != FORMAT_VERSION:
                raise RuntimeError(
                    f"unsupported replay version {version}; "
                    f"expected {FORMAT_VERSION}"
                )
            if not flags & 0x1:
                raise RuntimeError("replay is not marked little-endian")
            if not flags & 0x2:
                raise RuntimeError("replay does not use sparse occupancy")

            file.seek(-FOOTER.size, 2)
            footer_magic, index_offset = FOOTER.unpack(
                read_exact(file, FOOTER.size, "footer")
            )
            if footer_magic != FOOTER_MAGIC:
                raise RuntimeError("replay footer is missing or incomplete")
            if not FILE_HEADER.size <= index_offset < file_size - FOOTER.size:
                raise RuntimeError(f"invalid replay index offset {index_offset}")

            file.seek(index_offset)
            index_magic, index_count = INDEX_HEADER.unpack(
                read_exact(file, INDEX_HEADER.size, "index header")
            )
            if index_magic != INDEX_MAGIC:
                raise RuntimeError("invalid replay index magic")
            if index_count != num_blocks:
                raise RuntimeError(
                    f"header/index block count differs: "
                    f"{num_blocks} != {index_count}"
                )

            entries = []
            for _ in range(index_count):
                values = INDEX_ENTRY.unpack(
                    read_exact(file, INDEX_ENTRY.size, "index entry")
                )
                entry = ReplayIndexEntry(*values)
                if (
                    entry.offset < FILE_HEADER.size
                    or entry.size < BLOCK_PREFIX.size
                    or entry.offset + entry.size > index_offset
                ):
                    raise RuntimeError(
                        f"invalid block range for keyframe "
                        f"{entry.keyframe_id}"
                    )
                entries.append(entry)

            expected_footer_offset = file_size - FOOTER.size
            if file.tell() != expected_footer_offset:
                raise RuntimeError("replay index size does not match footer")

        return entries

    def _read_block_metadata(
        self, entry: ReplayIndexEntry
    ) -> ReplayBlockMetadata:
        with self.path.open("rb") as file:
            file.seek(entry.offset)
            block_magic, block_size = BLOCK_PREFIX.unpack(
                read_exact(file, BLOCK_PREFIX.size, "block prefix")
            )
            if block_magic != BLOCK_MAGIC or block_size != entry.size:
                raise RuntimeError(
                    f"invalid block prefix for keyframe {entry.keyframe_id}"
                )

            (
                keyframe_id,
                timestamp,
                num_tracking_samples,
                num_dense_points,
                origin_x,
                origin_y,
                origin_z,
                voxel_size,
                voxels_per_side,
                occupied_threshold,
                num_nonzero_occupancy,
            ) = BLOCK_DATA_HEADER.unpack(
                read_exact(file, BLOCK_DATA_HEADER.size, "block header")
            )

            if keyframe_id != entry.keyframe_id or timestamp != entry.timestamp:
                raise RuntimeError("block contents do not match replay index")
            if voxel_size <= 0.0 or voxels_per_side == 0:
                raise RuntimeError("invalid occupancy dimensions")

            camera_model_id, width, height = CAMERA_HEADER.unpack(
                read_exact(file, CAMERA_HEADER.size, "camera header")
            )
            camera_parameters = np.frombuffer(
                read_exact(file, 4 * 8, "camera parameters"),
                dtype="<f8",
            ).copy()
            distortion = np.frombuffer(
                read_exact(file, 6 * 8, "distortion parameters"),
                dtype="<f8",
            ).copy()
            quaternion = np.frombuffer(
                read_exact(file, 4 * 8, "camera quaternion"),
                dtype="<f8",
            ).copy()
            translation = np.frombuffer(
                read_exact(file, 3 * 8, "camera translation"),
                dtype="<f8",
            ).copy()

            image_name_size = struct.unpack(
                "<Q", read_exact(file, 8, "image-name size")
            )[0]
            image_name = read_exact(
                file, image_name_size, "image name"
            ).decode("utf-8")

            tracking_data = read_exact(
                file,
                num_tracking_samples * 4 * 8,
                "tracking samples",
            )
            if num_tracking_samples:
                tracking_records = np.frombuffer(
                    tracking_data, dtype="<f8"
                ).reshape(num_tracking_samples, 4).copy()
                tracking_timestamps = tracking_records[:, 0]
                tracking_points = tracking_records[:, 1:]
            else:
                tracking_timestamps = np.empty((0,), dtype=np.float64)
                tracking_points = np.empty((0, 3), dtype=np.float64)

            dense_offset = file.tell()
            occupancy_offset = dense_offset + num_dense_points * 3 * 4
            expected_end = (
                occupancy_offset + num_nonzero_occupancy * 2 * 8
            )
            if expected_end != entry.offset + entry.size:
                raise RuntimeError(
                    f"block {keyframe_id} size mismatch: "
                    f"expected end {expected_end}, "
                    f"indexed end {entry.offset + entry.size}"
                )

        image_id = keyframe_id + 1
        model_name = CAMERA_MODEL_NAMES.get(
            camera_model_id, f"MODEL_{camera_model_id}"
        )
        camera = SimpleNamespace(
            id=image_id,
            model=model_name,
            width=width,
            height=height,
            params=camera_parameters,
        )
        image = SimpleNamespace(
            id=image_id,
            qvec=quaternion,
            tvec=translation,
            camera_id=image_id,
            name=image_name,
            xys=np.empty((0, 2), dtype=np.float64),
            point3D_ids=np.empty((0,), dtype=np.int64),
        )

        return ReplayBlockMetadata(
            keyframe_id=keyframe_id,
            timestamp=timestamp,
            num_dense_points=num_dense_points,
            origin_key=np.array(
                [origin_x, origin_y, origin_z], dtype=np.int32
            ),
            voxel_size=voxel_size,
            voxels_per_side=voxels_per_side,
            occupied_observation_threshold=occupied_threshold,
            num_nonzero_occupancy=num_nonzero_occupancy,
            camera=camera,
            image=image,
            distortion=distortion,
            tracking_timestamps=tracking_timestamps,
            tracking_points=tracking_points,
            dense_offset=dense_offset,
            occupancy_offset=occupancy_offset,
        )

    def read_dynamic_data(self, step: int):
        block = self.blocks[step]
        with self.path.open("rb") as file:
            file.seek(block.dense_offset)
            dense_data = read_exact(
                file,
                block.num_dense_points * 3 * 4,
                "dense points",
            )
            if block.num_dense_points:
                dense_points = np.frombuffer(
                    dense_data, dtype="<f4"
                ).reshape(block.num_dense_points, 3).copy()
            else:
                dense_points = np.empty((0, 3), dtype=np.float32)

            file.seek(block.occupancy_offset)
            occupancy_data = read_exact(
                file,
                block.num_nonzero_occupancy * 2 * 8,
                "sparse occupancy",
            )

        if block.num_nonzero_occupancy:
            entries = np.frombuffer(
                occupancy_data, dtype="<u8"
            ).reshape(block.num_nonzero_occupancy, 2).copy()
            indices = entries[:, 0]
            counts = entries[:, 1]
            voxel_count = block.voxels_per_side ** 3
            if np.any(indices >= voxel_count):
                raise RuntimeError(
                    f"block {block.keyframe_id} contains an invalid voxel index"
                )
        else:
            indices = np.empty((0,), dtype=np.uint64)
            counts = np.empty((0,), dtype=np.uint64)

        occupancy = {
            "origin_key": block.origin_key,
            "voxel_size": block.voxel_size,
            "voxels_per_side": block.voxels_per_side,
            "occupied_observation_threshold": (
                block.occupied_observation_threshold
            ),
            "indices": indices,
            "counts": counts,
        }
        return dense_points, occupancy

    def state_for_step(self, step: int):
        if not 0 <= step < len(self.blocks):
            raise IndexError(step)

        visible_blocks = self.blocks[: step + 1]
        cameras = {
            block.keyframe_id + 1: block.camera
            for block in visible_blocks
        }
        images = {
            block.keyframe_id + 1: block.image
            for block in visible_blocks
        }
        distortions = {
            block.keyframe_id + 1: block.distortion
            for block in visible_blocks
        }
        track_end = self.tracking_end_indices[step]
        tracks = self.all_tracking_points[:track_end]
        dense_points, occupancy = self.read_dynamic_data(step)

        black = np.zeros(3, dtype=np.uint8)
        points3d = {
            point_id: SimpleNamespace(xyz=point, rgb=black)
            for point_id, point in enumerate(dense_points)
        }

        return tracks, cameras, images, points3d, distortions, occupancy


def find_replay(root: Path, requested_path: str | None) -> Path:
    if requested_path is not None:
        path = Path(requested_path)
        if not path.is_absolute():
            path = root / path
        if not path.is_file():
            raise FileNotFoundError(path)
        return path

    replay_directory = root / "full_binaries"
    candidates = list(replay_directory.glob("*.bin"))
    if not candidates:
        raise FileNotFoundError(
            f"no replay binaries found in {replay_directory}"
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def main(root_path: str, replay_path: str | None = None):
    root = Path(root_path)
    replay_file_path = find_replay(root, replay_path)
    replay = ReplayFile(replay_file_path)
    if not replay.blocks:
        raise RuntimeError(f"{replay_file_path} contains no keyframe blocks")

    print(f"root path = {root.resolve()}")
    print(f"images path = {(root / 'images').resolve()}")
    print(f"replay path = {replay_file_path.resolve()}")
    print(f"keyframe blocks = {len(replay.blocks)}")

    ground_truth = read_tracking_binary(root / "sparse" / "ground_truth.bin")
    ground_truth_timestamps = read_timestamp_binary(
        root / "sparse" / "ground_truth_timestamps.bin"
    )
    if len(ground_truth_timestamps) not in (0, len(ground_truth)):
        raise RuntimeError(
            "ground-truth position and timestamp counts differ: "
            f"{len(ground_truth)} != {len(ground_truth_timestamps)}"
        )

    server = viser.ViserServer()
    initial_point_size = 0.02
    initial_frustum_scale = initial_point_size * 10.0

    gui_play = server.gui.add_button("Play", color="green")
    gui_pause = server.gui.add_button("Pause", color="red")
    gui_previous = server.gui.add_button("Skip back one keyframe")
    gui_next = server.gui.add_button("Skip forward one keyframe")
    gui_playback_speed = server.gui.add_slider(
        "Playback speed",
        min=0.1,
        max=10.0,
        step=0.1,
        initial_value=1.0,
    )
    gui_replay_status = server.gui.add_text(
        "Replay position",
        "Not loaded",
        disabled=True,
    )

    gui_point_size = server.gui.add_slider(
        "Point size", min=0.001, max=1.0, step=0.001,
        initial_value=initial_point_size,
    )
    gui_frustum_scale = server.gui.add_slider(
        "Frustum scale", min=0.01, max=5.0, step=0.01,
        initial_value=initial_frustum_scale,
    )
    gui_tracking_thickness = server.gui.add_slider(
        "Tracking thickness", min=1.0, max=10.0, step=0.5,
        initial_value=3.0,
    )
    gui_show_keyframes = server.gui.add_checkbox(
        "Show keyframes", initial_value=True,
        hint="Show or hide all keyframe axes and image frustums.",
    )
    gui_show_expected_gt = server.gui.add_checkbox(
        "Show expected GT position", initial_value=True,
        hint=(
            "Highlight the ground-truth position matching the latest or "
            "clicked keyframe timestamp."
        ),
    )
    gui_show_occupancy = server.gui.add_checkbox(
        "Show occupancy map", initial_value=True,
        hint=(
            "Show the green rolling-window boundary and red voxels with "
            "three or more observations."
        ),
    )
    gui_free_voxel_alpha = server.gui.add_slider(
        "Free-space wireframe alpha", min=0.0, max=1.0, step=0.001,
        initial_value=0.15,
    )
    gui_occupied_voxel_alpha = server.gui.add_slider(
        "Occupied voxel alpha", min=0.0, max=1.0, step=0.001,
        initial_value=0.55,
    )
    camera_image_preview = server.gui.add_image(
        np.zeros((2, 2, 3), dtype=np.uint8),
        label="Camera image (latest; click a frustum to inspect)",
        format="jpeg",
        jpeg_quality=90,
        visible=False,
    )
    server.gui.add_markdown(
        "**Image points:** replay keyframes do not contain 2D observations."
    )
    server.gui.add_markdown(
        "**Occupancy:** the <span style='color:#28d250'>green</span> "
        "wireframe is the rolling map boundary; "
        "<span style='color:#eb3232'>red</span> voxels have three or more "
        "observations."
    )
    server.gui.add_markdown(
        "**Free camera:** W/A/S/D move, Q/E move down/up, "
        "arrow keys rotate, and the mouse orbits/pans/zooms."
    )
    gui_follow_track = server.gui.add_checkbox(
        "Follow latest track", initial_value=False,
        hint="Toggle a third-person view behind the newest tracked frame.",
    )
    gui_follow_distance = server.gui.add_slider(
        "Follow distance", min=0.05, max=20.0, step=0.05,
        initial_value=2.0,
    )
    gui_follow_height = server.gui.add_slider(
        "Follow height", min=0.0, max=10.0, step=0.05,
        initial_value=0.5,
    )
    gui_follow_look_ahead = server.gui.add_slider(
        "Follow look-ahead", min=0.05, max=20.0, step=0.05,
        initial_value=2.0,
    )

    handles = {
        "point_cloud": None,
        "occupancy": None,
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
        "latest_cameras": {},
        "latest_distortions": {},
        "latest_ground_truth": np.empty((0, 3), dtype=np.float64),
        "latest_ground_truth_timestamps": np.empty((0,), dtype=np.float64),
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

    @gui_show_occupancy.on_update
    def _(_):
        if handles["occupancy"] is not None:
            handles["occupancy"].visible = gui_show_occupancy.value

    def refresh_occupancy_opacities():
        if handles["occupancy"] is None:
            return
        handles["occupancy"].set_opacities(
            gui_free_voxel_alpha.value,
            gui_occupied_voxel_alpha.value,
        )

    @gui_free_voxel_alpha.on_update
    def _(_):
        refresh_occupancy_opacities()

    @gui_occupied_voxel_alpha.on_update
    def _(_):
        refresh_occupancy_opacities()

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

    playback_lock = threading.Lock()
    playback = {
        "current_step": -1,
        "requested_step": 0,
        "playing": False,
        "deadline": None,
    }

    @gui_play.on_click
    def _(_):
        with playback_lock:
            if playback["current_step"] >= len(replay.blocks) - 1:
                playback["requested_step"] = 0
            playback["playing"] = True
            playback["deadline"] = None

    @gui_pause.on_click
    def _(_):
        with playback_lock:
            playback["playing"] = False
            playback["deadline"] = None

    @gui_previous.on_click
    def _(_):
        with playback_lock:
            playback["playing"] = False
            playback["requested_step"] = max(
                0, playback["current_step"] - 1
            )
            playback["deadline"] = None

    @gui_next.on_click
    def _(_):
        with playback_lock:
            playback["playing"] = False
            playback["requested_step"] = min(
                len(replay.blocks) - 1,
                playback["current_step"] + 1,
            )
            playback["deadline"] = None

    @gui_playback_speed.on_update
    def _(_):
        with playback_lock:
            playback["deadline"] = None

    def render_step(step: int):
        (
            tracks,
            cameras,
            images,
            points3d,
            distortions,
            occupancy,
        ) = replay.state_for_step(step)

        update_visualization(
            server,
            root,
            tracks,
            ground_truth,
            ground_truth_timestamps,
            cameras,
            images,
            points3d,
            distortions,
            occupancy,
            gui_point_size,
            gui_frustum_scale,
            gui_tracking_thickness,
            gui_show_keyframes,
            gui_show_expected_gt,
            gui_show_occupancy,
            gui_free_voxel_alpha,
            gui_occupied_voxel_alpha,
            gui_follow_track,
            gui_follow_distance,
            gui_follow_height,
            gui_follow_look_ahead,
            handles,
        )

        block = replay.blocks[step]
        gui_replay_status.value = (
            f"{step + 1}/{len(replay.blocks)} | "
            f"KF {block.keyframe_id} | t={block.timestamp:.6f}"
        )
        print(
            f"[VISER] Replay step {step + 1}/{len(replay.blocks)}: "
            f"KF {block.keyframe_id}, t={block.timestamp:.6f}, "
            f"dense={block.num_dense_points}, "
            f"occupied entries={block.num_nonzero_occupancy}"
        )

    print("Open the viser URL shown above in your browser.")
    while True:
        with playback_lock:
            requested_step = playback["requested_step"]
            current_step = playback["current_step"]

        if requested_step != current_step:
            try:
                render_step(requested_step)
            except Exception as error:
                print(
                    f"[VISER] Failed to render replay step "
                    f"{requested_step}: {error}"
                )
                with playback_lock:
                    playback["playing"] = False
                    playback["requested_step"] = current_step
                    playback["deadline"] = None
            else:
                with playback_lock:
                    playback["current_step"] = requested_step
                    playback["deadline"] = None
            continue

        with playback_lock:
            if not playback["playing"]:
                should_advance = False
            elif current_step >= len(replay.blocks) - 1:
                playback["playing"] = False
                playback["deadline"] = None
                should_advance = False
            else:
                if playback["deadline"] is None:
                    timestamp_delta = max(
                        0.0,
                        replay.blocks[current_step + 1].timestamp
                        - replay.blocks[current_step].timestamp,
                    )
                    playback["deadline"] = (
                        time.monotonic()
                        + timestamp_delta / gui_playback_speed.value
                    )
                should_advance = time.monotonic() >= playback["deadline"]
                if should_advance:
                    playback["requested_step"] = current_step + 1
                    playback["deadline"] = None

        time.sleep(0.02 if not should_advance else 0.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Replay a PantoSLAM full binary in Viser."
    )
    parser.add_argument("root", help="Path to the colmap output directory")
    parser.add_argument(
        "replay",
        nargs="?",
        help=(
            "Replay binary path. Defaults to the newest .bin under "
            "ROOT/full_binaries. Relative paths are resolved from ROOT."
        ),
    )
    arguments = parser.parse_args()
    main(arguments.root, arguments.replay)
