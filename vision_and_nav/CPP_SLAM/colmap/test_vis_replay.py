from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import vis_replay


def write_replay(path, version=2, empty=False):
    """Small indexed fixture with distinct dense/sparse geometry per step."""
    data = bytearray(struct.pack("<8sIIQ", b"PANTOVIZ", version, 3, 2))
    index = []
    for step in range(2):
        dense = [] if empty else [[step + 1, 2, 3], [4, 5, 6]]
        sparse = [] if empty else [[step + 10.123456789, 20, 30]]
        counts = (step, float(step), 1, len(dense))
        voxel_header = (-1, -2, -3, 0.1, 4, 3, 1)
        if version == 2:
            header = struct.pack("<QdQQQiiifQQQ", *counts, len(sparse), *voxel_header)
        else:
            header = struct.pack("<QdQQiiifQQQ", *counts, *voxel_header)
        payload = header + struct.pack("<iQQ", 1, 640, 480)
        payload += struct.pack("<4d", 100, 100, 320, 240)  # intrinsics
        payload += struct.pack("<6d", *([0] * 6))  # distortion
        payload += struct.pack("<7d", 1, 0, 0, 0, -step, 0, 0)  # pose
        name = f"{step}.png".encode()
        payload += struct.pack("<Q", len(name)) + name
        payload += struct.pack("<4d", float(step), step, 0, 0)  # tracking
        payload += np.asarray(dense, dtype="<f4").tobytes()
        if version == 2:
            payload += np.asarray(sparse, dtype="<f8").tobytes()
        payload += struct.pack("<QQ", 5, 3)  # nonzero occupancy
        block = struct.pack("<8sQ", b"PVBLOCK1", 16 + len(payload)) + payload
        index.append((step, float(step), len(data), len(block)))
        data.extend(block)
    index_offset = len(data)
    data.extend(struct.pack("<8sQ", b"PVINDEX1", len(index)))
    for entry in index:
        data.extend(struct.pack("<QdQQ", *entry))
    data.extend(struct.pack("<8sQ", b"PVEND001", index_offset))
    path.write_bytes(data)


class ReplayTest(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.path = self.root / "replay.bin"
        write_replay(self.path)

    def test_select_cloud_and_seek_back_without_changing_other_state(self):
        replay = vis_replay.ReplayFile(self.path)
        for step in (0, 1, 0):
            dense_state = replay.state_for_step(step)
            sparse_state = replay.state_for_step(step, dense=False)
            np.testing.assert_array_equal(
                [p.xyz for p in dense_state[3].values()],
                [[step + 1, 2, 3], [4, 5, 6]],
            )
            np.testing.assert_array_equal(
                [p.xyz for p in sparse_state[3].values()],
                [[step + 10.123456789, 20, 30]],
            )
            np.testing.assert_array_equal(dense_state[0], sparse_state[0])
            self.assertEqual(len(dense_state[1]), step + 1)
            self.assertEqual(dense_state[1], sparse_state[1])
            np.testing.assert_array_equal(sparse_state[5]["indices"], [5])
            np.testing.assert_array_equal(sparse_state[5]["counts"], [3])
            np.testing.assert_array_equal(dense_state[5]["counts"], [3])

    def test_empty_clouds(self):
        write_replay(self.path, empty=True)
        replay = vis_replay.ReplayFile(self.path)
        for dense in (True, False):
            self.assertEqual(replay.read_dynamic_data(0, dense=dense)[0].shape, (0, 3))
            self.assertEqual(replay.state_for_step(0, dense=dense)[3], {})

    def test_v1_dense_compatibility_and_explicit_sparse_error(self):
        write_replay(self.path, version=1)
        replay = vis_replay.ReplayFile(self.path)
        self.assertEqual(len(replay.state_for_step(1)[3]), 2)
        with self.assertRaisesRegex(ValueError, "v1 replay contains no sparse points"):
            replay.state_for_step(0, dense=False)
        with patch.object(vis_replay.viser, "ViserServer") as server:
            with self.assertRaisesRegex(ValueError, "v1 replay contains no sparse points"):
                vis_replay.main(str(self.root), "replay.bin", dense=False)
            server.assert_not_called()

    def test_truncated_sparse_payload_is_rejected(self):
        replay = vis_replay.ReplayFile(self.path)
        data = bytearray(self.path.read_bytes())
        # Increase the first block's sparse count without changing its indexed size.
        struct.pack_into("<Q", data, replay.index[0].offset + 16 + 32, 2)
        self.path.write_bytes(data)
        with self.assertRaisesRegex(RuntimeError, "block 0 size mismatch"):
            vis_replay.ReplayFile(self.path)

    def test_command_line_modes(self):
        self.assertTrue(vis_replay.parse_arguments(["colmap"]).dense)
        self.assertTrue(vis_replay.parse_arguments(["colmap", "--dense"]).dense)
        for option in ("--no-dense", "--sparse"):
            args = vis_replay.parse_arguments(["colmap", "saved.bin", option])
            self.assertFalse(args.dense)
            self.assertEqual(args.replay, "saved.bin")

    def run_gui(self, dense=True, version=2):
        write_replay(self.path, version=version)

        class Control(SimpleNamespace):
            def on_update(self, callback):
                self.callback = callback
                return callback

            on_click = on_update

            def trigger(self, value=None):
                if value is not None:
                    self.value = value
                self.callback(None)

        class Gui:
            def __init__(self):
                self.controls = {}

            def add(self, label, *args, **kwargs):
                control = Control(value=kwargs.get("initial_value"), **kwargs)
                self.controls[label] = control
                return control

            add_slider = add_checkbox = add_button = add_text = add

            def add_image(self, image, **kwargs):
                return Control(image=image, **kwargs)

            def add_markdown(self, text):
                pass

        class EndReplay(BaseException):
            pass

        gui = Gui()
        server = SimpleNamespace(gui=gui, on_client_connect=lambda callback: callback)
        rendered = []

        def render(*args):
            rendered.append((len(args[5]), [p.xyz.tolist() for p in args[7].values()]))

        actions = [
            lambda: gui.controls["Dense mode"].trigger(not dense),
            lambda: gui.controls["Skip forward one keyframe"].trigger(),
            lambda: gui.controls["Dense mode"].trigger(dense),
            lambda: gui.controls["Skip back one keyframe"].trigger(),
        ]

        def sleep(_):
            if version == 1 or not actions:
                raise EndReplay()
            actions.pop(0)()

        with patch.object(vis_replay.viser, "ViserServer", return_value=server), \
                patch.object(vis_replay, "update_visualization", side_effect=render), \
                patch.object(vis_replay.time, "sleep", side_effect=sleep), \
                redirect_stdout(StringIO()):
            with self.assertRaises(EndReplay):
                vis_replay.main(str(self.root), "replay.bin", dense=dense)
        return gui, rendered

    def test_gui_toggle_refreshes_paused_frame_and_survives_seeking(self):
        for initial_dense in (True, False):
            with self.subTest(initial_dense=initial_dense):
                gui, rendered = self.run_gui(dense=initial_dense)
                self.assertEqual(gui.controls["Dense mode"].initial_value, initial_dense)
                self.assertFalse(gui.controls["Dense mode"].disabled)
                self.assertEqual([item[0] for item in rendered], [1, 1, 2, 2, 1])
                expected_sizes = [2, 1, 1, 2, 2] if initial_dense else [1, 2, 2, 1, 1]
                self.assertEqual([len(item[1]) for item in rendered], expected_sizes)

    def test_v1_gui_disables_dense_toggle(self):
        gui, rendered = self.run_gui(version=1)
        self.assertTrue(gui.controls["Dense mode"].disabled)
        self.assertEqual(len(rendered), 1)


if __name__ == "__main__":
    unittest.main()
