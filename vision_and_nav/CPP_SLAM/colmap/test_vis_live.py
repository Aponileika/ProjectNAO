from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import vis_live


class LiveSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        for snapshot_id, mode in enumerate(("replace", "append", "append", "replace")):
            path = self.root / "sparse" / "snapshots" / str(snapshot_id)
            path.mkdir(parents=True)
            (path / "keyframes_mode.txt").write_text(mode)

    def test_tracking_prefix_is_frozen_until_full_replacement(self):
        initial = np.array([[1., 0., 0.], [2., 0., 0.]])
        incoming = np.array([[10., 0., 0.], [20., 0., 0.], [3., 0., 0.]])
        displayed = vis_live.append_tracking_frames(None, initial, False)
        appended = vis_live.append_tracking_frames(displayed, incoming, False)
        np.testing.assert_array_equal(appended, [[1., 0., 0.], [2., 0., 0.], [3., 0., 0.]])
        self.assertIs(vis_live.append_tracking_frames(appended, incoming, False), appended)
        np.testing.assert_array_equal(vis_live.append_tracking_frames(appended, incoming, True), incoming)

    def test_legacy_full_snapshot(self):
        (self.root / "sparse/snapshots/2/keyframes_mode.txt").unlink()
        self.assertEqual(vis_live.read_keyframe_mode(self.root, 2), "replace")

    def test_missed_map_reset_clears_history_without_replaying_kfs(self):
        path = self.root / "sparse/snapshots/4"
        path.mkdir()
        (path / "keyframes_mode.txt").write_text("append")
        self.assertFalse(vis_live.live_snapshot_replaces_history(self.root, 0, 2))
        self.assertTrue(vis_live.live_snapshot_replaces_history(self.root, 2, 4))
        self.assertTrue(vis_live.live_snapshot_replaces_history(self.root, 2, 3))

    def test_viewer_only_applies_latest_kf_and_freezes_old_tracking_positions(self):
        class Handle(SimpleNamespace):
            def remove(self):
                self.removed = True

            def on_update(self, callback):
                return callback

        class Scene:
            def __init__(self):
                self.frustums = {}
                self.tracking = None

            def add_camera_frustum(self, name, **kwargs):
                handle = Handle(**kwargs)
                self.frustums[name] = handle
                return handle

            def add_line_segments(self, name, **kwargs):
                handle = Handle(**kwargs)
                if name == "/tracking/trajectory":
                    self.tracking = handle
                return handle

        class Gui:
            def add_slider(self, name, **kwargs):
                return Handle(value=kwargs["initial_value"])

            add_checkbox = add_slider

            def add_markdown(self, text):
                pass

        class EndPolls(Exception):
            pass

        scene = Scene()
        server = SimpleNamespace(scene=scene, gui=Gui())
        camera = SimpleNamespace(model="PINHOLE", params=[100, 100, 50, 50], width=100, height=100)

        def image(position):
            return SimpleNamespace(camera_id=1, qvec=np.array([1., 0., 0., 0.]), tvec=np.array([-position, 0., 0.]))

        images = {
            0: {1: image(1)},
            1: {2: image(2)},
            2: {3: image(3)},
            3: {1: image(10), 2: image(20), 3: image(30)},
        }
        loaded = []

        def load(root, snapshot_id):
            loaded.append(snapshot_id)
            tracks = np.array([[0., 0., 0.], [1., 0., 0.]])
            if snapshot_id == 2:
                tracks = np.array([[90., 0., 0.], [91., 0., 0.], [2., 0., 0.]])
            elif snapshot_id == 3:
                tracks = np.array([[10., 0., 0.], [11., 0., 0.], [12., 0., 0.]])
            return tracks, {1: camera}, images[snapshot_id]

        observed = []
        tracking = []

        def sleep(_):
            observed.append({int(name.rsplit("_", 1)[1]): float(handle.position[0])
                             for name, handle in scene.frustums.items()})
            tracking.append(scene.tracking.points.copy())
            if len(observed) == 3:
                raise EndPolls()

        with patch.object(vis_live.viser, "ViserServer", return_value=server), \
                patch.object(vis_live, "initialize_view_from_first_camera"), \
                patch.object(vis_live, "read_latest_snapshot", side_effect=[0, 2, 3]), \
                patch.object(vis_live, "load_live_snapshot", side_effect=load), \
                patch.object(vis_live.time, "sleep", side_effect=sleep):
            with self.assertRaises(EndPolls):
                vis_live.main(str(self.root))

        self.assertEqual(observed, [{1: 1.}, {1: 1., 3: 3.}, {1: 10., 3: 30., 2: 20.}])
        self.assertEqual(loaded, [0, 2, 3])
        np.testing.assert_array_equal(tracking[1][:, :, 0], [[0., 1.], [1., 2.]])
        np.testing.assert_array_equal(tracking[2][:, :, 0], [[10., 11.], [11., 12.]])


if __name__ == "__main__":
    unittest.main()
