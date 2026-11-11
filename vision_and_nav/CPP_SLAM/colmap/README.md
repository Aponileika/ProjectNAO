# Saved-data visualization

Install the visualization dependencies from the repository root:

```sh
python -m pip install -r requirements.txt
```

Open the newest replay in `colmap/output/full_binaries` with dense points (the default):

```sh
python colmap/vis_replay.py colmap/output
```

Start with sparse landmarks instead:

```sh
python colmap/vis_replay.py colmap/output --no-dense
```

`--sparse` is an alias for `--no-dense`; `--dense` explicitly selects the default.
An optional replay path follows the root, for example
`python colmap/vis_replay.py colmap/output full_binaries/saved.bin --sparse`.

The **Dense mode** checkbox switches between dense reconstruction points and
the sparse landmarks used by local mapping. It refreshes the current keyframe
while paused and also works during playback. The occupancy overlay always
comes from the dense reconstruction.

New post-run replay exports (format v2) contain both clouds for each keyframe's
temporally grounded local map, using the final optimized poses and landmarks.
Rebuild SLAM and record a new run to export both. Existing v1 replays still
open in dense mode, with the mode checkbox disabled because those files do
not contain sparse landmarks.
