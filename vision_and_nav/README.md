# PantoPilot vision and navigation

PantoPilot's SLAM implementation lives in [`CPP_SLAM`](CPP_SLAM). It uses
feature tracking, keyframes, a covisibility graph, and local bundle adjustment
to estimate camera motion and build a map. The current development focus is
stereo visual-inertial SLAM on EuRoC MH01 (`MH_01_easy`).

The system is heavily inspired by ORB-SLAM and ORB-SLAM3. It currently uses
AKAZE keypoints with TEBLID descriptors, DBoW3 vocabulary matching, Ceres
optimization, and stereo depth for dense mapping. Monocular visual
initialization and Stereo-NEC-based inertial initialization are also present;
see [References](#references) for their sources. This is an experimental
implementation; the references do not imply that every upstream feature is
implemented here.

## Directory layout

| Path | Contents |
| --- | --- |
| [`CPP_SLAM/MIT/include`](CPP_SLAM/MIT/include), [`CPP_SLAM/MIT/src`](CPP_SLAM/MIT/src) | SLAM configuration, tracking, mapping, initialization, IMU integration, optimization, and export |
| [`CPP_SLAM/makefile`](CPP_SLAM/makefile) | Main C++ build and current test targets |
| [`CPP_SLAM/MIT/test`](CPP_SLAM/MIT/test) | C++ regression tests |
| [`CPP_SLAM/colmap`](CPP_SLAM/colmap) | Python live visualization and saved-map/replay viewers |
| [`CPP_SLAM/MIT/PiImagereader`](CPP_SLAM/MIT/PiImagereader) | Stereo image receiver and calibration utilities |
| [`python_testing`](python_testing) | Earlier camera calibration, ArUco, and detection experiments |

## C++ requirements

The existing build targets macOS with Apple Clang, Make, and Homebrew. It
requires **C++20**. CMake and Git are needed when building DBoW3 from source;
the SLAM executable itself is built with Make, not CMake.

| Dependency | Use |
| --- | --- |
| OpenCV **4.x**, including `opencv_contrib` / `xfeatures2d` | Image processing, calibration, feature detection, TEBLID, stereo matching, and image/video I/O |
| Eigen | Linear algebra |
| Ceres Solver | Pose optimization and bundle adjustment |
| Sophus | Lie-group operations for inertial optimization |
| DBoW3 | Visual vocabulary and feature matching |
| glog, gflags | Solver dependencies explicitly linked by the makefile |
| SuiteSparse, OpenBLAS, TBB | Numerical libraries and threading dependencies used by the build |
| libomp and Apple's Accelerate framework | macOS runtime/threading and numerical support |

A local stereo-inertial release build was checked on 2026-11-12 with Apple
Clang 17, OpenCV 4.13.0, Ceres 2.2.0, Eigen 5.0.1, and Sophus 1.24.6.
These are observed versions, not tested minimum requirements.

The makefile discovers most libraries with `brew --prefix`, and defaults to
`/usr/local/include` and `/usr/local/lib` for DBoW3. Those paths can be
overridden as shown below. Linux and case-sensitive filesystems need further
portability work, including capitalization of some include paths.

## Setup and build

From the PANTOPILOT repository root:

```sh
cd vision_and_nav/CPP_SLAM
```

Run the remaining commands from `CPP_SLAM`.

### 1. Install dependencies

Install Apple's Command Line Tools if they are not already installed:

```sh
xcode-select --install
```

With Homebrew installed:

```sh
brew install cmake opencv@4 eigen ceres-solver sophus glog gflags \
  suite-sparse openblas tbb libomp
OPENCV_PREFIX="$(brew --prefix opencv@4)"
```

Use the [OpenCV 4 formula](https://formulae.brew.sh/formula/opencv@4) because
the makefile expects OpenCV 4 headers and libraries. On an existing setup
where `brew --prefix opencv` points to OpenCV 4, use
`OPENCV_PREFIX="$(brew --prefix opencv)"` instead.

DBoW3 source and build products under `vendor` are ignored by Git. If
`vendor/DBow3` is missing, obtain it from the
[upstream repository](https://github.com/rmsalinas/DBow3):

```sh
mkdir -p vendor
git clone https://github.com/rmsalinas/DBow3.git vendor/DBow3
```

Build and install DBoW3 into a local prefix, using the same OpenCV as SLAM:

```sh
DBOW3_PREFIX="$PWD/vendor/DBow3/install"
cmake -S vendor/DBow3 -B vendor/DBow3/build-local \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_PREFIX="$DBOW3_PREFIX" \
  -DOpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4" \
  -DBUILD_UTILS=OFF
cmake --build vendor/DBow3/build-local --parallel
cmake --install vendor/DBow3/build-local
```

The policy option allows older DBoW3 CMake files to configure with CMake 4.
If DBoW3 is already installed in `/usr/local`, the `EXT_INCLUDE_DIR` and
`EXT_LIB_DIR` overrides in the build command below can be omitted.

### 2. Prepare visualization and local paths

Python is used for visualization, separately from the C++ dependencies:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
mkdir -p logs/Latest
```

Before building on a different checkout, update
[`MIT/include/Config.hpp`](CPP_SLAM/MIT/include/Config.hpp). It currently
contains machine-specific absolute paths. Set:

- `PANTO_PATH_TO_PYTHON_INTERPRETER` to your virtual environment's Python
  executable, for example `"./.venv/bin/python"`.
- `PANTO_LOGPATH` to `"./logs/Latest"` and `PANTO_LOGPATH_HISTORICAL` to
  `"./logs/Historical"`, or equivalent writable absolute paths.

Keep the working directory at `CPP_SLAM`: vocabulary, dataset, viewer, and
export paths are resolved from there. The default descriptor vocabulary is
`PantoTeblidVocabulary.dbow3`, which is included in the repository.

### 3. Build SLAM

For stereo-inertial SLAM with initialization independent of ground truth:

```sh
make main CONFIG_STEREO=1 CONFIG_IMU=1 USE_GT_INIT=0 \
  OPENCV="$OPENCV_PREFIX" \
  EXT_INCLUDE_DIR="$DBOW3_PREFIX/include" \
  EXT_LIB_DIR="$DBOW3_PREFIX/lib"
```

The makefile builds `MIT/src/main.cpp` as one translation unit, including
the implementation files, and writes `./main`.

| Build option | Effect |
| --- | --- |
| `CONFIG_STEREO=1` | Enable stereo processing and dense mapping |
| `CONFIG_IMU=1` | Enable inertial integration and optimization |
| `USE_GT_INIT=0` | Disable ground-truth initialization; the stereo-inertial path starts with a metric stereo seed, then attempts inertial initialization |
| `USE_GT_INIT=1` | Use ground-truth states during initialization |
| `DEBUG=1` | Use `-O0`, debug symbols, and AddressSanitizer; disable the viewer and replay export |
| `CONFIG_PLANAR=1` | Enable the planar floor filter for dense mapping |

**Ground-truth initialization defaults to enabled** in `Config.hpp` when
`USE_GT_INIT` is omitted. Choose it explicitly when recording results.
The release build uses `-O3`; stereo and IMU support also need to be enabled
explicitly. Rebuild after changing configuration or build options.

## Run EuRoC MH01

Download the `MH_01_easy` sequence from the
[EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/)
and arrange the extracted data as follows:

```text
CPP_SLAM/datasets/EuRoC/machine_hall/MH_01_easy/mav0/
├── cam0/                       # data.csv, sensor.yaml, data/*.png
├── cam1/                       # data.csv, sensor.yaml, data/*.png
├── imu0/                       # data.csv, sensor.yaml
└── state_groundtruth_estimate0/ # data.csv, sensor.yaml
```

`PANTO_ACTIVE_DATASET` defaults to `EUROC_MAV_MACHINE_HALL1_EASY` in
`Config.hpp`; calibration and dataset paths are compiled into the program.
With the build and Python paths configured:

```sh
./main
```

An optional integer argument limits the tracking-loop iterations, for example
`./main 500`; it is not a duration in seconds. Without an argument the limit
is 100000, and processing also stops when the dataset ends. The release
build starts the Python viewer and waits for Enter at completion to close it.

`PANTO_DATASET_REALTIME_MODE` defaults to `true`: replay follows dataset
timestamps and skips frames when tracking is late. Set it to `false` in
`Config.hpp` and rebuild to consume every frame and serialize each queued
keyframe with its local-mapping iteration. Record the replay mode when
comparing runs.

Exports go to `colmap/output`. Each run refreshes its `images` and `sparse`
directories; replay binaries are saved under `full_binaries` with timestamps.
Open the newest saved replay with:

```sh
.venv/bin/python colmap/vis_replay.py colmap/output
```

See the [viewer README](CPP_SLAM/colmap/README.md) for dense/sparse display
options and replay compatibility.

## Recent MH01 results

The four latest locally available completed DATA logs, inspected on
2026-11-12, report the following values. Run IDs are the timestamps in the
log filenames, newest first.

| Run ID | Mean ATE (m) | Mean RRE (degrees) | GT-matched / recorded poses | Late frames skipped |
| --- | ---: | ---: | ---: | ---: |
| [2026-11-11_21-51-25](CPP_SLAM/logs/Latest/DATAPantoLOG_2026-11-11_21-51-25) | 0.094841 | 0.113358 | 3620 / 3664 | 18 |
| [2026-11-10_23-14-46](CPP_SLAM/logs/Historical/DATAPantoLOG_2026-11-10_23-14-46) | 0.065452 | 0.106587 | 3635 / 3679 | 3 |
| [2026-11-10_22-37-19](CPP_SLAM/logs/Historical/DATAPantoLOG_2026-11-10_22-37-19) | 0.091170 | 0.119086 | 3351 / 3392 | 290 |
| [2026-11-10_21-56-43](CPP_SLAM/logs/Historical/DATAPantoLOG_2026-11-10_21-56-43) | 0.364403 | 0.157166 | 2954 / 2984 | 698 |

These are the logged **arithmetic means, not RMSE**. Their definitions are
implemented in [`GT_Alignment.cpp`](CPP_SLAM/MIT/src/GT_Alignment.cpp):

- **ATE:** mean Euclidean camera-position error after a least-squares rigid
  SE(3) alignment to ground truth, with scale fixed at 1.
- **RRE:** mean angular difference between estimated and reference relative
  rotations for consecutive valid recorded poses. It uses adjacent poses,
  rather than a fixed one-second interval; skipped frames change the interval.
- Ground-truth body poses are interpolated at camera timestamps and converted
  to camera poses using `T_BS`. Samples outside the GT time range are excluded.

The logs do not record a full build configuration, Git revision, or whether
ground-truth initialization was enabled. These are development-run results
with different frame coverage, rather than a controlled no-GT benchmark or
a direct comparison with the papers. In particular, the default GT setting
above should not be inferred to have been disabled for these saved runs.

The summaries can be read from `CPP_SLAM` with:

```sh
rg '\[SLAMGTAlignment\] SE3 applied|\[SLAMFramePacingSummary\]' \
  logs/Latest/DATAPantoLOG* logs/Historical/DATAPantoLOG*
```

Starting another run moves the previous `logs/Latest` files into
`logs/Historical`. Logs are ignored by Git, so the links above refer to local
artifacts; the latest run's link will need its directory updated after log
rotation. The table preserves the observed results for readers without logs.

## References

1. W. Wang et al., **Stereo-NEC: Enhancing Stereo Visual-Inertial SLAM
   Initialization with Normal Epipolar Constraints**, 2024.
   [Paper](https://arxiv.org/abs/2403.07225).
   Basis for gyroscope-bias estimation, inertial state refinement, rotation
   updates from preintegration, and translation-only bundle adjustment in
   `OP_BA.*`, called by the stereo-inertial startup path in `SL_SLAM.cpp`.
2. H. Dou, B. Liu, Y. Jia, and C. Wang, **Monocular Initialization for
   Real-Time Feature-Based SLAM in Dynamic Environments with Multiple
   Frames**, *Sensors* 25(8), 2404, 2025.
   [DOI: 10.3390/s25082404](https://doi.org/10.3390/s25082404).
   Basis for the independent multi-frame monocular visual initializer in
   `INIT_InitializeSLAM.*`, as acknowledged in its header.
3. R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós, **ORB-SLAM: A Versatile
   and Accurate Monocular SLAM System**, *IEEE Transactions on Robotics*,
   2015. [Paper](https://arxiv.org/abs/1502.00956).
   General inspiration for feature-based tracking, keyframes, covisibility,
   map-point management, and local bundle adjustment.
4. C. Campos et al., **ORB-SLAM3: An Accurate Open-Source Library for Visual,
   Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics*,
   2021. [Paper](https://arxiv.org/abs/2007.11898),
   [implementation](https://github.com/UZ-SLAMLab/ORB_SLAM3).
   Further inspiration for visual-inertial optimization and local mapping.
