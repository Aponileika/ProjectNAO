#ifndef CONFIG_HPP_
#define CONFIG_HPP_
#include <random>
#include <string>
#include <cmath>
#include <array>
#include "CArenaAlloc.h"

inline std::mt19937& PANTO_GetRandomGenerator(void)
{
    static std::random_device RandomDevice;
    static std::mt19937 Generator(RandomDevice());

    return Generator;
}

inline i64 PANTO_GetUniformRandom( const u64 Low, const u64 High)
{
    std::uniform_int_distribution<u64> Distribution(Low, High);

    return static_cast<i64>(Distribution(PANTO_GetRandomGenerator()));
}

inline constexpr const char* PANTO_SLAMSTARTMSG =
"        _____                 ^\n"
"    ___/_____\\___             |\n"
"  _/  _     _  \\_        .----O----.      [o_o]\n"
" /__/(_)---(_)\\__\\_______|         |_____/|___|\\\n"
"        \\______/          \\________/      /|   |\\\n"
"\n"
"              P a n t o P i l o t\n";

// #define PANTO_DBG

#define CERES_MAX_ITER 200
#define CERES_NUM_THREADS 4
#define CERES_HUBER_THRESHOLD 2.5

// #define OPENCV_AKAZETHRESHOLD 0.001
#define OPENCV_AKAZETHRESHOLD 0.001
#define OPENCV_AKAZE_NOCTAVES 4
#define OPENCV_AKAZE_NOCTAVELAYERS 4
#define PANTO_DESCRIPTOR_ANMS false
//Initializes with Ground truth frame data, to avoid having to code monocular IMU initialization
#if defined(CONFIG_IMU)
    #define PANTO_GROUNDTRUTH_INIT true
#else
    #define PANTO_GROUNDTRUTH_INIT false
#endif 

#define PANTO_DESCRIPTOR_SIZE 61 //Bytes
#define PANTO_LOCAL_MAP_SAMPLE_STRIDE 5
#define PANTO_NUM_BOOTSTRAP_FRAMES 100
// [Number of frames], from bootstrap learning mean distance * number of frames
// should trigger keyframe insertion.
#define PANTO_KEYFRAME_MEAN_DISTANCE_THRESHOLD_GAIN 10
#define PANTO_KEYFRAME_MEAN_VELOCITY_THRESHOLD_GAIN 4
#define PANTO_KEYFRAME_MEAN_TRACKING_HIGH_THRESHOLD_GAIN 0.5f
#define PANTO_KEYFRAME_MEAN_TRACKING_LOW_THRESHOLD_GAIN 0.1f
#define PANTO_KEYFRAME_FUZZY_MAX_RULE_THRESHOLD 0.98f
#define PANTO_KEYFRAME_FUZZY_SPATIAL_TRACKING_THRESHOLD 0.6f
#define PANTO_TRACKING_MIN_MATCHED_MAP_POINTS 10
#define PANTO_VISUAL_ALIGNMENT_WINDOW_FRAMES 30
// Same as slam orb, baseline > 1% of median depth of local map relative to a keyframe
#define PANTO_BASELINE_THRESHOLD 0.01f
#define PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth) ((BaseLine / MedianDepth) > PANTO_BASELINE_THRESHOLD)
#define PANTO_FEATURE_TRACK_NOT_OBSERVED -1
#if defined(DEBUG)
    #define PANTO_INIT_MIN_NUM_FRAMES 2
#else
    #define PANTO_INIT_MIN_NUM_FRAMES 10
#endif
constexpr const char* PANTO_COLMAP_PATH = "./colmap";
constexpr const char* PANTO_COLMAP_PYTHON_SCRIPT_PATH = "./colmap/vis_colmap.py";
constexpr const char* PANTO_PATH_TO_PYTHON_INTERPRETER = "/Users/Jonathan/Programmering/FIA/PANTOPILOT/vision_and_nav/CPP_SLAM/.venv/bin/python";
#if defined(DEBUG)
    #define CONFIG_PRINT_LOGS_TO_STDOUT true
#else
    #define CONFIG_PRINT_LOGS_TO_STDOUT true
#endif
#define PANTO_MIN_FOUND_RATIO 0.25
#define PANTO_INIT_MAX_REPROJECTION_ERROR 2.5
#define PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED 2.5*2.5
#define PANTO_INIT_MIN_PARALLAX_DEGREES 1.0
#define PANTO_TOP_N_KF_FOR_LOCAL_MAP 30
#define PANTO_MAX_LOCAL_TRACKING_MAP_SIZE 80
#define PANTO_PIXEL_CHI_SQUARED_T 5.991
#define PANTO_PIXEL_CHI_SQUARED_T_SQRT 2.448
#define PANTO_IMAGEPOINT_RESERVE 1000

#if defined(CONFIG_IMU)
#define PANTO_NUM_TEMPORALLY_CONNECTED_KFS_LOCAL_BA 21
#define PANTO_MAX_FIXED_KFS_LOCAL_BA 200
#endif

constexpr fp64 PANTO_MINIMUMPARALLAX = 1.0 * M_PI / 180.0;

inline const fp64 PANTO_MAXIMUMCOSPARALLAX = std::cos(PANTO_MINIMUMPARALLAX);

using PantoClock = std::chrono::steady_clock;

#define PANTO_USE_DATASET true
#define PANTO_DATASET_BASE_PATH "./datasets"

#ifndef PANTO_ACTIVE_DATASET
    #define PANTO_ACTIVE_DATASET EUROC_MAV_VICON_ROOM1_EASY 
#endif

#define DATASETS \
    X(TUM_FREIBURG1_XYZ, "/tum/rgbd_dataset_freiburg1_xyz") \
    X(TUM_FREIBURG1_RPY, "/tum/rgbd_dataset_freiburg1_rpy") \
    X(TUM_FREIBURG2_XYZ, "/tum/rgbd_dataset_freiburg2_xyz") \
    X(TUM_FREIBURG2_PIONEER_SLAM, "/tum/rgbd_dataset_freiburg2_pioneer_slam") \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, "/tum/rgbd_dataset_freiburg3_long_office_household") \
    X(EUROC_MAV_VICON_ROOM1_EASY, "/EuRoC/vicon_room1/V1_01_easy/mav0")

#define DATASET_SEQUENCES \
    X(TUM_FREIBURG1_XYZ, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG1_RPY, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG2_XYZ, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG2_PIONEER_SLAM, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, RGB_ORDERED, "rgb_ordered") \
    X(EUROC_MAV_VICON_ROOM1_EASY, RGB_ORDERED, "/cam0/data")

#define DATASET_IMUS \
    X(TUM_FREIBURG1_XYZ, IMU_MEASUREMENTS, "") \
    X(TUM_FREIBURG1_RPY, IMU_MEASUREMENTS, "") \
    X(TUM_FREIBURG2_XYZ, IMU_MEASUREMENTS, "") \
    X(TUM_FREIBURG2_PIONEER_SLAM, IMU_MEASUREMENTS, "") \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, IMU_MEASUREMENTS, "") \
    X(EUROC_MAV_VICON_ROOM1_EASY, IMU_MEASUREMENTS, "/imu0")

#define DATASET_GT \
    X(TUM_FREIBURG1_XYZ, GT_POSE, "") \
    X(TUM_FREIBURG1_RPY, GT_POSE, "") \
    X(TUM_FREIBURG2_XYZ, GT_POSE, "") \
    X(TUM_FREIBURG2_PIONEER_SLAM, GT_POSE, "") \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, GT_POSE, "") \
    X(EUROC_MAV_VICON_ROOM1_EASY, GT_POSE, "/state_groundtruth_estimate0")

// Sensor extrinsics are T_BS: sensor frame with respect to the body frame.
#define PANTO_T_BS_IDENTITY \
    1.0, 0.0, 0.0, 0.0, \
    0.0, 1.0, 0.0, 0.0, \
    0.0, 0.0, 1.0, 0.0, \
    0.0, 0.0, 0.0, 1.0

#define PANTO_T_BS_EUROC_CAM0 \
     0.0148655429818, -0.999880929698,    0.00414029679422, -0.0216401454975, \
     0.999557249008,   0.0149672133247,   0.025715529948,   -0.064676986768, \
    -0.0257744366974,  0.00375618835797,  0.999660727178,    0.00981073058949, \
     0.0,               0.0,               0.0,                1.0

// fx, fy, s, cx, cy, k1, k2, p1, p2, k3, width, height, rate_hz, T_BS.
// A zero rate means that the sampling rate has not been configured.
#define DATASET_INTRINSICS \
    X(TUM_FREIBURG1_XYZ, 517.3, 516.5, 0.0, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633, 640, 480, 30.0, PANTO_T_BS_IDENTITY) \
    X(TUM_FREIBURG1_RPY, 517.3, 516.5, 0.0, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633, 640, 480, 30.0, PANTO_T_BS_IDENTITY) \
    X(TUM_FREIBURG2_XYZ, 520.9, 521.0, 0.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172, 640, 480, 30.0, PANTO_T_BS_IDENTITY) \
    X(TUM_FREIBURG2_PIONEER_SLAM, 520.9, 521.0, 0.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172, 640, 480, 30.0, PANTO_T_BS_IDENTITY) \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, 535.4, 539.2, 0.0, 320.1, 247.6, 0.0, 0.0, 0.0, 0.0, 0.0, 640, 480, 30.0, PANTO_T_BS_IDENTITY) \
    X(EUROC_MAV_VICON_ROOM1_EASY, 458.654, 457.296, 0.0, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0, 752, 480, 20.0, PANTO_T_BS_EUROC_CAM0) \
    X(WEBCAM_JE, 974.7187409387847, 976.5223334221673, 0.0, 666.3249058750432, 337.4737864029501, 0.06475901025911835, -0.1903655376657792, -0.003666863513699757, 0.002119531347424837, 0.1113497353944944, 640, 480, 0.0, PANTO_T_BS_IDENTITY)

// rate_hz, T_BS, gyroscope noise density, gyroscope random walk,
// accelerometer noise density, accelerometer random walk
#define DATASET_IMU_INTRINSICS \
    X(TUM_FREIBURG1_XYZ, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(TUM_FREIBURG1_RPY, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(TUM_FREIBURG2_XYZ, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(TUM_FREIBURG2_PIONEER_SLAM, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(TUM_FREIBURG3_LONG_OFFICE_HOUSEHOLD, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(WEBCAM_JE, 0.0, PANTO_T_BS_IDENTITY, 0.0, 0.0, 0.0, 0.0) \
    X(EUROC_MAV_VICON_ROOM1_EASY, 200.0, PANTO_T_BS_IDENTITY, 1.6968e-04, 1.9393e-05, 2.0000e-3, 3.0000e-3)

enum class Dataset : u8
{
#define X(name, ...) name,
    DATASET_INTRINSICS
#undef X
};

#define X(dataset, path) \
    constexpr const char* DATASET_PATH_##dataset = path;
DATASETS
#undef X

#define X(dataset, seq, folder) \
    constexpr const char* SEQUENCE_PATH_##dataset##_##seq = folder;
DATASET_SEQUENCES
#undef X

#define X(dataset, seq, folder) \
    constexpr const char* IMU_MEASUREMENTS_##dataset##_##seq = folder;
DATASET_IMUS
#undef X

#define X(dataset, seq, folder) \
    constexpr const char* GT_##dataset##_##seq = folder;
DATASET_GT
#undef X

#define PANTO_EXPAND_DATASET_PATH_IMPL(dataset) DATASET_PATH_##dataset
#define PANTO_EXPAND_DATASET_PATH(dataset) PANTO_EXPAND_DATASET_PATH_IMPL(dataset)

#define PANTO_EXPAND_SEQUENCE_PATH_IMPL(dataset) SEQUENCE_PATH_##dataset##_RGB_ORDERED
#define PANTO_EXPAND_SEQUENCE_PATH(dataset) PANTO_EXPAND_SEQUENCE_PATH_IMPL(dataset)

#define PANTO_EXPAND_IMU_PATH_IMPL(dataset) IMU_MEASUREMENTS_##dataset##_IMU_MEASUREMENTS
#define PANTO_EXPAND_IMU_PATH(dataset) PANTO_EXPAND_IMU_PATH_IMPL(dataset)

#define PANTO_EXPAND_GT_PATH_IMPL(dataset) GT_##dataset##_GT_POSE
#define PANTO_EXPAND_GT_PATH(dataset) PANTO_EXPAND_GT_PATH_IMPL(dataset)

constexpr auto panto_dataset_path =
    PANTO_EXPAND_DATASET_PATH(PANTO_ACTIVE_DATASET);

constexpr auto panto_sequence_path =
    PANTO_EXPAND_SEQUENCE_PATH(PANTO_ACTIVE_DATASET);

constexpr auto panto_imu_path =
    PANTO_EXPAND_IMU_PATH(PANTO_ACTIVE_DATASET);

constexpr auto panto_gt_path =
    PANTO_EXPAND_GT_PATH(PANTO_ACTIVE_DATASET);

const Dataset panto_dataset =
    Dataset::PANTO_ACTIVE_DATASET;

#define PANTO_LOGPATH "/Users/Jonathan/Programmering/FIA/PANTOPILOT/vision_and_nav/CPP_SLAM/logs/Latest"
#define PANTO_LOGPATH_HISTORICAL "/Users/Jonathan/Programmering/FIA/PANTOPILOT/vision_and_nav/CPP_SLAM/logs/Historical"

#define CV_NFEATURES 2000
#define PANTO_MATCHRATIO 0.8f

#define CV_PNPRANSACITS 2000 
#define CV_REPROJERR 2.0F
#define CV_CONF 0.99F
#define PANTO_PNPPOINTCNTTHRESHOLD 20
#define PANTO_NONPNPTHRESHOLD 30 
#define PANTO_SEARCHWINDOW2D3D 2

#define PANTO_CAMERA_MODEL_ID 1 //pinhole

#define PANTO_EPIPOLARTRESHOLD 4.0f

#define PANTO_PIXEL_MEAS_STD_DEV 1.0f

#define PANTO_CELL_SIZE 32

#define PANTO_MAPPOINT_MATCH_SEARCH_RADIUS 20.0f

//arbitrary, now same as slam orb
#define PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD 200
#define PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW 100

#define PANTO_ID_NOT_SET U64_MAX

#define PANTO_TIMESTAMP_NOT_SET -1.0f

inline constexpr const char* PANTO_VocabFilePath =
    "PantoVocabulary.dbow3";

// Controls how many children in vocab tree
#define PANTO_DBOW_BRANCHING_FACTOR 10

// Controls how deep vocab tree 
#define PANTO_DBOW_DEPTH 5

#define PANTO_DBOW_LEVELSUP (PANTO_DBOW_DEPTH - 1)

#define PANTO_INIT_ERROR_THRESHOLD_INLIER_HOMOGRAPHY 5.991*PANTO_PIXEL_MEAS_STD_DEV*PANTO_PIXEL_MEAS_STD_DEV

#define PANTO_INIT_ERROR_THRESHOLD_INLIER_FUNDAMENTAL 3.841*PANTO_PIXEL_MEAS_STD_DEV*PANTO_PIXEL_MEAS_STD_DEV

#define PANTO_INIT_STRANSAC_RATIO_INLIER_OUTLIER_THRESHOLD 0.6f

#define PANTO_INIT_RANSAC_LOOP_CNT 1000

#define PANTO_FUNDAMENTAL_MIN_POINTS 8
#define PANTO_HOMOGRAPHY_MIN_POINTS 4
#define PANTO_INIT_MIN_STATIONARY_POINTS 100
#define PANTO_MIN_NUMBER_INITIAL_MAP_POINTS 30
#define PANTO_MIN_INITIALIZATION_BASELINE_METERS 0.1
#define PANTO_NUM_THREADS_MAX 4
#define PANTO_INIT_CANDIDATE_BATCHES 5

/*                      
 ******************************************************************                                                  
 ******************************************************************                                                  
 ******************************************************************                                                  
 *
 *              Platform dependent 
 *
 ******************************************************************                                                  
 ******************************************************************                                                  
 ******************************************************************                                                  
 */

#define X(name, fx, fy, s, cx, cy, k1, k2, p1, p2, k3, width, height, rate_hz, t_bs) \
    constexpr u64 DATASET_IMAGE_WIDTH_##name = width; \
    constexpr u64 DATASET_IMAGE_HEIGHT_##name = height; \
    constexpr fp64 DATASET_CAMERA_RATE_HZ_##name = rate_hz;
DATASET_INTRINSICS
#undef X

#define X(name, rate_hz, t_bs, gyro_noise_density, gyro_random_walk, accel_noise_density, accel_random_walk) \
    constexpr fp64 DATASET_IMU_RATE_HZ_##name = rate_hz; \
    constexpr fp64 DATASET_GYROSCOPE_NOISE_DENSITY_##name = gyro_noise_density; \
    constexpr fp64 DATASET_GYROSCOPE_RANDOM_WALK_##name = gyro_random_walk; \
    constexpr fp64 DATASET_ACCELEROMETER_NOISE_DENSITY_##name = accel_noise_density; \
    constexpr fp64 DATASET_ACCELEROMETER_RANDOM_WALK_##name = accel_random_walk;
DATASET_IMU_INTRINSICS
#undef X

#define PANTO_EXPAND_IMAGE_WIDTH_IMPL(dataset) DATASET_IMAGE_WIDTH_##dataset
#define PANTO_EXPAND_IMAGE_WIDTH(dataset) PANTO_EXPAND_IMAGE_WIDTH_IMPL(dataset)
#define PANTO_EXPAND_IMAGE_HEIGHT_IMPL(dataset) DATASET_IMAGE_HEIGHT_##dataset
#define PANTO_EXPAND_IMAGE_HEIGHT(dataset) PANTO_EXPAND_IMAGE_HEIGHT_IMPL(dataset)
#define PANTO_EXPAND_CAMERA_RATE_HZ_IMPL(dataset) DATASET_CAMERA_RATE_HZ_##dataset
#define PANTO_EXPAND_CAMERA_RATE_HZ(dataset) PANTO_EXPAND_CAMERA_RATE_HZ_IMPL(dataset)
#define PANTO_EXPAND_IMU_RATE_HZ_IMPL(dataset) DATASET_IMU_RATE_HZ_##dataset
#define PANTO_EXPAND_IMU_RATE_HZ(dataset) PANTO_EXPAND_IMU_RATE_HZ_IMPL(dataset)
#define PANTO_EXPAND_GYROSCOPE_NOISE_DENSITY_IMPL(dataset) DATASET_GYROSCOPE_NOISE_DENSITY_##dataset
#define PANTO_EXPAND_GYROSCOPE_NOISE_DENSITY(dataset) PANTO_EXPAND_GYROSCOPE_NOISE_DENSITY_IMPL(dataset)
#define PANTO_EXPAND_GYROSCOPE_RANDOM_WALK_IMPL(dataset) DATASET_GYROSCOPE_RANDOM_WALK_##dataset
#define PANTO_EXPAND_GYROSCOPE_RANDOM_WALK(dataset) PANTO_EXPAND_GYROSCOPE_RANDOM_WALK_IMPL(dataset)
#define PANTO_EXPAND_ACCELEROMETER_NOISE_DENSITY_IMPL(dataset) DATASET_ACCELEROMETER_NOISE_DENSITY_##dataset
#define PANTO_EXPAND_ACCELEROMETER_NOISE_DENSITY(dataset) PANTO_EXPAND_ACCELEROMETER_NOISE_DENSITY_IMPL(dataset)
#define PANTO_EXPAND_ACCELEROMETER_RANDOM_WALK_IMPL(dataset) DATASET_ACCELEROMETER_RANDOM_WALK_##dataset
#define PANTO_EXPAND_ACCELEROMETER_RANDOM_WALK(dataset) PANTO_EXPAND_ACCELEROMETER_RANDOM_WALK_IMPL(dataset)

constexpr u64 PANTO_IMAGE_WIDTH = PANTO_EXPAND_IMAGE_WIDTH(PANTO_ACTIVE_DATASET);
constexpr u64 PANTO_IMAGE_HEIGHT = PANTO_EXPAND_IMAGE_HEIGHT(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_CAMERA_RATE_HZ = PANTO_EXPAND_CAMERA_RATE_HZ(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_IMU_RATE_HZ = PANTO_EXPAND_IMU_RATE_HZ(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_GYROSCOPE_NOISE_DENSITY =
    PANTO_EXPAND_GYROSCOPE_NOISE_DENSITY(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_GYROSCOPE_RANDOM_WALK =
    PANTO_EXPAND_GYROSCOPE_RANDOM_WALK(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_ACCELEROMETER_NOISE_DENSITY =
    PANTO_EXPAND_ACCELEROMETER_NOISE_DENSITY(PANTO_ACTIVE_DATASET);
constexpr fp64 PANTO_ACCELEROMETER_RANDOM_WALK =
    PANTO_EXPAND_ACCELEROMETER_RANDOM_WALK(PANTO_ACTIVE_DATASET);

constexpr u64 PANTO_GRID_COLUMNS =
    (PANTO_IMAGE_WIDTH + PANTO_CELL_SIZE - 1) / PANTO_CELL_SIZE;
constexpr u64 PANTO_GRID_ROWS =
    (PANTO_IMAGE_HEIGHT + PANTO_CELL_SIZE - 1) / PANTO_CELL_SIZE;
constexpr u64 PANTO_NUM_IMAGE_CELLS = PANTO_GRID_COLUMNS * PANTO_GRID_ROWS;

/*                      
 ******************************************************************                                                  
 ******************************************************************                                                  
 ******************************************************************                                                  
 *
 *               General Macros and typedefs
 *
 ******************************************************************                                                  
 ******************************************************************                                                  
 ******************************************************************                                                  
 */

#define PANTO_SIGNX(x) \
    ((x < 0.0f) ? -1.0f : 1.0f)

typedef std::array<u8, PANTO_DESCRIPTOR_SIZE> typeDescriptor;

#endif // CONFIG_HPP_
