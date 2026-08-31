#ifndef CONFIG_HPP_
#define CONFIG_HPP_
#include <random>
#include <string>
#include <cmath>
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

#define OPENCV_AKAZETHRESHOLD 0.001
#define OPENCV_AKAZE_NOCTAVES 4
#define OPENCV_AKAZE_NOCTAVELAYERS 4
#define PANTO_DESCRIPTOR_SIZE 61 //Bytes
#define PANTO_LOCAL_MAP_SAMPLE_STRIDE 2
#define PANTO_NUM_BOOTSTRAP_FRAMES 50
// [Number of frames], from bootstrap learning mean distance * number of frames
// should trigger keyframe insertion.
#define PANTO_KEYFRAME_MEAN_DISTANCE_THRESHOLD_GAIN 20
#define PANTO_KEYFRAME_MEAN_VELOCITY_THRESHOLD_GAIN 4
#define PANTO_KEYFRAME_MEAN_TRACKING_HIGH_THRESHOLD_GAIN 0.8f
#define PANTO_KEYFRAME_MEAN_TRACKING_LOW_THRESHOLD_GAIN 0.2f
#define PANTO_KEYFRAME_FUZZY_MAX_RULE_THRESHOLD 0.95f
#define PANTO_KEYFRAME_FUZZY_SPATIAL_TRACKING_THRESHOLD 0.5f
// Same as slam orb, baseline > 1% of median depth of local map relative to a keyframe
#define PANTO_BASELINE_THRESHOLD 0.01f
#define PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth) ((BaseLine / MedianDepth) > PANTO_BASELINE_THRESHOLD)
#define PANTO_FEATURE_TRACK_NOT_OBSERVED -1
#if defined(DEBUG)
    #define PANTO_INIT_MIN_NUM_FRAMES 2
#else
    #define PANTO_INIT_MIN_NUM_FRAMES 60
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
#define PANTO_TOP_N_KF_FOR_LOCAL_MAP 20
#define PANTO_MAX_LOCAL_TRACKING_MAP_SIZE 80
#define PANTO_PIXEL_CHI_SQUARED_T 5.991
#define PANTO_PIXEL_CHI_SQUARED_T_SQRT 2.448
#define PANTO_IMAGEPOINT_RESERVE 1000

constexpr fp64 PANTO_MINIMUMPARALLAX = 1.0 * M_PI / 180.0;

inline const fp64 PANTO_MAXIMUMCOSPARALLAX = std::cos(PANTO_MINIMUMPARALLAX);


using PantoClock = std::chrono::steady_clock;

#define PANTO_USE_DATASET true
#define PANTO_DATASET_BASE_PATH "./datasets"

#define PANTO_ACTIVE_DATASET TUM_FREIBURG1_XYZ

#define DATASETS \
    X(TUM_FREIBURG1_XYZ, "/tum/rgbd_dataset_freiburg1_xyz") \
    X(TUM_FREIBURG1_RPY, "/tum/rgbd_dataset_freiburg1_rpy") \
    X(TUM_FREIBURG2_XYZ, "/tum/rgbd_dataset_freiburg2_xyz") \
    X(TUM_FREIBURG2_PIONEER_SLAM, "/tum/rgbd_dataset_freiburg2_pioneer_slam")

#define DATASET_SEQUENCES \
    X(TUM_FREIBURG1_XYZ, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG1_RPY, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG2_XYZ, RGB_ORDERED, "rgb_ordered") \
    X(TUM_FREIBURG2_PIONEER_SLAM, RGB_ORDERED, "rgb_ordered")

//fx, fy, s, cx, cy //k1, k2, p1, p2, k3
#define DATASET_INTRINSICS \
    X(TUM_FREIBURG1_XYZ, 517.3, 516.5, 0.0, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633) \
    X(TUM_FREIBURG1_RPY, 517.3, 516.5, 0.0, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633) \
    X(TUM_FREIBURG2_XYZ, 520.9, 521.0, 0.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172) \
    X(TUM_FREIBURG2_PIONEER_SLAM, 520.9, 521.0, 0.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172) \
    X(WEBCAM_JE, 974.7187409387847, 976.5223334221673, 0.0, 666.3249058750432, 337.4737864029501, 0.06475901025911835, -0.1903655376657792, -0.003666863513699757, 0.002119531347424837, 0.1113497353944944)

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

#define PANTO_EXPAND_DATASET_PATH_IMPL(dataset) DATASET_PATH_##dataset
#define PANTO_EXPAND_DATASET_PATH(dataset) PANTO_EXPAND_DATASET_PATH_IMPL(dataset)

#define PANTO_EXPAND_SEQUENCE_PATH_IMPL(dataset) SEQUENCE_PATH_##dataset##_RGB_ORDERED
#define PANTO_EXPAND_SEQUENCE_PATH(dataset) PANTO_EXPAND_SEQUENCE_PATH_IMPL(dataset)

constexpr auto panto_dataset_path =
    PANTO_EXPAND_DATASET_PATH(PANTO_ACTIVE_DATASET);

constexpr auto panto_sequence_path =
    PANTO_EXPAND_SEQUENCE_PATH(PANTO_ACTIVE_DATASET);

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

#define PANTO_EPIPOLARTRESHOLD 2.0f

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

#define PANTO_INIT_RANSAC_LOOP_CNT 500

#define PANTO_FUNDAMENTAL_MIN_POINTS 8
#define PANTO_HOMOGRAPHY_MIN_POINTS 4
#define PANTO_INIT_MIN_STATIONARY_POINTS 200
#define PANTO_MIN_NUMBER_INITIAL_MAP_POINTS 100
#define PANTO_NUM_THREADS_MAX 8

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

#define PANTO_IMAGE_WIDTH 640
#define PANTO_IMAGE_HEIGHT 480

#define PANTO_GRID_COLUMNS 20 // 640 / 32
#define PANTO_GRID_ROWS 15 // 480 / 32
#define PANTO_NUM_IMAGE_CELLS PANTO_GRID_COLUMNS * PANTO_GRID_ROWS

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
