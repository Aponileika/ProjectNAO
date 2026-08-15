#ifndef __CONFIG_HPP_
#define __CONFIG_HPP_
#include <string>
#include "CArenaAlloc.h"

// #define PANTO_DBG

#define CERES_MAX_ITER 200
#define CERES_NUM_THREADS 8
#define CERES_HUBER_THRESHOLD 2.0f

//LOOK INTO USAC
#define OPENCV_RANSACMETHOD cv::USAC_DEFAULT
//#define RANSACMETHOD cv::RANSAC
#define OPENCV_PROBECORRECT 0.99f
#define OPENCV_RANSACEPIXELT 2.0f
#define OPENCV_RANSACMAXITERS 2000
#define PANTO_NEWFRAMECORRPTHRESHOLD 30
#define PANTO_INITFRAMETHRESHOLDCORRP 50
#define OPENCV_AKAZETHRESHOLD 0.0005f
#define OPENCV_AKAZE_NOCTAVES 4
#define OPENCV_AKAZE_NOCTAVELAYERS 4
#define PANTO_DESCRIPTOR_SIZE 61 //Bytes

const char* PANTO_SLAMSTARTMSG =
"        _____                 ^\n"
"    ___/_____\\___             |\n"
"  _/  _     _  \\_        .----O----.      [o_o]\n"
" /__/(_)---(_)\\__\\_______|         |_____/|___|\\\n"
"        \\______/          \\________/      /|   |\\\n"
"\n"
"              P a n t o P i l o t\n";

#define PANTO_USE_DATASET true
#define PANTO_DATASET_BASE_PATH "./datasets"

#define DATASETS \
    X(TUM_FREIBURG1_XYZ, "/tum/rgbd_dataset_freiburg1_xyz") 

#define DATASET_SEQUENCES \
    X(TUM_FREIBURG1_XYZ, RGB_ORDERED, "rgb_ordered")

//fx, fy, s, cx, cy
//k1, k2, p1, p2, k3
#define DATASET_INTRINSICS \
    X(TUM_FREIBURG1_XYZ, \
            (517.3f), (516.5f), (0.0f), (318.6f), (255.3f), \
            (0.2624f), (-0.9531f), (-0.0054f), (0.0026f), (1.1633f)) \
    X(WEBCAM_JE, \
            (9.747187409387847*100.0f), (9.765223334221673*100.0f), (0.0f), \
            (6.663249058750432*100.0f), (3.374737864029501*100.0f), \
            (6.475901025911835*0.01f), (-1.903655376657792*0.1f), (-3.666863513699757*0.001f), \
            (2.119531347424837*0.001f), (1.113497353924944*0.1f))

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

constexpr auto panto_dataset_path = DATASET_PATH_TUM_FREIBURG1_XYZ;
constexpr auto panto_sequence_path = SEQUENCE_PATH_TUM_FREIBURG1_XYZ_RGB_ORDERED;
const Dataset panto_dataset = Dataset::TUM_FREIBURG1_XYZ;

#define PANTO_LOGPATH "/Users/Jonathan/Programmering/FIA/ProjectNAO/vision_and_nav/CPP_SLAM/logs/log.txt"

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

#define PANTO_MAPPOINT_MATCH_SEARCH_AREA 20.0f

//arbitrary, now same as slam orb
#define PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD 200
#define PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW 100

#define PANTO_ID_NOT_SET U64_MAX

#define PANTO_TIMESTAMP_NOT_SET -1.0f

const std::string PANTO_VocabFilePath("PantoVocabulary.dbow3");

// Controls how many children in vocab tree
#define PANTO_DBOW_BRANCHING_FACTOR 10

// Controls how deep vocab tree 
#define PANTO_DBOW_DEPTH 5

#define PANTO_DBOW_LEVELSUP (PANTO_DBOW_DEPTH - 1);


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

#endif // __CONFIG_HPP_
