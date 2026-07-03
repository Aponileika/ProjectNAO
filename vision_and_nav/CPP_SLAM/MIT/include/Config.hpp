#ifndef __CONFIG_HPP_
#define __CONFIG_HPP_
#include <string>
#include "CArenaAlloc.h"

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

#define PANTO_SLAMSTARTMSG "\
    --------------------------------\n\
    --------------------------------\n\
    --------------------------------\n\
            SLAM IS STARTING\n\
    --------------------------------\n\
    --------------------------------\n\
    --------------------------------\n\
"

#define PANTO_USE_DATASET true
#define PANTO_DATASET_BASE_PATH "./datasets"

#define DATASETS \
    X(TUM_FREIBURG1_XYZ, "/tum/rgbd_dataset_freiburg1_xyz") 

#define DATASET_SEQUENCES \
    X(TUM_FREIBURG1_XYZ, RGB_ORDERED, "rgb_ordered")

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

// TODO add config for camera intrinsics

#endif // __CONFIG_HPP_
