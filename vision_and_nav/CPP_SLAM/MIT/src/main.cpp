#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "../include/CArenaAlloc.h"
#include "../include/CM_Camera.hpp"
#include "../include/EP_CorrespondingPoints.hpp"
#include "../include/FR_Frames.hpp"
#include "../include/MX_Matrix.hpp"
#include "../include/OB_Observations.hpp"
#include "../include/PT_Points.hpp"
#include "../include/SL_SLAM.hpp"
#include "../include/VT_VecUtils.hpp"
#include "../include/VW_Views.hpp"
#include "../include/LG_Logging.hpp"
#include "../include/OP_BA.hpp"
#include "../include/VIZ_Visualization.hpp"
#include "../include/FEAT_Features.hpp"
#include "../include/PROJ_ProjectiveUtils.hpp"
#include "../../third-party/ORBSLAM/include/ORBextractor.h"

#include "CArenaAlloc.c"
#include "CM_Camera.cpp"
#include "EP_CorrespondingPoints.cpp"
#include "FR_Frames.cpp"
#include "MX_Matrix.cpp"
#include "OB_Observations.cpp"
#include "PT_Points.cpp"
#include "SL_SLAM.cpp"
#include "VT_VecUtils.cpp"
#include "VW_Views.cpp"
#include "LG_Logging.cpp"
#include "OP_BA.cpp"
#include "VIZ_Visualization.cpp"
#include "FEAT_Features.cpp"
#include "PROJ_ProjectiveUtils.cpp"
#include "../../third-party/ORBSLAM/src/ORBextractor.cc"

int main(int argc, char* argv[])
{
    i32 num_loops = 10000;
    if(argc >= 2)
    {
        num_loops = std::stoi(argv[1]);
    }
    setbuf(stdout, NULL);
    LG_InitLogger();
    LG_Log("Initiating SLAM\n");
    SL_InitSlam();
    LG_Log("Setting intrinsics\n");
    LG_Log("[main] OpenCV is using %lld threads \n", cv::getNumThreads());
    CM_SetIntrinsics("");
    LG_Log("Starting SLAMLoop\n");
    SL_SlamLoop(num_loops);
    return 0;
}
