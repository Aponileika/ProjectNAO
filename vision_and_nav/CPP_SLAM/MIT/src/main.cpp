#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "../include/LG_Logging.hpp"
#include "../include/CArenaAlloc.h"
#include "../include/EP_CorrespondingPoints.hpp"
#include "../include/KEY_KeyFrame.hpp"
#include "../include/PT_PantoImagePoint.hpp"
#include "../include/PT_PantoMapPoints.hpp"
#include "../include/MAP_Mapping.hpp"
#include "../include/FR_Frames.hpp"
#include "../include/SL_SLAM.hpp"
#include "../include/PANTO_Utils.hpp"
#include "../include/CM_Camera.hpp"
#include "../include/OP_BA.hpp"
#include "../include/VIZ_Visualization.hpp"
#include "../include/PROJ_ProjectiveUtils.hpp"
#include "../include/PANTO_Utils.hpp"
#include "../include/DBOW3_DeepBagofWords.hpp"
#include "../include/Config.hpp"
#include "../include/INIT_InitializeSLAM.hpp"
#include "../include/PANTOVEC_PantoVector.hpp"
#if defined(CONFIG_IMU)
#include "../include/IMU_IMUReader.hpp"
#include "../include/IMU_PreIntegration.hpp"
#include "../include/GT_ReadGroundTruth.hpp"
#endif 

#include "LG_Logging.cpp"
#include "CArenaAlloc.c"
#include "EP_CorrespondingPoints.cpp"
#include "FR_Frames.cpp"
#include "SL_SLAM.cpp"
#include "CM_Camera.cpp"
#include "OP_BA.cpp"
#include "PROJ_ProjectiveUtils.cpp"
#include "PANTO_Utils.cpp"
#include "KEY_KeyFrame.cpp"
#include "PT_PantoImagePoints.cpp"
#include "PT_PantoMapPoints.cpp"
#include "MAP_Mapping.cpp"
#include "DBOW3_DeepBagofWords.cpp"
#include "INIT_InitializeSLAM.cpp"
#include "GRAPH_PantoGraph.cpp"
#include "VIZ_Visualization.cpp"
#if defined(CONFIG_IMU)
#include "IMU_IMUReader.cpp"
#include "IMU_PreIntegration.cpp"
#include "GT_ReadGroundTruth.cpp"
#endif

int main(int argc, char* argv[])
{
    setbuf(stdout, NULL);
    printf("Entered main\n");
    i32 num_loops = 100000;
    if(argc >= 2)
    {
        num_loops = std::stoi(argv[1]);
    }
    LG_InitLogger();
    LG_Log(LogSeverity::DBG, "Initiating SLAM\n");
    SL_InitSlam();
    LG_Log(LogSeverity::DBG, "Setting intrinsics\n");
    LG_Log(LogSeverity::DBG, "[main] OpenCV is using %lld threads \n", cv::getNumThreads());
    CM_SetIntrinsics();
    LG_Log(LogSeverity::DBG, "Initializing visualization\n");
#if !defined(DEBUG)
    VIZ_InitVisualization();
#endif
    LG_Log(LogSeverity::DBG, "Starting SLAMLoop\n");
    SL_PantoSLAM(num_loops);
    return 0;
}
