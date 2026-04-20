#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

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

int main(void)
{
    setbuf(stdout, NULL);
    printf("Initiating SLAM\n");
    SL_InitSlam();
    printf("Setting intrinsics\n");
    CM_SetIntrinsics("");
    printf("Starting SLAMLoop\n");
    SL_SlamLoop();
    return 0;
}
