#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "CArenaAlloc.h"
#include "VW_Views.hpp"
#include "CM_Camera.hpp"
#include "MX_Matrix.hpp"
#include "PT_Points.hpp"
#include "OB_Observations.hpp"
#include "SL_SLAM.hpp"
#include "CArenaAlloc.c"
#include "VW_Views.cpp"
#include "CM_Camera.cpp"
#include "MX_Matrix.cpp"
#include "PT_Points.cpp"
#include "OB_Observations.cpp"
#include "SL_SLAM.cpp"

int main(void)
{
    SL_InitSlam();
    CM_SetIntrinsics(NULL);
    return 0;
}
