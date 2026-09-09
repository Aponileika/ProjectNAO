#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

/* Intrinsic camera matrices */
const cv::Matx33d KLeft(
    583.656905612597, 0.0,              318.246701709208,
    0.0,              584.913010873283, 219.459363187465,
    0.0,              0.0,                1.0
);

const cv::Matx33d KRight(
    585.964174869895, 0.0,              316.930217707653,
    0.0,              586.841937275282, 217.008369998399,
    0.0,              0.0,                1.0
);


/* Distortion coefficients: k1, k2, p1, p2, k3 */
const cv::Vec<double, 5> DLeft(
    -0.070231934465,
     0.581467630762,
    -0.011830893759,
    -0.001925557061,
    -1.100253544397
);

const cv::Vec<double, 5> DRight(
    -0.06616119276960,
     0.4795788187738,
    -0.01156431233099,
    -0.0007034267423764,
    -0.8721983929016
);


/* Transformation from the left camera coordinate system to the right */
const cv::Matx33d RLeftToRight(
     0.999847700117, -0.003529743310,  0.017091444725,
     0.003687614197,  0.999950749274, -0.009214148264,
    -0.017058079381,  0.009275771604,  0.999811473223
);

const cv::Vec3d TLeftToRight(
     0.06917822119529,
     0.00006405125386527,
    -0.003335395230894
);


/* Rectification rotations */
const cv::Matx33d RLeftRectification(
     0.999512054949, -0.003047586781, -0.031086399372,
     0.002905116148,  0.999985074594, -0.004627190226,
     0.031100037159,  0.004534622810,  0.999505990420
);

const cv::Matx33d RRightRectification(
     0.9988392751028,  0.0009248128518370, -0.04815856344735,
    -0.0007041022937771, 0.9999891731673,  0.004599759581089,
     0.04816229595941, -0.004560511970603,  0.9988291119999
);


/* Rectified projection matrices */
const cv::Matx34d PLeft(
    614.558600234117, 0.0,              346.102619171143, 0.0,
    0.0,              614.558600234117, 206.618324279785, 0.0,
    0.0,                0.0,              1.0,            0.0
);

const cv::Matx34d PRight(
    614.558600234117, 0.0,              346.102619171143, 42.563475269922,
    0.0,              614.558600234117, 206.618324279785,  0.0,
    0.0,                0.0,              1.0,             0.0
);


/* Disparity-to-3D reprojection matrix */
const cv::Matx44d Q(
    1.0, 0.0,   0.0,             -346.102619171143,
    0.0, 1.0,   0.0,             -206.618324279785,
    0.0, 0.0,   0.0,              614.558600234117,
    0.0, 0.0, -14.438637736623,      0.0
);

int main(void)
{

    const cv::Size ImageSize(
    FRAME_WIDTH,
    FRAME_HEIGHT);

    cv::Mat LeftMap1;
    cv::Mat LeftMap2;
    cv::Mat RightMap1;
    cv::Mat RightMap2;

    cv::initUndistortRectifyMap(
            KLeft, DLeft,
            RLeftRectification, PLeft,
            ImageSize, CV_16SC2,
            LeftMap1, LeftMap2);

    cv::initUndistortRectifyMap(
            KRight, DRight,
            RRightRectification, PRight,
            ImageSize, CV_16SC2,
            RightMap1, RightMap2);

    while(true)
    {
        cv::Mat LeftRectified;
        cv::Mat RightRectified;

        cv::remap(LeftImage,
                LeftRectified,
                LeftMap1,
                LeftMap2,
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);

        cv::remap(RightImage,
                RightRectified,
                RightMap1,
                RightMap2,
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
    }
}
