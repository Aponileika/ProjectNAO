#include "PROJ_ProjectiveUtils.hpp"

Eigen::Vector3d PROJ_Homog2Cart(cv::Mat vec)
{
    assert(vec.rows == 4 && vec.cols == 1);
    fp64 w = vec.at<fp64>(3, 0);
    return Eigen::Vector3d(
            vec.at<fp64>(0, 0) / w,
            vec.at<fp64>(1, 0) / w,
            vec.at<fp64>(2, 0) / w);
}

Eigen::Vector3d PROJ_Homog2Cart(Eigen::Vector4d vec)
{
    fp64 w = vec(3);
    return Eigen::Vector3d(
            vec(0) / w,
            vec(1) / w,
            vec(2) / w);
}

Eigen::Vector2d PROJ_Homog2Cart(Eigen::Vector3d vec)
{
    fp64 w = vec(2);
    return Eigen::Vector2d(
            vec(0) / w,
            vec(1) / w);
}

Eigen::Vector4d PROJ_CV2NormalizedEigen(cv::Mat vec)
{
    assert(vec.rows == 4 && vec.cols == 1);
    Eigen::Vector4d ret(
            vec.at<fp64>(0, 0),
            vec.at<fp64>(1, 0),
            vec.at<fp64>(2, 0),
            vec.at<fp64>(3, 0));
    fp64 norm = ret.norm();
    return ret / norm;
}

cv::Mat PROJ_ToHomogFromCart(cv::Point2d point)
{
    return (cv::Mat_<fp64>(3, 1) << 
            point.x,
            point.y,
            1.0f);
}
