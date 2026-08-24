#ifndef __PROJ_PROJECTIVEUTILS_HPP_
#define __PROJ_PROJECTIVEUTILS_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "CM_Camera.hpp"

//I love function overloading
//cv::Mat only for 4->3, i hate general cv::Mat!
Eigen::Vector3d PROJ_Homog2Cart(cv::Mat vec);
Eigen::Vector3d PROJ_Homog2Cart(Eigen::Vector4d vec);
Eigen::Vector2d PROJ_Homog2Cart(Eigen::Vector3d vec);
Eigen::Vector4d PROJ_CV2NormalizedEigen(cv::Mat vec);
void PROJ_NormalizeToSpherical(Eigen::Vector4d& Vec);
Eigen::Vector4d PROJ_NormalizeToSpherical(const Eigen::Vector4d& Point);
Eigen::Matrix3d PROJ_CrossProductMatrix(Eigen::Vector3d vec);
cv::Mat PROJ_ToHomogFromCart(cv::Point2d point);
Eigen::Vector4d PROJ_TriangulateDLT(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2, const Eigen::Matrix<fp64, 3, 4>& P1,
        const Eigen::Matrix<fp64, 3, 4>& P2);
std::vector<Eigen::Vector4d> PROJ_TriangulateLOST(const std::vector<std::vector<Eigen::Vector3d>>& pixelCoords,
        const std::vector<std::vector<Eigen::Matrix4d>>& T, const Eigen::Matrix3d K);
bool PROJ_Project(const Eigen::Vector4d& MapPoint, Eigen::Vector2d& ImagePoint, const typeCamera& Pose);

inline Eigen::Vector3d PROJ_Cart2Homog(Eigen::Vector2d Vec)
{
    return Eigen::Vector3d(Vec.x(), Vec.y(), 1.0f);
}

#endif //__PROJ_PROJECTIVEUTILS_HPP_
