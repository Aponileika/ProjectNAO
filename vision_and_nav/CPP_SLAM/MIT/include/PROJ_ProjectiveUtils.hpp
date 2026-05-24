#ifndef __PROJ_PROJECTIVEUTILS_HPP_
#define __PROJ_PROJECTIVEUTILS_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"

//I love function overloading
//cv::Mat only for 4->3, i hate general cv::Mat!
Eigen::Vector3d PROJ_Homog2Cart(cv::Mat vec);
Eigen::Vector3d PROJ_Homog2Cart(Eigen::Vector4d vec);
Eigen::Vector2d PROJ_Homog2Cart(Eigen::Vector3d vec);
Eigen::Vector4d PROJ_CV2NormalizedEigen(cv::Mat vec);
cv::Mat PROJ_ToHomogFromCart(cv::Point2d point);

#endif //__PROJ_PROJECTIVEUTILS_HPP_
