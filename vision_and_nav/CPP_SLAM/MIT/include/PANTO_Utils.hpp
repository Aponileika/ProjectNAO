#ifndef __VT_VECUTILS_HPP_
#define __VT_VECUTILS_HPP_
#include <vector>
#include <assert.h>
#include <Eigen/core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"

template<typename T>
void PANTO_EraseUnordered(std::vector<T> vec, u64 idx)
{
    std::size_t last_idx = vec.size() - static_cast<std::size_t>(1);

    assert(idx <= last_idx);
    
    if(idx != last_idx)
    {
        vec[idx] = std::move(vec.back());
    }

    vec.pop_back();
}

template<typename T>
T PANTO_Cv2Eigen(cv::Mat cvmat)
{
    T eigenmat;
    cv2eigen(cvmat, eigenmat);
    return eigenmat;
}

template<typename T>
T PANTO_Cv2Eigen(cv::Matx33d cvmat)
{
    T eigenmat;
    cv2eigen(cvmat, eigenmat);
    return eigenmat;
}

typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointPair2D;
std::vector<std::vector<Eigen::Vector3d>> PANTO_PointPair2Eigen(PointPair2D pp);

#endif //__VT_VECUTILS_HPP_
