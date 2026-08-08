#include "../include/PANTO_Utils.hpp"

std::vector<std::vector<Eigen::Vector3d>> PANTO_PointPair2Eigen(PointPair2D pp)
{
    const u64 n = pp.first.size();
    std::vector<std::vector<Eigen::Vector3d>> ret(n);
    for(u64 i = 0; i < n; i++)
    {
        ret[i].resize(2);
        //TODO make homog and take from pointpar which is cv::vector2d
        ret[i][0] << pp.first[i].x, pp.first[i].y, 1.0f;
        ret[i][1] << pp.second[i].x, pp.second[i].y, 1.0f;
    }
    return ret;
}
