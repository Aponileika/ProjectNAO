#include "../include/PANTO_Utils.hpp"
#include <cstring>
#include <bit>

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

u32 PANTO_HammingDistance(typeDescriptor& a, typeDescriptor& b)
{
    u32 HammingDistance = 0;

    constexpr std::size_t N64 = PANTO_DESCRIPTOR_SIZE / sizeof(u64);

    for(std::size_t i{}; i < N64; i++)
    {
        u64 _a, _b;

        std::memcpy(&_a, a.data() + i * sizeof(u64), sizeof(u64));
        std::memcpy(&_b, b.data() + i * sizeof(u64), sizeof(u64));

        HammingDistance += std::popcount(_a ^ _b);
    }

    for(std::size_t i = N64 * sizeof(u64); i < PANTO_DESCRIPTOR_SIZE; i++)
    {
        HammingDistance += std::popcount(static_cast<unsigned>(a[i] ^ b[i]));
    }

    return HammingDistance;
}
