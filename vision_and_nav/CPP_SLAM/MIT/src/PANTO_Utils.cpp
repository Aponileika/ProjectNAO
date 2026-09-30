#include "../include/PANTO_Utils.hpp"
#include "Config.hpp"
#include <cstring>
#include <bit>

u32 PANTO_HammingDistance(const typeDescriptor& a, const typeDescriptor& b)
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

// Matches a left patch with a right patch with a rectified stereo pair, searching 5 patches
// along a line in the right image.
// This could be made more efficient for the right image patches, currently
// the mean sums are recalculated, even though there is no stride between right image patches partial sums are recalculated
// a better implementation would save a cusum for each element considered in the right frame and use those to calculate
// the means faster. Same goes for the Sum of absolute differences, use a rolling window!
fp64 PANTO_ZeroMeanSAD(cv::Mat GrayFrameLeft, cv::Mat GrayFrameRight, i64 RowCenterLeft, i64 ColCenterLeft, 
        i64 RowStartRight, i64 ColStartRight)
{
    // Calculate Left patch
    std::array<i64, PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT> LeftPatch;
    i64 TopLeftRowLeft = RowCenterLeft - static_cast<i64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_HALF_SIDE_LENGTH);
    i64 TopLeftColLeft = RowCenterLeft - static_cast<i64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_HALF_SIDE_LENGTH);
    if(TopLeftRowLeft < 0)
    {
        TopLeftRowLeft = 0;
    } 
    if(TopLeftColLeft < 0)
    {
        TopLeftColLeft = 0;
    } 

    fp64 Sum = 0;
    for(i64 i = TopLeftRowLeft; i < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && i < static_cast<i64>(PANTO_IMAGE_HEIGHT); i++)
    {
        for(i64 j = TopLeftColLeft; j < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && j < static_cast<i64>(PANTO_IMAGE_WIDTH); j++)
        {
            Sum += GrayFrameLeft.at<u8>(i, j);
        }
    }

    const fp64 MeanLeftPatch = Sum / static_cast<fp64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT);

    for(i64 i = TopLeftRowLeft; i < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && i < static_cast<i64>(PANTO_IMAGE_HEIGHT); i++)
    {
        for(i64 j = TopLeftColLeft; j < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && j < static_cast<i64>(PANTO_IMAGE_WIDTH); j++)
        {
            LeftPatch[i * PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH + j] = GrayFrameLeft.at<u8>(i, j) / MeanLeftPatch;
        }
    }

    i64 i = ColStartRight - PANTO_SAD_REFINMENT_SEARCH_LENGTH;
    if(i < 0)
    {
        i = 0;
    }
    i64 ColEnd = ColStartRight + PANTO_SAD_REFINMENT_SEARCH_LENGTH;
    if(ColEnd >= static_cast<i64>(PANTO_IMAGE_HEIGHT))
    {
        ColEnd = static_cast<i64>(PANTO_IMAGE_HEIGHT - 1);
    }

    i64 TopLeftRowRight = RowStartRight - static_cast<i64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_HALF_SIDE_LENGTH);
    if(TopLeftRowRight < 0)
    {
        TopLeftRowRight = 0;
    }

    for(; i < ColEnd; i++)
    {
        std::array<i64, PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT> RightPatch;

        i64 TopLeftColRight = ColStartRight - static_cast<i64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_HALF_SIDE_LENGTH);

        if(TopLeftColRight < 0)
        {
            TopLeftColRight = 0;
        } 

        fp64 Sum = 0;

        for(i64 j = TopLeftRowRight; i < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && j < static_cast<i64>(PANTO_IMAGE_HEIGHT); j++)
        {
            for(i64 k = TopLeftColRight; k < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && k < static_cast<i64>(PANTO_IMAGE_WIDTH); k++)
            {
                Sum += GrayFrameLeft.at<u8>(j, k);
            }
        }

        const fp64 MeanRightPatch = Sum / static_cast<fp64>(PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT);

        for(i64 j = TopLeftRowRight; i < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && j < static_cast<i64>(PANTO_IMAGE_HEIGHT); j++)
        {
            for(i64 k = TopLeftColRight; k < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH && k < static_cast<i64>(PANTO_IMAGE_WIDTH); k++)
            {
                RightPatch[j * PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT_SIDE_LENGTH + k] = GrayFrameLeft.at<u8>(j, k) / MeanRightPatch;
            }
        }
        for(i64 j = 0; j < PANTO_PATCH_SIZE_STEREO_SAD_REFINMENT; j++)
        {
        }
    }
}
