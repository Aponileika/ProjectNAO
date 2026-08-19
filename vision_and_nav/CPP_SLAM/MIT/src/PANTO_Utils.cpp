#include "../include/PANTO_Utils.hpp"
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
