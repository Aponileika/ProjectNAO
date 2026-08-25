#ifndef PANTOVEC_PANTOVECTOR_HPP_
#define PANTOVEC_PANTOVECTOR_HPP_
#include <vector>
#include <queue>
#include <assert.h>
#include "CArenaAlloc.h"
#include "Config.hpp"

template <typename T>
class typePantoVector
{
    public:

    std::vector<T> Data;

    typePantoVector(std::size_t Size, T Value)
    {
        Data(Size, Value);
        FreeIDs = {};
    }

    void push_back(T NewElement)
    {
        if(FreeIDs.front() == PANTO_ID_NOT_SET)
        {
            Data.push_back();
        }
        else
        {
            Data[FreeIDs.front()] = NewElement;
            FreeIDs.pop();
        }
    }

    void remove(u64 Index)
    {
        FreeIDs.push(Index);
    }

    void resize(std::size_t Size)
    {
        Data.resize(Size);
    }

    void clear()
    {
        Data.clear();
        std::queue<u64> Empty{};
        std::swap(FreeIDs, Empty);
    }

    inline T& operator[](u64 Index)
    {
        assert(Index != PANTO_ID_NOT_SET);
        assert(Index < Data.size());
        return Data[Index];
    }

    private:

    std::queue<u64> FreeIDs;
};

#endif //  PANTOVEC_PANTOVECTOR_HPP_
