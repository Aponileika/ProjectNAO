#ifndef __VT_VECUTILS_HPP_
#define __VT_VECUTILS_HPP_
#include <vector>
#include <assert.h>
#include "CArenaAlloc.h"

template<typename T>
void VT_EraseUnordered(std::vector<T> vec, u64 idx)
{
    std::size_t last_idx = vec.size() - static_cast<std::size_t>(1);

    assert(idx <= last_idx);
    
    if(idx != last_idx)
    {
        vec[idx] = std::move(vec.back());
    }

    vec.pop_back();
}


#endif //__VT_VECUTILS_HPP_
