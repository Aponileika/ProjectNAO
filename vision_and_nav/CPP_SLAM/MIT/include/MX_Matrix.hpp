#ifndef __MX__MATRIX_HPP_
#define __MX__MATRIX_HPP_
#include "CArenaAlloc.h"
#include <vector>

struct Matrix
{
    std::vector<fp64> data;
    u32 rows, cols;
};

#endif //__MX__MATRIX_HPP_
