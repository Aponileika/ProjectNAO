#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <utility>

using pathObj = std::vector<std::array<double, 4>>;
using pathsObj = std::vector<pathObj>;
using Vec2 = std::array<double,2>;

std::pair<pathsObj, std::vector<double>> dubinsPath(
    std::array<double, 3> const start, std::array<double, 3> const goal, bool const getDistance);