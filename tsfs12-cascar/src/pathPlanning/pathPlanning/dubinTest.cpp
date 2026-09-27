#include "dubinPathCpp.hpp"
#include "hybridAStar_cpp.hpp"

#include <iostream>
#include <vector>
#include <array>
#include <random>
#include <cstdio>
#include <fstream>
#include <random>
#include <chrono>

using Grid = std::vector<std::vector<double>>;

struct RectangleObstacle {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};

struct PathPoint {
    double x;
    double y;
    double theta;
    double distance = 0.0;
};

void addRectangleObstacle(
    Grid& grid, const RectangleObstacle& obstacle, std::array<double, 2> const& xlim,
    std::array<double, 2> const& ylim, double resolution) {

    int x0 = static_cast<int>((obstacle.x_min - xlim[0]) / resolution);
    int x1 = static_cast<int>((obstacle.x_max - xlim[0]) / resolution);
    int y0 = static_cast<int>((obstacle.y_min - ylim[0]) / resolution);
    int y1 = static_cast<int>((obstacle.y_max - ylim[0]) / resolution);

    x0 = std::max(0, x0);
    x1 = std::min(static_cast<int>(grid[0].size()), x1);

    y0 = std::max(0, y0);
    y1 = std::min(static_cast<int>(grid.size()), y1);

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
        grid[y][x] = 1.0;
        }
    }
}

void savePath(const std::vector<std::array<double, 4>>& path,
              const std::array<double,3>& start,
              const std::array<double,3>& goal)
{
    std::ofstream file("result.json");

    file << "{\n";
    file << "  \"start\": [" << start[0] << "," << start[1] << "," << start[2] << "],\n";
    file << "  \"goal\": [" << goal[0] << "," << goal[1] << "," << goal[2] << "],\n";
    file << "  \"path\": [\n";

    for (size_t i = 0; i < path.size(); ++i) {
        file << "    [" << path[i][0] << "," << path[i][1] << "," << path[i][2] << "]";
        if (i + 1 < path.size())
            file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";
}


int main(){
    std::array<double, 2> xlim{-5.0, 5.0};
    std::array<double, 2> ylim{-5.0, 5.0};
    
    double xy_resolution = 0.05;
    int size = static_cast<int>((xlim[1] - xlim[0]) / xy_resolution);

    Grid map(size, std::vector<double>(size, 0.0));
    std::vector<RectangleObstacle> obstacles = {
        {-0.5, 0.5, -3.0, 3.0},
        {-3.0, -2.0, -3.0, -2.0},
        { 2.0, 3.0, 2.0, 3.0},
        {-3.0, -2.0, 2.0, 3.0},
        { 2.0, 3.0, -3.0, -2.0}
        };
    
    for (const auto& obstacle : obstacles){
        addRectangleObstacle(map, obstacle, xlim, ylim, xy_resolution);
    }

    HybridAStar planner {};

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    std::array<double, 3> start {dist(gen)*3-4, dist(gen)*8-4, dist(gen)*2*3.1415};
    std::array<double, 3> goal  {dist(gen)*3+1, dist(gen)*8-4, dist(gen)*2*3.1415};

    start = {-3, 0, 0};
    goal = {3, 0, -3.141592};

    auto t0 {std::chrono::high_resolution_clock::now()};
    auto result = planner.search(start, goal, map, xlim, ylim);
    auto t1 {std::chrono::high_resolution_clock::now()};
    auto duration {std::chrono::duration_cast<std::chrono::microseconds>(t1-t0)};
    std::cout << "Runtime: " << duration.count()/1000.0 << "ms" << std::endl;

    savePath(result, start, goal);

    return 0;
}