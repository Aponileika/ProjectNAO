#include "hybridAStar_cpp.hpp"
#include "dubinPathCpp.hpp"

#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <map>

constexpr int THETA_BINS {12}; //IF THE THETA RESOLUTION IS EVER CHANGED; CHANGE THIS

double const pi {3.14159265359};
using Pose = std::array<double, 3>;
using PathItem = std::array<double, 4>;

struct Node {
    double x;
    double y;
    double theta;
    double steering;

    std::shared_ptr<Node> parent;
    double g;
    double h;
};

// NOTE: aprroximatly 10ms can be saved by switching to storing nodes in preallocated vector.
//       This has not been done for the sake of readability.


HybridAStar::HybridAStar() {
    Vmax = 1;
    deltaMax = pi/6;
    N = 5;

    steeringAngles.resize(N);
    for (int i {0}; i < N; ++i) {
        steeringAngles[i] = -deltaMax + i * 2 * deltaMax / (N - 1);
    }

    width = 0;
    height = 0;

    xy_resolution = 0.05;
    theta_resolution = 30 * pi / 180;
    map_resolution = 0.05;
    xy_tolerance = 0.05;
    theta_tolerance = 10 * pi / 180;

    L = 0.285;
    track = 0.15;
    radius = 1.0 * std::max(L, track) / 2;

    propogationDistance = 0.2;
    propogationInterval = 0.04;
    dThetas.resize(N);
    for (int i {0}; i < N; ++i) {
        dThetas[i] = std::tan(steeringAngles[i]) / L;
    }
}

std::vector<uint8_t> HybridAStar::inflateMap(const std::vector<uint8_t>& grid) {
    int cells {static_cast<int>(std::ceil(radius / map_resolution)) + 1};

    std::vector<uint8_t> result(width * height, 0);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (!grid[y * width + x]) continue;
            for (int dy = -cells; dy <= cells; ++dy) {
                for (int dx = -cells; dx <= cells; ++dx) {
                    if (dx * dx + dy * dy > cells * cells) continue;

                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        result[ny * width + nx] = 1;
                    }
                }
            }
        }
    }
    return result;
}

double HybridAStar::wrap_angle(double theta) {
    return std::fmod(std::fmod(theta + pi, 2.0 * pi) + 2.0 * pi,
                     2.0 * pi) - pi;
};

std::array<int, 2> HybridAStar::worldToGrid(double const x, double const y) {
    int gx {static_cast<int>(std::floor(x / xy_resolution))};
    int gy {static_cast<int>(std::floor(y / xy_resolution))};
    return {gx, gy};
};

int HybridAStar::stateIndex(double const x, double const y, double const theta, double width) {
    std::array<int, 2> gKey {worldToGrid(x, y)};
    int gx {gKey[0]};
    int gy {gKey[1]};
    int thetaIndex {static_cast<int>(std::round(wrap_angle(theta) / theta_resolution))};
    thetaIndex %= THETA_BINS;
    if (thetaIndex < 0) {thetaIndex += THETA_BINS;}

    return thetaIndex + THETA_BINS * (gy * width + gx);
}

bool HybridAStar::isColliding(std::vector<Pose> const& trajectory) {
    for (const auto& point : trajectory) {
        auto [gx, gy] = worldToGrid(point[0], point[1]);
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            return true;
        }
        if (mapGrid[gy * width + gx]) {
            return true;
        }
    }
    return false;
};

bool HybridAStar::isColliding(std::vector<PathItem> const& path) {
    for (const auto& point : path) {
        auto [gx, gy] = worldToGrid(point[0], point[1]);
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            return true;
        }
        if (mapGrid[gy * width + gx]) {
            return true;
        }
    }
    return false;
};

bool HybridAStar::isColliding(Pose const& pose) {
    auto [gx, gy] = worldToGrid(pose[0], pose[1]);
    if (gx < 0 || gx >= width || gy < 0 || gy >= height) return true;
    if (mapGrid[gy * width + gx]) return true;

    return false;
};

double HybridAStar::heuristic(Node const& start, Node const& goal) {
    double dx {start.x - goal.x};
    double dy {start.y - goal.y};

    return dx*dx + dy*dy;
};

std::vector<PathItem> HybridAStar::reconstructPath(std::shared_ptr<Node> node) {
    std::vector<PathItem> path;

    while (node) {
        path.push_back({node->x, node->y, node->theta, 0});
        node = node -> parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
};

Pose HybridAStar::propogate(Node const& node, double const dTheta) {
    double x {node.x};
    double y {node.y};
    double theta {node.theta};

    std::array<int, 2> gridIndex {};
    int gx {};
    int gy {};

    int dir {1};   //-1 = Backwards search

    Pose result {{x, y, theta}};
    double travelled {0};
    while(std::abs(travelled) < propogationDistance) {
        double ds {dir * std::min(propogationDistance - travelled, dir * propogationInterval)};
        x += ds * std::cos(theta);
        y += ds * std::sin(theta);

        gridIndex = worldToGrid(x, y);
        gx = gridIndex[0];
        gy = gridIndex[1];
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            return {-1, -1, -1};
        }
        if (mapGrid[gy * width + gx]) {
            return {-1, -1, -1};
        }
        theta += ds * dTheta;

        result = {x, y, theta};
        travelled += ds;
    }
    result = {result[0], result[1], wrap_angle(result[2])};
    return result;
};

bool HybridAStar::isFinished(Node const& node, Node const& goal) {
    double ex {std::abs(node.x - goal.x)};
    double ey {std::abs(node.y - goal.y)};
    double etheta {std::abs(wrap_angle(node.theta - goal.theta))};

    if (ex < xy_tolerance && ey < xy_resolution && etheta < theta_tolerance) {
        return true;
    }
    return false;
};

std::vector<PathItem> HybridAStar::getShortestDubin(
    std::vector<std::vector<PathItem>> const& paths,
    std::vector<double> const& pathLengths, double const lowerLimit) {

    double shortestLength {lowerLimit * propogationDistance};
    std::vector<PathItem> shortestPath {};
    for (std::size_t i {0}; i < paths.size(); ++i) {
        if (pathLengths[i] < shortestLength && !isColliding(paths[i])) {
            shortestLength = pathLengths[i];
            shortestPath = paths[i];
        }
    }
    return shortestPath;
}

std::vector<PathItem> HybridAStar::addDubinPaths(std::vector<PathItem> & path) {
    size_t lower {0};
    size_t upper {path.size()-1};

    size_t maxIter {path.size()};
    int iterations {0};

    while (lower + 1 < upper && iterations < path.size()) {
        ++iterations;
        if (iterations >= maxIter) {
            std::cout << "WARNING ; MAX ITERATIONS REACHED" << std::endl;
            std::cout << "Lower: " << lower << " Upper: " << upper << std::endl;
        }
        // Calculate upper path
        std::vector<PathItem> upperDubin {};
        PathItem staticPoint {path[upper]};
        int upperIndex {};
        for (int i {static_cast<int>(lower)}; i<upper-1; ++i) {
            PathItem dynamicPoint {path[i]};
            std::pair result {dubinsPath({dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, 
                                         {staticPoint[0], staticPoint[1], staticPoint[2]}, false)};
            std::vector<std::vector<PathItem>> dubins {result.first};
            std::vector<double> dubinLengths {result.second};

            std::vector<PathItem> shortestPath {getShortestDubin(dubins, dubinLengths, upper - i)};
            if (shortestPath.size() != 0) {
                upperDubin = shortestPath;
                upperIndex = i;
                break;
            }
        }

        // Calculate lower path
        std::vector<PathItem> lowerDubin {};
        staticPoint = path[lower];
        int lowerIndex {};
        for (int i {static_cast<int>(upper)}; i>lower+1; --i) {
            PathItem dynamicPoint {path[i]};
            std::pair result {dubinsPath({staticPoint[0], staticPoint[1], staticPoint[2]}, 
                                         {dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, false)};
            std::vector<std::vector<PathItem>> dubins {result.first};
            std::vector<double> dubinLengths {result.second};

            std::vector<PathItem> shortestPath {getShortestDubin(dubins, dubinLengths, i-lower)};
            if (shortestPath.size() != 0) {
                lowerDubin = shortestPath;
                lowerIndex = i;
                break;
            }
        }
        if (lowerDubin.size() == 0) ++lower;
        
        std::vector<PathItem> newPath {};
        if (upperDubin.size() != 0 && lowerDubin.size() != 0) {
            if ((upper - upperIndex) - static_cast<int>(upperDubin.size()) >= (lowerIndex - lower) - static_cast<int>(lowerDubin.size())) {
                newPath.insert(newPath.end(), path.begin(), path.begin() + upperIndex);
                newPath.insert(newPath.end(), upperDubin.begin(), upperDubin.end() - 1);
                newPath.insert(newPath.end(), path.begin() + upper, path.end());
                path = std::move(newPath);
                upper = upperIndex;
            } 
            else {
                newPath.insert(newPath.end(), path.begin(), path.begin() + lower);
                newPath.insert(newPath.end(), lowerDubin.begin(), lowerDubin.end() - 1);
                newPath.insert(newPath.end(), path.begin() + lowerIndex, path.end());
                path = std::move(newPath);
                lower = static_cast<int>(lowerDubin.size());
                upper += static_cast<int>(lowerDubin.size()) - lowerIndex - 1;
            }
        }
        else if (upperDubin.size() != 0) {
            newPath.insert(newPath.end(), path.begin(), path.begin() + upperIndex);
            newPath.insert(newPath.end(), upperDubin.begin(), upperDubin.end() - 1);
            newPath.insert(newPath.end(), path.begin() + upper, path.end());
            path = std::move(newPath);
            upper = upperIndex;
        }
        else if (lowerDubin.size() != 0) {
            newPath.insert(newPath.end(), path.begin(), path.begin() + lower);
            newPath.insert(newPath.end(), lowerDubin.begin(), lowerDubin.end() - 1);
            newPath.insert(newPath.end(), path.begin() + lowerIndex, path.end());
            path = std::move(newPath);
            lower = static_cast<int>(lowerDubin.size());
            upper += static_cast<int>(lowerDubin.size()) - lowerIndex - 1;
        }
        if (newPath.size() != 0) {
            path = std::move(newPath);
        }
    }
    return path;
}

std::vector<PathItem> HybridAStar::search(
    Pose const& start, Pose const& goal, std::vector<std::vector<double>> const& map,
    std::array<double, 2> xlim, std::array<double,2> ylim) {

    double minX {xlim[0]};
    double minY {ylim[0]};

    // Initiate start as goal and goal as start, since search is backwards.
    Node startNode {};
    startNode.x = goal[0] - minX;
    startNode.y = goal[1] - minY;
    startNode.theta = wrap_angle(goal[2] + pi);

    Node goalNode {};
    goalNode.x = start[0] - minX;
    goalNode.y = start[1] - minY;
    goalNode.theta = wrap_angle(start[2] + pi);

    height = static_cast<int>(map.size());
    width = height ? static_cast<int>(map[0].size()) : 0;
    if (!width) return {};
    std::vector<uint8_t> grid(width * height, 0);
    for (int y {0}; y < height; ++y) {
        if (static_cast<int>(map[y].size()) != width) return {};
        for (int x {0}; x < width; ++x) grid[y * width + x] = map[y][x] != 0;
    }
    mapGrid = inflateMap(grid);

    if (isColliding({startNode.x, startNode.y, 0}) || isColliding({goalNode.x, goalNode.y, 0}) ) {
        if (isColliding({startNode.x, startNode.y, 0})) {
            std::cout << "Start is inside object" << std::endl;
        }
        if (isColliding({goalNode.x, goalNode.y, 0})) {
            std::cout << "Goal is inside object" << std::endl;
        }

        return {};
    }
    startNode.h = heuristic(startNode, goalNode);
    
    struct QueueEntry {
        double f;
        std::size_t counter;
        std::shared_ptr<Node> node;

        bool operator>(const QueueEntry& other) const {
            if (f != other.f)
                return f > other.f;

            return counter > other.counter;
        }
    };

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> openSet;
    std::vector<double> best_gs(width*height*THETA_BINS, std::numeric_limits<double>::infinity());
    std::size_t counter {0};

    auto firstNode {std::make_shared<Node>(startNode)};
    openSet.push(QueueEntry{firstNode->g + firstNode->h, counter++, firstNode});
    best_gs[stateIndex(firstNode->x, firstNode->y, firstNode->theta, width)] = firstNode->g;
    
    int iterations {0};
    int maxIterations {1000000};
    while (!openSet.empty()) {
        if (++iterations > maxIterations) {
            std::cout << "Max iterations reached" << std::endl;
            return {};
        }

        auto current {openSet.top().node};
        openSet.pop();

        auto key {stateIndex(current->x, current->y, current->theta, width)};
        if (current->g > best_gs[key]) continue;

        if (isFinished(*current, goalNode)) {
            std::cout << "Path found" << std::endl;
            std::vector<PathItem> path {reconstructPath(current)};

            // Find shortcuts using dubin paths
            path.back() = {goalNode.x, goalNode.y, wrap_angle(goalNode.theta), 0};
            path = addDubinPaths(path);

            // Reverse found path, since search is backwards
            std::reverse(path.begin(), path.end());
            for (auto& point : path) {
                point[0] += minX;
                point[1] += minY;
                point[2] = wrap_angle(point[2] + pi);
            }
            return path;
        }
        
        for(size_t i {0}; i < steeringAngles.size(); ++i) {
            auto newPoint {propogate(*current, dThetas[i])};
            if (newPoint[0] < 0) {continue;}

            double newX {newPoint[0]};
            double newY {newPoint[1]};
            double newTheta {newPoint[2]};
            
            // Check if this is the cheapest path here yet
            double newg {current->g + propogationDistance * propogationDistance};
            int key {stateIndex(newX, newY, newTheta, width)};
            if (newg >= best_gs[key]) {continue;} // Skip if there is a cheaper way
            best_gs[key] = newg;

            auto newNode {std::make_shared<Node>()};
            newNode->x = newX;
            newNode->y = newY;
            newNode->theta = newTheta;
            newNode->steering = steeringAngles[i];
            newNode->parent = current;
            newNode->g = newg;
            newNode->h = heuristic(*newNode, goalNode);
            
            openSet.push(QueueEntry{newNode->g + newNode->h, counter++, newNode});
        }
    }
    std::cout << "No path found" << std::endl;
    return {};
};