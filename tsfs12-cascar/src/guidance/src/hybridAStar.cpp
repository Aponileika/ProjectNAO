#include "guidance/hybridAStar.hpp"
#include "guidance/dubinPath.hpp"

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
using PathItem = std::array<double, 5>;

struct Node {
    double x;
    double y;
    double theta;

    int gx;
    int gy;
    int thetaIndex;

    double dir;
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
    dirs = {-1, 1};

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

void HybridAStar::addDistance(std::vector<PathItem>& path) {
    double dist {0.0};
    for (int i {1}; i < path.size(); ++i) {
        double dx {path[i][0] - path[i-1][0]};
        double dy {path[i][1] - path[i-1][1]};

        dist += std::sqrt(dx*dx + dy*dy);

        path[i][4] = dist;
    }
};

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

int HybridAStar::stateIndex(int const gx, int const gy, int thetaIndex, double width) {
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
        path.push_back({node->x, node->y, node->theta, node->dir, 0.0});
        node = node -> parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
};

std::pair<Pose, std::array<int, 2>> HybridAStar::propogate(Node const& node, double const dTheta, int const dir) {
    double x {node.x};
    double y {node.y};
    double theta {node.theta};

    std::array<int, 2> gridIndex {};
    int gx {};
    int gy {};

    Pose result {{x, y, theta}};
    double travelled {0};
    while(travelled < propogationDistance) {
        double ds {dir * std::min(propogationDistance - travelled, propogationInterval)};
        x += ds * std::cos(theta);
        y += ds * std::sin(theta);

        gridIndex = worldToGrid(x, y);
        gx = gridIndex[0];
        gy = gridIndex[1];
        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            return {{-1, -1, -1}, {0,0}};
        }
        if (mapGrid[gy * width + gx]) {
            return {{-1, -1, -1}, {0,0}};
        }
        theta += ds * dTheta;

        result = {x, y, theta};
        travelled += std::abs(ds);
    }
    result = {result[0], result[1], wrap_angle(result[2])};
    return {result, gridIndex};
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

size_t HybridAStar::getShortestDubin(
    std::vector<std::vector<Pose>> const& dubinPaths,
    std::vector<double> const& pathLengths, double maximumLength) {

   size_t bestPathIndex {1024};
    for (std::size_t i {0}; i < dubinPaths.size(); ++i) {
        if (pathLengths[i] < maximumLength && !isColliding(dubinPaths[i])) {
            bestPathIndex = i;
            maximumLength = pathLengths[i];
        }
    }
    return bestPathIndex;
}

void HybridAStar::insertPath(std::vector<PathItem>& path, std::vector<PathItem> const& shortcut, int lowerCut, int upperCut) {
    std::vector<PathItem> newPath {};
    newPath.insert(newPath.end(), path.begin(), path.begin() + lowerCut);
    newPath.insert(newPath.end(), shortcut.begin(), shortcut.end() - 1);
    newPath.insert(newPath.end(), path.begin() + upperCut, path.end());
    path = std::move(newPath);
}

std::vector<PathItem> HybridAStar::poseToPath(std::vector<Pose> const& poseVect, double const dir) {
    std::vector<PathItem> result {};
    result.reserve(poseVect.size());
    for (Pose const& pose : poseVect) {
        result.push_back({pose[0], pose[1], pose[2], dir, 0});
    }
    return result;
}

void HybridAStar::addDubinPaths(std::vector<PathItem> & path) {
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
        double upperShortcutLength {0};
        std::vector<Pose> upperDubin {};
        PathItem staticPoint {path[upper]};
        double upperDir {staticPoint[3]};
        int upperIndex {};
        for (int i {static_cast<int>(lower)}; i<upper-1; ++i) {
            PathItem dynamicPoint {path[i]};
            if (upperDir != dynamicPoint[3]) continue;
            std::pair<std::vector<std::vector<Pose>>, std::vector<double>> result {};
            if (upperDir == 1){
                result = dubinsPath({dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, 
                                    {staticPoint[0], staticPoint[1], staticPoint[2]}, false);
            }
            else {
                result = dubinsPath({staticPoint[0], staticPoint[1], staticPoint[2]}, 
                                    {dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, false);
            }
            std::vector<std::vector<Pose>> dubins {result.first};
            std::vector<double> dubinLengths {result.second};

            size_t bestPathIndex {getShortestDubin(dubins, dubinLengths, path[upper][4] - path[i][4])};
            if (bestPathIndex != 1024) {
                upperDubin = dubins[bestPathIndex];
                if (upperDir == -1) std::reverse(upperDubin.begin(), upperDubin.end());
                upperShortcutLength = (path[upper][4] - path[i][4]) - dubinLengths[bestPathIndex];
                upperIndex = i;
                break;
            }
        }

        // Calculate lower path
        double lowerShortcutLength {0};
        std::vector<Pose> lowerDubin {};
        staticPoint = path[lower];
        double lowerDir {staticPoint[3]};
        int lowerIndex {};
        for (int i {static_cast<int>(upper)}; i>lower+1; --i) {
            PathItem dynamicPoint {path[i]};
            if (lowerDir != dynamicPoint[3]) continue;
            std::pair<std::vector<std::vector<Pose>>, std::vector<double>> result {};
            if (lowerDir == 1) {
                result = dubinsPath({staticPoint[0], staticPoint[1], staticPoint[2]}, 
                                            {dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, false);
            }
            else {
               result = dubinsPath({dynamicPoint[0], dynamicPoint[1], dynamicPoint[2]}, 
                                             {staticPoint[0], staticPoint[1], staticPoint[2]}, false);
            }
            std::vector<std::vector<Pose>> dubins {result.first};
            std::vector<double> dubinLengths {result.second};

            size_t bestPathIndex {getShortestDubin(dubins, dubinLengths, path[i][4] - path[lower][4])};
            if (bestPathIndex != 1024) {
                lowerDubin = dubins[bestPathIndex];
                if (lowerDir == -1) std::reverse(lowerDubin.begin(), lowerDubin.end());
                lowerShortcutLength = (path[i][4]-path[lower][4]) - dubinLengths[bestPathIndex];
                lowerIndex = i;
                break;
            }
        }
        if (lowerShortcutLength == 0.0) ++lower;
        
        // Add the largest shortcut
        if (upperShortcutLength > 0 || lowerShortcutLength > 0){
            if (upperShortcutLength >= lowerShortcutLength) {
                std::vector<PathItem> upperDubinPath {poseToPath(upperDubin, upperDir)};
                insertPath(path, upperDubinPath, upperIndex, upper);
                upper = upperIndex;
            }
            else {
                std::vector<PathItem> lowerDubinPath {poseToPath(lowerDubin, lowerDir)};
                insertPath(path, lowerDubinPath, lower, lowerIndex);
                lower += static_cast<int>(lowerDubin.size()) - 1;
                upper += static_cast<int>(lowerDubin.size()) - lowerIndex - 1;
            }
            addDistance(path);
        }
    }
}


std::vector<PathItem> HybridAStar::search(
    Pose const& start, Pose const& goal, std::vector<std::vector<double>> const& map,
    std::array<double, 2> xlim, std::array<double,2> ylim) {

    std::cout << "Entered search function!" << std::endl;

    if (map.size() == 0) {
        std::cout << "No map, dubin search started!" << std::endl;
        std::pair result {dubinsPath(start, goal, false)};

        std::vector<std::vector<Pose>> dubins {result.first};
        std::vector<double> dubinLengths {result.second};

        double maximumLength {1024};
        size_t bestPathIndex {0};
        for (std::size_t i {0}; i < dubinPaths.size(); ++i) {
            if (pathLengths[i] < maximumLength) {
                bestPathIndex = i;
                maximumLength = pathLengths[i];
            }
        }
        std::vector<PathItem> path {poseToPath(dubins[bestPathIndex], 1)};

        addDistance(path);

        return path;
    }

    double minX {xlim[0]};
    double minY {ylim[0]};

    // Initiate start as goal and goal as start, since search is backwards.
    Node startNode {};
    startNode.x = goal[0] - minX;
    startNode.y = goal[1] - minY;
    startNode.theta = wrap_angle(goal[2] + pi);
    startNode.dir = 1;

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

        auto key {stateIndex(current->gx, current->gy, current->thetaIndex, width)};
        if (current->g > best_gs[key]) continue;

        if (isFinished(*current, goalNode)) {
            std::cout << "Path found" << std::endl;
            std::vector<PathItem> path {reconstructPath(current)};
            addDistance(path);

            // Find shortcuts using dubin paths. Twice for even better results
            path.back() = {goalNode.x, goalNode.y, wrap_angle(goalNode.theta), 0, 0};
            addDubinPaths(path);
            addDubinPaths(path);
            addDubinPaths(path);

            // Reverse found path, since search is backwards
            std::reverse(path.begin(), path.end());
            for (auto& point : path) {
                point[0] += minX;
                point[1] += minY;
                point[2] = wrap_angle(point[2] + pi);
            }
            addDistance(path);
            return path;
        }
        
        for(int dir : dirs) {
            for(size_t i {0}; i < steeringAngles.size(); ++i) {
                auto result {propogate(*current, dThetas[i], dir)};
                auto newPoint {result.first};
                auto gKey {result.second};
                int gx {gKey[0]};
                int gy {gKey[1]};
                
                if (newPoint[0] < 0) {continue;}

                double newX {newPoint[0]};
                double newY {newPoint[1]};
                double newTheta {newPoint[2]};

                int thetaIndex {static_cast<int>(std::round(newTheta / theta_resolution))};
                
                // Check if this is the cheapest path here yet
                double newg {current->g + propogationDistance};
                if (dir == -1) newg += propogationDistance; // punish going backwards
                if (current->dir != dir) newg += propogationDistance; //Punish switching direction
                int key {stateIndex(gx, gy, thetaIndex, width)};
                if (newg >= best_gs[key]) {continue;} // Skip if there is a cheaper way
                best_gs[key] = newg;

                auto newNode {std::make_shared<Node>()};
                newNode->x = newX;
                newNode->y = newY;
                newNode->theta = newTheta;
                newNode->gx = gx;
                newNode->gy = gy;
                newNode->thetaIndex = thetaIndex;
                newNode->dir = dir;
                newNode->steering = steeringAngles[i];
                newNode->parent = current;
                newNode->g = newg;
                newNode->h = heuristic(*newNode, goalNode);
                
                openSet.push(QueueEntry{newNode->g + newNode->h, counter++, newNode});
            }
        }
    }
    std::cout << "No path found" << std::endl;
    return {};
};