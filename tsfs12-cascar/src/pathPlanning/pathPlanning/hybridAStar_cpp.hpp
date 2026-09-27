#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <memory>

class HybridAStar {
    public:
        HybridAStar();

        using Pose = std::array<double, 3>;
        using PathItem = std::array<double, 4>;

        std::vector<PathItem> search(
            Pose const& start, Pose const& goal, std::vector<std::vector<double>> const& map,
            std::array<double, 2> xlim, std::array<double,2> ylim);


    private:
        struct Node {
            double x;
            double y;
            double theta;

            int gx;
            int gy;
            int thetaIndex;

            double steering;

            std::shared_ptr<Node> parent;
            double g;
            double h;

            double f() const {return g + h;}
        };

        double Vmax;
        double deltaMax;
        int N;
        std::vector<double> steeringAngles;

        std::vector<uint8_t> mapGrid;
        double width;
        double height;

        double xy_resolution;
        double theta_resolution;
        double map_resolution;
        double xy_tolerance;
        double theta_tolerance;

        double L;
        double track;
        double radius;

        double propogationDistance;
        double propogationInterval;
        std::vector<double> dThetas;

        std::vector<uint8_t> inflateMap(const std::vector<uint8_t>& grid);
        double wrap_angle(double theta);
        std::array<int, 2> worldToGrid(double const x, double const y);
        int stateIndex(int const gx, int const gy, int const thetaIndex, double width);
        bool isColliding(std::vector<Pose> const& trajectory);
        bool isColliding(std::vector<PathItem> const& path);
        bool isColliding(Pose const& pose);
        double heuristic(Node const& start, Node const& goal);
        std::vector<PathItem> reconstructPath(std::shared_ptr<Node> node);
        std::pair<Pose, std::array<int, 2>> propogate(Node const& node, double const dTheta);
        bool isFinished(Node const& node, Node const& goal);
        std::vector<PathItem> getShortestDubin(
            std::vector<std::vector<PathItem>> const& paths,
            std::vector<double> const& pathLengths, double const lowerLimit);
        std::vector<PathItem> addDubinPaths(std::vector<PathItem> & path);

};