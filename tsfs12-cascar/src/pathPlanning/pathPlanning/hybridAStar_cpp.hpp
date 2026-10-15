#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <memory>

class HybridAStar {
    public:
        HybridAStar();

        using Pose = std::array<double, 3>;
        using PathItem = std::array<double, 5>;

        std::vector<PathItem> search(
            Pose const& start, Pose const& goal, std::vector<std::vector<double>> const& map = {},
            std::array<double, 2> xlim = {0,0}, std::array<double,2> ylim = {0,0});


    private:
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

            double f() const {return g + h;}
        };

        double Vmax;
        double deltaMax;
        int N;
        std::vector<double> dirs;
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

        void addDistance(std::vector<PathItem>& path);
        std::vector<uint8_t> inflateMap(const std::vector<uint8_t>& grid);
        double wrap_angle(double theta);
        std::array<int, 2> worldToGrid(double const x, double const y);
        int stateIndex(int const gx, int const gy, int const thetaIndex, double width);
        bool isColliding(std::vector<Pose> const& trajectory);
        bool isColliding(std::vector<PathItem> const& path);
        bool isColliding(Pose const& pose);
        double heuristic(Node const& start, Node const& goal);
        std::vector<PathItem> reconstructPath(std::shared_ptr<Node> node);
        std::pair<Pose, std::array<int, 2>> propogate(Node const& node, double const dTheta, int const dir);
        bool isFinished(Node const& node, Node const& goal);
        size_t getShortestDubin(
            std::vector<std::vector<Pose>> const& dubinPaths,
            std::vector<double> const& pathLengths, double maximumLength);
        void insertPath(std::vector<PathItem>& path, std::vector<PathItem> const& shortcut, int lowerCut, int upperCut);
        std::vector<PathItem> poseToPath(std::vector<Pose> const& poseVect, double const dir);
        void addDubinPaths(std::vector<PathItem> & path);

};