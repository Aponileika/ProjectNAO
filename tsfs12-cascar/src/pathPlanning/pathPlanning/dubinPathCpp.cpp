#include "dubinPathCpp.hpp"

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <utility>

double const pi {3.14159265359};

using Vec2 = std::array<double,2>;
Vec2 operator+(Vec2 const& a, Vec2 const& b) {
    return {a[0]+b[0], a[1]+b[1]};
}
Vec2 operator-(Vec2 const& a, Vec2 const& b) {
    return {a[0]-b[0], a[1]-b[1]};
}
Vec2 operator*(double s, Vec2 const& v) {
    return {s * v[0], s*v[1]};
}
Vec2 operator/(Vec2 v, double s) {
    return {v[0]/s, v[1]/s};
}
bool operator==(Vec2 const& a, Vec2 const& b) {
    return (a[0]==b[0] && a[1]==b[1]);
}


struct pathConsts
{
    Vec2 startTang;
    Vec2 endTang;

    Vec2 startPos;
    Vec2 endPos;

    Vec2 startCircle;
    Vec2 endCircle;

    int startDir;
    int endDir;
    double length {0};
    bool valid {true};
};


Vec2 rot(Vec2 const& vector) {
    Vec2 rotated {-vector[1], vector[0]};
    return rotated;
}

double mod2pi(double theta) {
    double result {std::fmod(theta, 2*pi)};
    if (result < 0) {
        result += 2*pi;
    }
    return result;
}

double norm(Vec2 const& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1]);
}

double getPathLength(pathConsts const& config, double R) {
    double tangentLength {norm(config.startTang - config.endTang)};

    double startAngle {std::atan2(config.startPos[1]-config.startCircle[1],
                                  config.startPos[0]-config.startCircle[0])};
    double endAngle {std::atan2(config.startTang[1]-config.startCircle[1],
                                config.startTang[0]-config.startCircle[0])};
    double arcLenghtStart {R * mod2pi(config.startDir * (endAngle - startAngle))};

    startAngle = std::atan2(config.endTang[1]-config.endCircle[1],
                                  config.endTang[0]-config.endCircle[0]);
    endAngle = std::atan2(config.endPos[1]-config.endCircle[1],
                                config.endPos[0]-config.endCircle[0]);
    double arcLenghtEnd {R * mod2pi(config.endDir * (endAngle - startAngle))};   
    
    return arcLenghtStart + tangentLength + arcLenghtEnd;
}

void sampleArc(
    pathObj & sampledPath, double const startAngle, double const endAngle, int const dir,
    double const R, Vec2 const& center, double spacing){
    
    double arcLength {R * mod2pi(dir * (endAngle - startAngle))};
    int sampleCount {static_cast<int>(std::max(1.0, std::round(arcLength / spacing)))};
    double dTheta {arcLength / (R * sampleCount)};

    for (int i {0}; i <= sampleCount; ++i){
        double circleAngle {startAngle + dir*i*dTheta};
        sampledPath.push_back( {center[0] + R*std::cos(circleAngle),
                                center[1] + R*std::sin(circleAngle),
                                mod2pi(circleAngle + dir*pi/2),
                                0.0 });
    }

}

void sampleTangent(
    pathObj & sampledPath, pathConsts const& config, double const spacing
) {
    double dx {config.endTang[0] - config.startTang[0]};
    double dy {config.endTang[1] - config.startTang[1]};

    double tangentLength {std::sqrt(dx*dx + dy*dy)};
    int sampleCount {static_cast<int>(std::max(1.0, round(tangentLength / spacing)))};

    dx /= sampleCount;
    dy /= sampleCount;
    double theta {mod2pi(std::atan2(dy,dx))};

    for (int i {1}; i <= sampleCount; ++i){
        sampledPath.push_back( {config.startTang[0] + dx*i,
                                config.startTang[1] + dy*i,
                                theta,
                                0.0});
    }
}


void getPathDistance(pathObj & path){
    double distance {0};
    for (std::size_t i {0}; i + 1 < path.size(); ++i){
        auto const& startPoint {path[i]};
        auto const& nextPoint {path[i+1]};

        double dx {nextPoint[0]-startPoint[0]};
        double dy {nextPoint[1]-startPoint[1]};

        distance += std::sqrt(dx*dx + dy*dy);
        path[i+1][3] = distance;
    }
    return;
}


pathObj createPath(
    pathConsts const& config, double const R, double const spacing, bool getDistance) {
    pathObj sampledPath;
    sampledPath.reserve(static_cast<std::size_t>(getPathLength(config, R) / spacing) + 3);

    double startAngle {std::atan2(config.startPos[1]-config.startCircle[1],
                                  config.startPos[0]-config.startCircle[0])};
    double endAngle {std::atan2(config.startTang[1]-config.startCircle[1],
                                config.startTang[0]-config.startCircle[0])};
    sampleArc(sampledPath, startAngle, endAngle, config.startDir, R, config.startCircle, spacing);

    sampleTangent(sampledPath, config, spacing);

    startAngle = std::atan2(config.endTang[1]-config.endCircle[1],
                                  config.endTang[0]-config.endCircle[0]);
    endAngle = std::atan2(config.endPos[1]-config.endCircle[1],
                                config.endPos[0]-config.endCircle[0]);
    sampleArc(sampledPath, startAngle, endAngle, config.endDir, R, config.endCircle, spacing);
    
    if (getDistance) {
        getPathDistance(sampledPath);
    }

    return sampledPath;
}


std::pair<pathsObj, std::vector<double>> dubinsPath(
    std::array<double, 3> const start, std::array<double, 3> const goal, bool const getDistance) {

    double const spacing {0.2};
    double const L {0.285};
    double const deltaMax {pi / 6};
    double const multiplier {1};
    double const R {multiplier * L / std::tan(deltaMax)};

    Vec2 const startPos {start[0], start[1]};
    Vec2 const goalPos {goal[0], goal[1]};

    Vec2 perp { rot( {std::cos(start[2]), std::sin(start[2])} ) };
    Vec2 const circleStartCCW {startPos + R * perp};
    Vec2 const circleStartCW {startPos - R*perp};

    perp = rot( {std::cos(goal[2]), std::sin(goal[2])} );
    Vec2 const circleEndCCW {goalPos + R*perp};
    Vec2 const circleEndCW {goalPos - R*perp};

    //Configure LSL path
    pathConsts LSL;
    if (circleEndCCW == circleStartCCW) {
        LSL.valid = false;
    }
    else {
        Vec2 circleVector {circleEndCCW - circleStartCCW};
        Vec2 circleNorm {circleVector / norm(circleVector) };

        LSL.startTang = circleStartCCW + R*rot(-1*circleNorm);
        LSL.endTang = circleEndCCW + R*rot(-1*circleNorm);
        LSL.startPos = startPos;
        LSL.endPos = goalPos;
        LSL.startCircle = circleStartCCW;
        LSL.endCircle = circleEndCCW;
        LSL.startDir = 1;
        LSL.endDir = 1;
        LSL.length = getPathLength(LSL, R);
    }

    //Configure RSR path
    pathConsts RSR;
    if (circleEndCW == circleStartCW) {
        RSR.valid = false;
    }
    else {
        Vec2 circleVector {circleEndCW - circleStartCW};
        Vec2 circleNorm = {circleVector / norm(circleVector)};
        
        RSR.startTang = circleStartCW + R*rot(circleNorm);
        RSR.endTang = circleEndCW + R*rot(circleNorm);
        RSR.startPos = startPos;
        RSR.endPos = goalPos;
        RSR.startCircle = circleStartCW;
        RSR.endCircle = circleEndCW;
        RSR.startDir = -1;
        RSR.endDir = -1;
        RSR.length = getPathLength(RSR, R);        
    }

    //Configure LSR path
    pathConsts LSR;
    Vec2 Dvec {circleEndCW - circleStartCCW};
    double D2 {Dvec[0]*Dvec[0] + Dvec[1]*Dvec[1]};
    if (D2 >= 4*R*R) {
        double D {std::sqrt(D2)};

        double phi {std::atan2(circleEndCW[1] - circleStartCCW[1], circleEndCW[0]-circleStartCCW[0])};
        double alpha {std::acos(2*R/D)};
        double theta {phi-alpha};
        Vec2 v {std::cos(theta), std::sin(theta)};

        LSR.startTang = circleStartCCW + R*v;
        LSR.endTang = circleEndCW - R*v;
        LSR.startPos = startPos;
        LSR.endPos = goalPos;
        LSR.startCircle = circleStartCCW;
        LSR.endCircle = circleEndCW;
        LSR.startDir = 1;
        LSR.endDir = -1;
        LSR.length = getPathLength(LSR, R);
    }
    else {
        LSR.valid = false;
    }

    //Configure RSL path
    pathConsts RSL;
    Dvec = circleEndCCW - circleStartCW;
    D2 = Dvec[0]*Dvec[0] + Dvec[1]*Dvec[1];
    if (D2 >= 4*R*R) {
        double D {std::sqrt(D2)};

        double phi {std::atan2(circleEndCCW[1] - circleStartCW[1], circleEndCCW[0]-circleStartCW[0])};
        double alpha {std::acos(2*R/D)};
        double theta {phi+alpha};
        Vec2 v {std::cos(theta), std::sin(theta)};

        RSL.startTang = circleStartCW + R*v;
        RSL.endTang = circleEndCCW - R*v;
        RSL.startPos = startPos;
        RSL.endPos = goalPos;
        RSL.startCircle = circleStartCW;
        RSL.endCircle = circleEndCCW;
        RSL.startDir = -1;
        RSL.endDir = 1;
        RSL.length = getPathLength(RSL, R);
    }
    else {
        RSL.valid = false;
    }
    
    std::array<pathConsts, 4> paths {LSL, RSR, LSR, RSL};
    pathsObj validPaths; validPaths.reserve(4);
    std::vector<double> pathLengths; pathLengths.reserve(4);

    for (pathConsts& path : paths) {
        if (path.valid) {
            pathLengths.push_back(path.length);
            validPaths.push_back(createPath(path, R, spacing, getDistance));
        }
    }

    return {validPaths, pathLengths};
}