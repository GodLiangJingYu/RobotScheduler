#include "Robot.h"
#include <cmath>

Robot::Robot(int id, RobotType type, const Position& pos)
    : id(id), type(type), position(pos), state(RobotState::IDLE),
      target(0, 0), currentTaskId(-1), pathIndex(0) {}

void Robot::setPath(const std::vector<Position>& newPath) {
    std::lock_guard<std::mutex> lock(mutex);
    originalPath = newPath;

    if (newPath.size() <= 1) {
        path = newPath;
    } else {
        path = interpolatePath(newPath);
    }
    pathIndex = 0;
}

std::vector<Position> Robot::interpolatePath(const std::vector<Position>& waypoints) {
    if (waypoints.size() <= 1) return waypoints;

    std::vector<Position> interpolated;
    interpolated.reserve(waypoints.size() * 3);

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Position start = waypoints[i];
        Position end = waypoints[i + 1];

        interpolated.push_back(start);

        int midX = (start.x + end.x) / 2;
        int midY = (start.y + end.y) / 2;
        interpolated.push_back(Position(midX, midY));
    }

    interpolated.push_back(waypoints.back());
    return interpolated;
}

bool Robot::moveStep() {
    std::lock_guard<std::mutex> lock(mutex);

    if (path.empty() || pathIndex >= path.size()) {
        return false;
    }

    position = path[pathIndex];
    pathIndex++;

    return pathIndex < path.size();
}

bool Robot::isAtTarget() const {
    std::lock_guard<std::mutex> lock(mutex);
    int dx = std::abs(position.x - target.x);
    int dy = std::abs(position.y - target.y);
    return (dx + dy) < 50;
}

std::string Robot::typeToString(RobotType type) {
    switch (type) {
        case RobotType::AGV: return "AGV";
        case RobotType::MIR: return "MIR";
        case RobotType::HUMANOID: return "HUMANOID";
        case RobotType::QUADRUPED: return "QUADRUPED";
        default: return "UNKNOWN";
    }
}