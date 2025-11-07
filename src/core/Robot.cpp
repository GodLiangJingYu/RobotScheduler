#include "Robot.h"
#include <cmath>
#include <algorithm>

Robot::Robot(int id, RobotType type, const Position& pos)
    : id(id), type(type), position(pos), state(RobotState::IDLE),
      target(0, 0), currentTaskId(-1), pathIndex(0) {}

void Robot::setPath(const std::vector<Position>& newPath) {
    std::lock_guard<std::mutex> lock(mutex);
    originalPath = newPath;
    path = interpolatePath(newPath);
    pathIndex = 0;
}

std::vector<Position> Robot::interpolatePath(const std::vector<Position>& waypoints) {
    if (waypoints.empty()) return {};
    if (waypoints.size() == 1) return waypoints;

    std::vector<Position> interpolated;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Position start = waypoints[i];
        Position end = waypoints[i + 1];

        int dx = end.x - start.x;
        int dy = end.y - start.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance == 0) {
            interpolated.push_back(start);
            continue;
        }

        int steps = std::max(1, static_cast<int>(distance / STEP_SIZE));

        for (int step = 0; step < steps; ++step) {
            double t = static_cast<double>(step) / steps;
            int x = static_cast<int>(start.x + t * dx);
            int y = static_cast<int>(start.y + t * dy);
            interpolated.push_back(Position(x, y));
        }
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
    return position == target;
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