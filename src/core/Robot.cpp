#include "Robot.h"
#include <cmath>

Robot::Robot(int id, RobotType type, const Position& pos)
    : id(id), type(type), position(pos), state(RobotState::IDLE),
      target(0, 0), currentTaskId(-1), pathIndex(0) {}

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