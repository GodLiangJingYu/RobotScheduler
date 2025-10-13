#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <atomic>
#include <mutex>
#include <memory>
#include <vector>

enum class RobotType {
    AGV,
    MIR,
    HUMANOID,
    QUADRUPED
};

enum class RobotState {
    IDLE,
    MOVING_TO_TASK,
    EXECUTING_TASK,
    MOVING_TO_END,
    COMPLETED
};

struct Position {
    int x, y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

class Robot {
public:
    Robot(int id, RobotType type, const Position& pos);

    // Getters
    int getId() const { return id; }
    RobotType getType() const { return type; }
    Position getPosition() const { return position; }
    RobotState getState() const { return state; }
    Position getTarget() const { return target; }
    std::vector<Position> getPath() const { return path; }
    int getCurrentTaskId() const { return currentTaskId; }

    // Setters
    void setPosition(const Position& pos) { position = pos; }
    void setState(RobotState newState) { state = newState; }
    void setTarget(const Position& target) { this->target = target; }
    void setPath(const std::vector<Position>& newPath) { path = newPath; }
    void setCurrentTaskId(int taskId) { currentTaskId = taskId; }

    // Task management
    bool hasTask() const { return currentTaskId != -1; }
    void assignTask(int taskId) { currentTaskId = taskId; }
    void completeTask() { currentTaskId = -1; state = RobotState::IDLE; }

    // Movement
    bool moveStep();
    bool isAtTarget() const;

    // Type conversion
    static std::string typeToString(RobotType type);

private:
    int id;
    RobotType type;
    Position position;
    RobotState state;
    Position target;
    std::vector<Position> path;
    int currentTaskId;
    int pathIndex;
    mutable std::mutex mutex;
};

#endif // ROBOT_H