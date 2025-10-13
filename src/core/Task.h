#ifndef TASK_H
#define TASK_H

#include "Robot.h"
#include <chrono>

struct Task {
    int id;
    Position startPoint;
    Position endPoint;
    int assignedRobotId;
    std::chrono::steady_clock::time_point createdTime;
    std::chrono::steady_clock::time_point assignedTime;
    std::chrono::steady_clock::time_point completedTime;
    bool isCompleted;

    Task(int taskId, const Position& start, const Position& end);

    bool isAssigned() const { return assignedRobotId != -1; }
    void assignToRobot(int robotId) {
        assignedRobotId = robotId;
        assignedTime = std::chrono::steady_clock::now();
    }
    void markCompleted() {
        isCompleted = true;
        completedTime = std::chrono::steady_clock::now();
    }
};

#endif // TASK_H