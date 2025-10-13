#include "Task.h"

Task::Task(int taskId, const Position& start, const Position& end)
    : id(taskId), startPoint(start), endPoint(end),
      assignedRobotId(-1), isCompleted(false) {
    createdTime = std::chrono::steady_clock::now();
}