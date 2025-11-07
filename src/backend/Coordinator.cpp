#include "Coordinator.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <climits>

Coordinator::Coordinator(Map* map, QObject* parent)
    : QObject(parent), map(map), pathPlanner(map), running(false),
      updateTimer(nullptr), nextTaskId(0), completedTasks(0) {}

Coordinator::~Coordinator() {
    stopScheduling();
}

void Coordinator::addRobot(std::shared_ptr<Robot> robot) {
    std::lock_guard<std::mutex> lock(robotsMutex);
    robots.push_back(robot);
    robotMap[robot->getId()] = robot;
}

void Coordinator::removeRobot(int robotId) {
    std::lock_guard<std::mutex> lock(robotsMutex);
    robots.erase(std::remove_if(robots.begin(), robots.end(),
        [robotId](const std::shared_ptr<Robot>& r) { return r->getId() == robotId; }),
        robots.end());
    robotMap.erase(robotId);
}

std::shared_ptr<Robot> Coordinator::getRobot(int robotId) const {
    std::lock_guard<std::mutex> lock(robotsMutex);
    auto it = robotMap.find(robotId);
    return it != robotMap.end() ? it->second : nullptr;
}

int Coordinator::createTask(const Position& start, const Position& end) {
    std::lock_guard<std::mutex> lock(tasksMutex);
    int taskId = nextTaskId++;
    auto task = std::make_shared<Task>(taskId, start, end);
    tasks[taskId] = task;
    return taskId;
}

void Coordinator::assignTaskToRobot(int taskId, int robotId) {
    std::lock_guard<std::mutex> lock(tasksMutex);
    auto task = getTask(taskId);
    auto robot = getRobot(robotId);

    if (task && robot) {
        task->assignToRobot(robotId);
        robot->assignTask(taskId);
        robot->setState(RobotState::MOVING_TO_TASK);
        robot->setTarget(task->startPoint);

        // Plan path to task start point
        auto path = pathPlanner.findPath(robot->getPosition(), task->startPoint);
        robot->setPath(path);
    }
}

void Coordinator::completeTask(int taskId) {
    std::lock_guard<std::mutex> lock(tasksMutex);
    auto task = getTask(taskId);
    if (task) {
        task->markCompleted();
        completedTasks++;

        auto robot = getRobot(task->assignedRobotId);
        if (robot) {
            robot->completeTask();
        }
    }
}

std::shared_ptr<Task> Coordinator::getTask(int taskId) const {
    std::lock_guard<std::mutex> lock(tasksMutex);
    auto it = tasks.find(taskId);
    return it != tasks.end() ? it->second : nullptr;
}

void Coordinator::startScheduling() {
    if (running.exchange(true)) return;

    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &Coordinator::update);
    updateTimer->start(100);  // Update every 100ms
}

void Coordinator::stopScheduling() {
    if (!running.exchange(false)) return;

    if (updateTimer) {
        updateTimer->stop();
        updateTimer->deleteLater();
        updateTimer = nullptr;
    }
}

void Coordinator::update() {
    updateRobotPositions();
    handleRobotMovement();
    assignTasks();
}

void Coordinator::updateRobotPositions() {
    std::lock_guard<std::mutex> lock(robotsMutex);
    for (auto& robot : robots) {
        auto currentPos = robot->getPosition();
        map->unregisterRobotPosition(robot->getId(), currentPos);
    }

    for (auto& robot : robots) {
        auto currentPos = robot->getPosition();
        map->registerRobotPosition(robot->getId(), currentPos);
    }
}

void Coordinator::handleRobotMovement() {
    std::lock_guard<std::mutex> lock(robotsMutex);
    for (auto& robot : robots) {
        if (robot->getState() == RobotState::MOVING_TO_TASK) {
            if (!robot->moveStep()) {
                if (robot->isAtTarget()) {
                    // Reached task start point, now move to end point
                    auto task = getTask(robot->getCurrentTaskId());
                    if (task) {
                        robot->setState(RobotState::MOVING_TO_END);
                        robot->setTarget(task->endPoint);

                        auto path = pathPlanner.findPath(robot->getPosition(), task->endPoint);
                        robot->setPath(path);
                    }
                }
            }
        } else if (robot->getState() == RobotState::MOVING_TO_END) {
            if (!robot->moveStep()) {
                if (robot->isAtTarget()) {
                    // Reached end point, complete task
                    auto task = getTask(robot->getCurrentTaskId());
                    if (task) {
                        completeTask(task->id);
                    }
                }
            }
        }
    }
}

std::shared_ptr<Robot> Coordinator::findNearestIdleRobot(const Position& target) const {
    std::lock_guard<std::mutex> lock(robotsMutex);
    std::shared_ptr<Robot> nearestRobot = nullptr;
    int minDistance = INT_MAX;

    for (const auto& robot : robots) {
        if (robot->getState() == RobotState::IDLE) {
            int distance = std::abs(robot->getPosition().x - target.x) +
                          std::abs(robot->getPosition().y - target.y);
            if (distance < minDistance) {
                minDistance = distance;
                nearestRobot = robot;
            }
        }
    }

    return nearestRobot;
}

void Coordinator::assignTasks() {
    std::lock_guard<std::mutex> lock(tasksMutex);
    std::vector<int> unassignedTasks;

    for (const auto& pair : tasks) {
        if (!pair.second->isAssigned() && !pair.second->isCompleted) {
            unassignedTasks.push_back(pair.first);
        }
    }

    for (int taskId : unassignedTasks) {
        auto task = tasks[taskId];
        auto robot = findNearestIdleRobot(task->startPoint);

        if (robot) {
            assignTaskToRobot(taskId, robot->getId());
        }
    }
}

int Coordinator::getActiveRobotCount() const {
    std::lock_guard<std::mutex> lock(robotsMutex);
    return robots.size();
}

int Coordinator::getPendingTaskCount() const {
    std::lock_guard<std::mutex> lock(tasksMutex);
    int count = 0;
    for (const auto& pair : tasks) {
        if (!pair.second->isCompleted) count++;
    }
    return count;
}

int Coordinator::getCompletedTaskCount() const {
    return completedTasks;
}

bool Coordinator::isSimulationComplete() const {
    std::lock_guard<std::mutex> lock(tasksMutex);
    for (const auto& pair : tasks) {
        if (!pair.second->isCompleted) return false;
    }
    return true;
}