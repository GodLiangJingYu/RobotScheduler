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
    auto task = getTask(taskId);
    auto robot = getRobot(robotId);

    if (!task || !robot) return;

    {
        std::lock_guard<std::mutex> lock(tasksMutex);
        task->assignToRobot(robotId);
    }

    robot->assignTask(taskId);
    robot->setState(RobotState::MOVING_TO_TASK);
    robot->setTarget(task->startPoint);

    Position start = robot->getPosition();
    Position end = task->startPoint;

    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);

    if (dx + dy < 500) {
        std::vector<Position> simplePath = {start, end};
        robot->setPath(simplePath);
    } else {
        auto path = pathPlanner.findPath(start, end);
        if (path.empty()) {
            path = {start, end};
        }
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
    updateTimer->start(200);
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
    std::vector<std::pair<int, Position>> updates;

    {
        std::lock_guard<std::mutex> lock(robotsMutex);
        updates.reserve(robots.size());
        for (auto& robot : robots) {
            updates.push_back({robot->getId(), robot->getPosition()});
        }
    }

    for (auto& update : updates) {
        auto oldPos = map->getRobotPosition(update.first);
        if (oldPos.x != -1 && !(oldPos == update.second)) {
            map->unregisterRobotPosition(update.first, oldPos);
            map->registerRobotPosition(update.first, update.second);
        } else if (oldPos.x == -1) {
            map->registerRobotPosition(update.first, update.second);
        }
    }
}

void Coordinator::handleRobotMovement() {
    std::vector<std::shared_ptr<Robot>> robotsCopy;
    {
        std::lock_guard<std::mutex> lock(robotsMutex);
        robotsCopy.reserve(robots.size());
        for (auto& r : robots) {
            if (r->getState() != RobotState::IDLE &&
                r->getState() != RobotState::COMPLETED) {
                robotsCopy.push_back(r);
            }
        }
    }

    for (auto& robot : robotsCopy) {
        if (robot->getState() == RobotState::MOVING_TO_TASK) {
            if (!robot->moveStep()) {
                if (robot->isAtTarget()) {
                    auto task = getTask(robot->getCurrentTaskId());
                    if (task) {
                        robot->setState(RobotState::MOVING_TO_END);
                        robot->setTarget(task->endPoint);

                        Position start = robot->getPosition();
                        Position end = task->endPoint;
                        std::vector<Position> path = {start, end};
                        robot->setPath(path);
                    }
                }
            }
        } else if (robot->getState() == RobotState::MOVING_TO_END) {
            if (!robot->moveStep()) {
                if (robot->isAtTarget()) {
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
    std::vector<std::shared_ptr<Robot>> idleRobots;
    {
        std::lock_guard<std::mutex> lock(robotsMutex);
        for (const auto& robot : robots) {
            if (robot->getState() == RobotState::IDLE) {
                idleRobots.push_back(robot);
            }
        }
    }

    if (idleRobots.empty()) return nullptr;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, std::min(9, (int)idleRobots.size() - 1));

    return idleRobots[dis(gen)];
}

void Coordinator::assignTasks() {
    std::vector<std::pair<int, Position>> unassignedTasks;
    {
        std::lock_guard<std::mutex> lock(tasksMutex);
        for (const auto& pair : tasks) {
            if (!pair.second->isAssigned() && !pair.second->isCompleted) {
                unassignedTasks.push_back({pair.first, pair.second->startPoint});
            }
        }
    }

    int maxAssignments = 5;
    int assigned = 0;

    for (auto& taskInfo : unassignedTasks) {
        if (assigned >= maxAssignments) break;

        auto robot = findNearestIdleRobot(taskInfo.second);
        if (robot) {
            assignTaskToRobot(taskInfo.first, robot->getId());
            assigned++;
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
    return completedTasks.load();
}

bool Coordinator::isSimulationComplete() const {
    std::lock_guard<std::mutex> lock(tasksMutex);
    for (const auto& pair : tasks) {
        if (!pair.second->isCompleted) return false;
    }
    return true;
}