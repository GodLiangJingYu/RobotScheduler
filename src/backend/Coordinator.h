#ifndef COORDINATOR_H
#define COORDINATOR_H

#include "../core/Robot.h"
#include "../core/Task.h"
#include "../core/Map.h"
#include "PathPlanner.h"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

class Coordinator {
public:
    Coordinator(Map* map);
    ~Coordinator();

    // Robot management
    void addRobot(std::shared_ptr<Robot> robot);
    void removeRobot(int robotId);
    std::shared_ptr<Robot> getRobot(int robotId) const;

    // Task management
    int createTask(const Position& start, const Position& end);
    void assignTaskToRobot(int taskId, int robotId);
    void completeTask(int taskId);
    std::shared_ptr<Task> getTask(int taskId) const;

    // Scheduling
    void startScheduling();
    void stopScheduling();
    void update();

    // Statistics
    int getActiveRobotCount() const;
    int getPendingTaskCount() const;
    int getCompletedTaskCount() const;

    // Status
    bool isRunning() const { return running; }
    bool isSimulationComplete() const;

private:
    Map* map;
    PathPlanner pathPlanner;

    std::vector<std::shared_ptr<Robot>> robots;
    std::unordered_map<int, std::shared_ptr<Task>> tasks;
    std::unordered_map<int, std::shared_ptr<Robot>> robotMap;

    mutable std::mutex robotsMutex;
    mutable std::mutex tasksMutex;

    std::atomic<bool> running;
    std::thread schedulerThread;
    std::condition_variable cv;

    std::atomic<int> nextTaskId;
    int completedTasks;

    // Internal methods
    void schedulerLoop();
    std::shared_ptr<Robot> findNearestIdleRobot(const Position& target) const;
    void updateRobotPositions();
    void handleRobotMovement();
    void assignTasks();
};

#endif // COORDINATOR_H