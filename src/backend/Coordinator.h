#ifndef COORDINATOR_H
#define COORDINATOR_H

#include "../core/Robot.h"
#include "../core/Task.h"
#include "../core/Map.h"
#include "PathPlanner.h"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <QObject>
#include <QTimer>

class Coordinator : public QObject {
    Q_OBJECT

public:
    explicit Coordinator(Map* map, QObject* parent = nullptr);
    ~Coordinator();

    void addRobot(std::shared_ptr<Robot> robot);
    void removeRobot(int robotId);
    std::shared_ptr<Robot> getRobot(int robotId) const;

    int createTask(const Position& start, const Position& end);
    void assignTaskToRobot(int taskId, int robotId);
    void completeTask(int taskId);
    std::shared_ptr<Task> getTask(int taskId) const;

    void startScheduling();
    void stopScheduling();
    void update();

    int getActiveRobotCount() const;
    int getPendingTaskCount() const;
    int getCompletedTaskCount() const;

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
    QTimer* updateTimer;

    std::atomic<int> nextTaskId;
    std::atomic<int> completedTasks;

    std::shared_ptr<Robot> findNearestIdleRobot(const Position& target) const;
    void updateRobotPositions();
    void handleRobotMovement();
    void assignTasks();
};

#endif // COORDINATOR_H