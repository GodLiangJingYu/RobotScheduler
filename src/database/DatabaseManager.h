#ifndef DATABASEMANAGER_H
#define DATABASEMANAGER_H
#pragma once

#include "../core/Robot.h"
#include "../core/Task.h"
#include <sqlite3.h>
#include <string>
#include <vector>
#include <memory>

class DatabaseManager {
public:
    DatabaseManager(const std::string& dbPath = "robot_scheduler.db");
    ~DatabaseManager();

    bool initialize();

    // Robot operations
    bool insertRobot(const std::shared_ptr<Robot>& robot);
    bool updateRobotPosition(int robotId, const Position& pos);
    bool deleteRobot(int robotId);
    std::vector<std::shared_ptr<Robot>> getAllRobots() const;

    // Task operations
    bool insertTask(const std::shared_ptr<Task>& task);
    bool updateTaskStatus(int taskId, bool completed);
    bool updateTask(const std::shared_ptr<Task>& task);
    std::vector<std::shared_ptr<Task>> getAllTasks() const;

    // Query operations
    std::shared_ptr<Robot> getRobotById(int id) const;
    std::vector<Position> getRobotPositions() const;
    int getRobotCount() const;

private:
    sqlite3* db;
    std::string dbPath;
    mutable std::mutex dbMutex;

    bool executeQuery(const std::string& query);
    void createTables();
};

#endif // DATABASEMANAGER_H