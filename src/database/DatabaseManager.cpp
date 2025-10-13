#include "DatabaseManager.h"
#include <iostream>

DatabaseManager::DatabaseManager(const std::string& dbPath) : db(nullptr), dbPath(dbPath) {}

DatabaseManager::~DatabaseManager() {
    if (db) {
        sqlite3_close(db);
    }
}

bool DatabaseManager::initialize() {
    int rc = sqlite3_open(dbPath.c_str(), &db);
    if (rc != SQLITE_OK) {
        std::cerr << "Cannot open database: " << sqlite3_errmsg(db) << std::endl;
        return false;
    }

    createTables();
    return true;
}

void DatabaseManager::createTables() {
    const char* createRobots = R"(
        CREATE TABLE IF NOT EXISTS robots (
            id INTEGER PRIMARY KEY,
            type TEXT NOT NULL,
            position_x INTEGER NOT NULL,
            position_y INTEGER NOT NULL,
            state INTEGER NOT NULL,
            current_task_id INTEGER DEFAULT -1
        );
    )";

    const char* createTasks = R"(
        CREATE TABLE IF NOT EXISTS tasks (
            id INTEGER PRIMARY KEY,
            start_x INTEGER NOT NULL,
            start_y INTEGER NOT NULL,
            end_x INTEGER NOT NULL,
            end_y INTEGER NOT NULL,
            assigned_robot_id INTEGER DEFAULT -1,
            is_completed BOOLEAN DEFAULT 0,
            created_time TEXT NOT NULL
        );
    )";

    executeQuery(createRobots);
    executeQuery(createTasks);
}

bool DatabaseManager::executeQuery(const std::string& query) {
    char* errMsg = nullptr;
    int rc = sqlite3_exec(db, query.c_str(), nullptr, nullptr, &errMsg);

    if (rc != SQLITE_OK) {
        std::cerr << "SQL error: " << errMsg << std::endl;
        sqlite3_free(errMsg);
        return false;
    }

    return true;
}

bool DatabaseManager::insertRobot(const std::shared_ptr<Robot>& robot) {
    std::string query = "INSERT INTO robots (id, type, position_x, position_y, state, current_task_id) VALUES (" +
                       std::to_string(robot->getId()) + ", '" +
                       Robot::typeToString(robot->getType()) + "', " +
                       std::to_string(robot->getPosition().x) + ", " +
                       std::to_string(robot->getPosition().y) + ", " +
                       std::to_string(static_cast<int>(robot->getState())) + ", " +
                       std::to_string(robot->getCurrentTaskId()) + ")";

    return executeQuery(query);
}

bool DatabaseManager::updateRobotPosition(int robotId, const Position& pos) {
    std::string query = "UPDATE robots SET position_x = " + std::to_string(pos.x) +
                       ", position_y = " + std::to_string(pos.y) +
                       " WHERE id = " + std::to_string(robotId);

    return executeQuery(query);
}

bool DatabaseManager::insertTask(const std::shared_ptr<Task>& task) {
    auto timePoint = std::chrono::system_clock::to_time_t(task->createdTime);
    std::string timeStr = std::to_string(timePoint);

    std::string query = "INSERT INTO tasks (id, start_x, start_y, end_x, end_y, created_time) VALUES (" +
                       std::to_string(task->id) + ", " +
                       std::to_string(task->startPoint.x) + ", " +
                       std::to_string(task->startPoint.y) + ", " +
                       std::to_string(task->endPoint.x) + ", " +
                       std::to_string(task->endPoint.y) + ", '" +
                       timeStr + "')";

    return executeQuery(query);
}

std::shared_ptr<Robot> DatabaseManager::getRobotById(int id) const {
    std::string query = "SELECT * FROM robots WHERE id = " + std::to_string(id);

    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);

    if (rc == SQLITE_OK && sqlite3_step(stmt) == SQLITE_ROW) {
        int dbId = sqlite3_column_int(stmt, 0);
        const char* typeStr = (const char*)sqlite3_column_text(stmt, 1);
        int x = sqlite3_column_int(stmt, 2);
        int y = sqlite3_column_int(stmt, 3);
        int state = sqlite3_column_int(stmt, 4);
        int taskId = sqlite3_column_int(stmt, 5);

        RobotType type = RobotType::AGV; // Default
        if (std::string(typeStr) == "MIR") type = RobotType::MIR;
        else if (std::string(typeStr) == "HUMANOID") type = RobotType::HUMANOID;
        else if (std::string(typeStr) == "QUADRUPED") type = RobotType::QUADRUPED;

        auto robot = std::make_shared<Robot>(dbId, type, Position(x, y));
        robot->setState(static_cast<RobotState>(state));
        robot->setCurrentTaskId(taskId);

        sqlite3_finalize(stmt);
        return robot;
    }

    sqlite3_finalize(stmt);
    return nullptr;
}

int DatabaseManager::getRobotCount() const {
    const char* query = "SELECT COUNT(*) FROM robots";
    sqlite3_stmt* stmt;
    int count = 0;

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) == SQLITE_OK) {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            count = sqlite3_column_int(stmt, 0);
        }
        sqlite3_finalize(stmt);
    }

    return count;
}