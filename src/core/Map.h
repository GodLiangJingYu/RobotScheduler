#ifndef MAP_H
#define MAP_H

#include "Robot.h"
#include <vector>
#include <mutex>
#include <unordered_set>
#include <unordered_map>

class Map {
public:
    static constexpr int MAP_SIZE = 10000;
    static constexpr int BLOCK_SIZE = 8; // 每个块8x8格子
    static constexpr int GRID_SIZE = MAP_SIZE / BLOCK_SIZE; // 1250x1250块

    Map();

    // 障碍物管理
    void addObstacle(const Position& pos);
    void removeObstacle(const Position& pos);
    bool isObstacle(const Position& pos) const;
    void generateRandomObstacles(int count);

    // 机器人位置管理
    bool isPositionOccupied(const Position& pos) const;
    bool isPositionValid(const Position& pos) const;
    void registerRobotPosition(int robotId, const Position& pos);
    void unregisterRobotPosition(int robotId, const Position& pos);
    Position getRobotPosition(int robotId) const;

    // 块管理
    int getBlockIndex(const Position& pos) const;
    std::vector<int> getOccupiedRobotsInBlock(const Position& pos) const;
    bool isBlockOccupied(const Position& pos) const;

    // 边界检查
    bool isValidPosition(const Position& pos) const;

    // 获取地图信息
    std::vector<Position> getObstacles() const;
    std::vector<std::pair<int, Position>> getRobotPositions() const;

private:
    std::vector<std::vector<bool>> grid; // 障碍物网格
    std::unordered_map<int, Position> robotPositions; // 机器人位置映射
    std::vector<std::vector<std::vector<int>>> blockRobots; // 块中的机器人ID
    mutable std::mutex mutex;
};

#endif // MAP_H