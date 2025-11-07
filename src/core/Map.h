#ifndef MAP_H
#define MAP_H

#include "Robot.h"
#include <vector>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <memory>

class Map {
public:
    static constexpr int MAP_SIZE = 10000;
    static constexpr int BLOCK_SIZE = 8;
    static constexpr int GRID_SIZE = MAP_SIZE / BLOCK_SIZE;

    Map();

    void addObstacle(const Position& pos);
    void removeObstacle(const Position& pos);
    bool isObstacle(const Position& pos) const;
    void generateRandomObstacles(int count);

    bool isPositionOccupied(const Position& pos) const;
    bool isPositionValid(const Position& pos) const;
    void registerRobotPosition(int robotId, const Position& pos);
    void unregisterRobotPosition(int robotId, const Position& pos);
    Position getRobotPosition(int robotId) const;

    int getBlockIndex(const Position& pos) const;
    std::vector<int> getOccupiedRobotsInBlock(const Position& pos) const;
    bool isBlockOccupied(const Position& pos) const;
    bool isValidPosition(const Position& pos) const;

    std::vector<Position> getObstacles() const;
    std::vector<std::pair<int, Position>> getRobotPositions() const;

private:
    std::vector<std::vector<bool>> grid;
    std::unordered_map<int, int> robotPositions;
    std::unordered_map<int, Position> robotPosMap;
    std::vector<std::vector<std::vector<int>>> blockRobots;

    mutable std::shared_ptr<std::vector<Position>> obstacleCache;
    mutable bool obstacleCacheDirty;

    mutable std::mutex mutex;
};

#endif // MAP_H