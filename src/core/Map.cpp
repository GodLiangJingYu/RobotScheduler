#include "Map.h"
#include <random>
#include <algorithm>

Map::Map() : grid(MAP_SIZE, std::vector<bool>(MAP_SIZE, false)),
             blockRobots(GRID_SIZE, std::vector<std::vector<int>>(GRID_SIZE)),
             obstacleCache(nullptr),
             obstacleCacheDirty(true) {}

void Map::addObstacle(const Position& pos) {
    if (isValidPosition(pos)) {
        std::lock_guard<std::mutex> lock(mutex);
        grid[pos.x][pos.y] = true;
        obstacleCacheDirty = true;  // 添加这行
    }
}

void Map::removeObstacle(const Position& pos) {
    if (isValidPosition(pos)) {
        std::lock_guard<std::mutex> lock(mutex);
        grid[pos.x][pos.y] = false;
        obstacleCacheDirty = true;  // 添加这行
    }
}

void Map::generateRandomObstacles(int count) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, MAP_SIZE - 1);

    std::lock_guard<std::mutex> lock(mutex);
    int generated = 0;

    while (generated < count) {
        Position pos(dis(gen), dis(gen));
        if (std::abs(pos.x - MAP_SIZE/2) > 200 || std::abs(pos.y - MAP_SIZE/2) > 200) {
            if (!grid[pos.x][pos.y]) {
                grid[pos.x][pos.y] = true;
                generated++;
            }
        }
    }
    obstacleCacheDirty = true;  // 添加这行
}

bool Map::isObstacle(const Position& pos) const {
    if (!isValidPosition(pos)) return true;
    // No lock needed for read-only access to vector<bool>
    return grid[pos.x][pos.y];
}

bool Map::isPositionOccupied(const Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex);
    int hash = pos.x * MAP_SIZE + pos.y;
    return robotPositions.find(hash) != robotPositions.end();
}

bool Map::isPositionValid(const Position& pos) const {
    return isValidPosition(pos) && !isObstacle(pos);
}

void Map::registerRobotPosition(int robotId, const Position& pos) {
    std::lock_guard<std::mutex> lock(mutex);
    int hash = pos.x * MAP_SIZE + pos.y;
    robotPositions[hash] = robotId;
    robotPosMap[robotId] = pos;

    int blockX = pos.x / BLOCK_SIZE;
    int blockY = pos.y / BLOCK_SIZE;
    if (blockX < GRID_SIZE && blockY < GRID_SIZE) {
        blockRobots[blockX][blockY].push_back(robotId);
    }
}

void Map::unregisterRobotPosition(int robotId, const Position& pos) {
    std::lock_guard<std::mutex> lock(mutex);
    int hash = pos.x * MAP_SIZE + pos.y;
    robotPositions.erase(hash);
    robotPosMap.erase(robotId);

    int blockX = pos.x / BLOCK_SIZE;
    int blockY = pos.y / BLOCK_SIZE;
    if (blockX < GRID_SIZE && blockY < GRID_SIZE) {
        auto& block = blockRobots[blockX][blockY];
        block.erase(std::remove(block.begin(), block.end(), robotId), block.end());
    }
}

Position Map::getRobotPosition(int robotId) const {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = robotPosMap.find(robotId);
    return it != robotPosMap.end() ? it->second : Position(-1, -1);
}

int Map::getBlockIndex(const Position& pos) const {
    int blockX = pos.x / BLOCK_SIZE;
    int blockY = pos.y / BLOCK_SIZE;
    return blockX * GRID_SIZE + blockY;
}

std::vector<int> Map::getOccupiedRobotsInBlock(const Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex);
    int blockX = pos.x / BLOCK_SIZE;
    int blockY = pos.y / BLOCK_SIZE;
    if (blockX < GRID_SIZE && blockY < GRID_SIZE) {
        return blockRobots[blockX][blockY];
    }
    return {};
}

bool Map::isBlockOccupied(const Position& pos) const {
    return !getOccupiedRobotsInBlock(pos).empty();
}

bool Map::isValidPosition(const Position& pos) const {
    return pos.x >= 0 && pos.x < MAP_SIZE && pos.y >= 0 && pos.y < MAP_SIZE;
}

// OPTIMIZED: Cache obstacles instead of regenerating every frame
std::vector<Position> Map::getObstacles() const {
    std::lock_guard<std::mutex> lock(mutex);

    if (!obstacleCacheDirty && obstacleCache) {
        return *obstacleCache;
    }

    auto obstacles = std::make_shared<std::vector<Position>>();
    obstacles->reserve(20000);

    for (int x = 0; x < MAP_SIZE; ++x) {
        for (int y = 0; y < MAP_SIZE; ++y) {
            if (grid[x][y]) {
                obstacles->emplace_back(x, y);
            }
        }
    }

    obstacleCache = obstacles;
    obstacleCacheDirty = false;
    return *obstacleCache;
}

std::vector<std::pair<int, Position>> Map::getRobotPositions() const {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::pair<int, Position>> result;
    result.reserve(robotPosMap.size());

    for (const auto& pair : robotPosMap) {
        result.push_back({pair.first, pair.second});
    }
    return result;
}