#include "PathPlanner.h"
#include <algorithm>
#include <cmath>

PathPlanner::PathPlanner(Map* map) : map(map) {}

std::vector<Position> PathPlanner::findPath(const Position& start, const Position& end) {
    if (start == end) return {start};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<int, int> closedSet; // pos hash -> g cost
    std::unordered_map<Position, Position> cameFrom;

    openSet.push(Node(start, 0, heuristic(start, end)));

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        int currentHash = current.pos.x * Map::MAP_SIZE + current.pos.y;
        if (closedSet.find(currentHash) != closedSet.end()) continue;

        closedSet[currentHash] = current.g;

        if (current.pos == end) {
            return reconstructPath(cameFrom, start, end);
        }

        for (const auto& neighbor : getNeighbors(current.pos)) {
            int neighborHash = neighbor.x * Map::MAP_SIZE + neighbor.y;
            if (closedSet.find(neighborHash) != closedSet.end()) continue;

            int tentativeG = current.g + 1;
            openSet.push(Node(neighbor, tentativeG, heuristic(neighbor, end)));
            cameFrom[neighbor] = current.pos;
        }
    }

    return {}; // No path found
}

std::vector<Position> PathPlanner::findPathAvoidingRobots(const Position& start,
                                                        const Position& end,
                                                        int robotId) {
    if (start == end) return {start};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<int, int> closedSet;
    std::unordered_map<Position, Position> cameFrom;

    openSet.push(Node(start, 0, heuristic(start, end)));

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        int currentHash = current.pos.x * Map::MAP_SIZE + current.pos.y;
        if (closedSet.find(currentHash) != closedSet.end()) continue;

        closedSet[currentHash] = current.g;

        if (current.pos == end) {
            return reconstructPath(cameFrom, start, end);
        }

        for (const auto& neighbor : getNeighbors(current.pos)) {
            if (map->isPositionOccupied(neighbor) &&
                map->getRobotPosition(robotId) != neighbor) {
                continue; // Skip occupied positions (except current robot's position)
            }

            int neighborHash = neighbor.x * Map::MAP_SIZE + neighbor.y;
            if (closedSet.find(neighborHash) != closedSet.end()) continue;

            int tentativeG = current.g + 1;
            openSet.push(Node(neighbor, tentativeG, heuristic(neighbor, end)));
            cameFrom[neighbor] = current.pos;
        }
    }

    return {}; // No path found
}

int PathPlanner::heuristic(const Position& a, const Position& b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}

std::vector<Position> PathPlanner::getNeighbors(const Position& pos) const {
    std::vector<Position> neighbors;
    std::vector<Position> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0} // 4-directional movement
    };

    for (const auto& dir : directions) {
        Position neighbor(pos.x + dir.x, pos.y + dir.y);
        if (map->isValidPosition(neighbor) && !map->isObstacle(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

std::vector<Position> PathPlanner::reconstructPath(const std::unordered_map<Position, Position>& cameFrom,
                                                 const Position& start,
                                                 const Position& end) {
    std::vector<Position> path;
    Position current = end;

    while (current != start) {
        path.push_back(current);
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) break;
        current = it->second;
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}