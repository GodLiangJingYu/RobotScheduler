#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "../core/Map.h"
#include "../core/Robot.h"
#include <vector>
#include <queue>
#include <unordered_map>

struct Node {
    Position pos;
    int g, h;
    Position parent;

    Node(const Position& p, int g_cost = 0, int h_cost = 0)
        : pos(p), g(g_cost), h(h_cost), parent(-1, -1) {}

    int f() const { return g + h; }
    bool operator>(const Node& other) const { return f() > other.f(); }
};

class PathPlanner {
public:
    PathPlanner(Map* map);

    std::vector<Position> findPath(const Position& start, const Position& end);
    std::vector<Position> findPathAvoidingRobots(const Position& start,
                                               const Position& end,
                                               int robotId);

private:
    Map* map;

    int heuristic(const Position& a, const Position& b) const;
    std::vector<Position> getNeighbors(const Position& pos) const;
    std::vector<Position> reconstructPath(const std::unordered_map<Position, Position>& cameFrom,
                                        const Position& start, const Position& end);
};

#endif // PATHPLANNER_H