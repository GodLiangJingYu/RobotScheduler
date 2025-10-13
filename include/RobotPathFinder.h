#ifndef ROBOT_PATH_FINDER_H
#define ROBOT_PATH_FINDER_H

#include "Grid.h"
#include <vector>
#include <utility>

/**
 * RobotPathFinder implements A* algorithm for robot pathfinding
 */
class RobotPathFinder {
public:
    using Point = std::pair<int, int>;
    using Path = std::vector<Point>;
    
    RobotPathFinder(const Grid& grid);
    
    /**
     * Find path from start to goal using A* algorithm
     * @return Path as vector of points, empty if no path found
     */
    Path findPath(Point start, Point goal);
    
private:
    const Grid& grid;
    
    double heuristic(Point a, Point b) const;
    std::vector<Point> getNeighbors(Point point) const;
};

#endif // ROBOT_PATH_FINDER_H
