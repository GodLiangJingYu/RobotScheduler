#include "RobotPathFinder.h"
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

RobotPathFinder::RobotPathFinder(const Grid& grid) : grid(grid) {
}

double RobotPathFinder::heuristic(Point a, Point b) const {
    // Manhattan distance
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

std::vector<RobotPathFinder::Point> RobotPathFinder::getNeighbors(Point point) const {
    std::vector<Point> neighbors;
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};
    
    for (int i = 0; i < 4; ++i) {
        int nx = point.first + dx[i];
        int ny = point.second + dy[i];
        
        if (grid.isValid(nx, ny) && !grid.isObstacle(nx, ny)) {
            neighbors.push_back({nx, ny});
        }
    }
    
    return neighbors;
}

RobotPathFinder::Path RobotPathFinder::findPath(Point start, Point goal) {
    // Priority queue for A* algorithm
    auto cmp = [](const std::pair<double, Point>& a, const std::pair<double, Point>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<double, Point>, 
                       std::vector<std::pair<double, Point>>, 
                       decltype(cmp)> openSet(cmp);
    
    std::map<Point, Point> cameFrom;
    std::map<Point, double> gScore;
    std::map<Point, double> fScore;
    
    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push({fScore[start], start});
    
    while (!openSet.empty()) {
        Point current = openSet.top().second;
        openSet.pop();
        
        if (current == goal) {
            // Reconstruct path
            Path path;
            Point p = goal;
            while (cameFrom.find(p) != cameFrom.end()) {
                path.push_back(p);
                p = cameFrom[p];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        for (const Point& neighbor : getNeighbors(current)) {
            double tentativeGScore = gScore[current] + 1.0;
            
            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal);
                openSet.push({fScore[neighbor], neighbor});
            }
        }
    }
    
    // No path found
    return Path();
}
