#include "Grid.h"
#include "RobotPathFinder.h"
#include <iostream>

void printGrid(const Grid& grid, const RobotPathFinder::Path& path, 
               RobotPathFinder::Point start, RobotPathFinder::Point goal) {
    std::cout << "\nGrid visualization:\n";
    std::cout << "S = Start, G = Goal, * = Path, # = Obstacle, . = Empty\n\n";
    
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            RobotPathFinder::Point current = {x, y};
            
            if (current == start) {
                std::cout << "S ";
            } else if (current == goal) {
                std::cout << "G ";
            } else if (grid.isObstacle(x, y)) {
                std::cout << "# ";
            } else {
                bool isPath = false;
                for (const auto& p : path) {
                    if (p == current) {
                        isPath = true;
                        break;
                    }
                }
                std::cout << (isPath ? "* " : ". ");
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "=== Robot Path Finding System ===" << std::endl;
    std::cout << "使用A*算法进行机器人路径规划\n" << std::endl;
    
    // Create a 10x10 grid
    Grid grid(10, 10);
    
    // Add some obstacles
    grid.setObstacle(3, 1, true);
    grid.setObstacle(3, 2, true);
    grid.setObstacle(3, 3, true);
    grid.setObstacle(3, 4, true);
    grid.setObstacle(3, 5, true);
    grid.setObstacle(6, 4, true);
    grid.setObstacle(6, 5, true);
    grid.setObstacle(6, 6, true);
    grid.setObstacle(6, 7, true);
    
    // Define start and goal positions
    RobotPathFinder::Point start = {1, 1};
    RobotPathFinder::Point goal = {8, 8};
    
    std::cout << "Grid size: " << grid.getWidth() << "x" << grid.getHeight() << std::endl;
    std::cout << "Start position: (" << start.first << ", " << start.second << ")" << std::endl;
    std::cout << "Goal position: (" << goal.first << ", " << goal.second << ")" << std::endl;
    
    // Create path finder
    RobotPathFinder pathFinder(grid);
    
    // Find path
    std::cout << "\n查找路径中..." << std::endl;
    RobotPathFinder::Path path = pathFinder.findPath(start, goal);
    
    if (path.empty()) {
        std::cout << "未找到路径！" << std::endl;
    } else {
        std::cout << "找到路径！长度: " << path.size() << " 步" << std::endl;
        std::cout << "\n路径坐标:" << std::endl;
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << "步骤 " << i << ": (" << path[i].first << ", " << path[i].second << ")";
            if (i < path.size() - 1) {
                std::cout << " -> ";
            }
            if ((i + 1) % 5 == 0) {
                std::cout << "\n";
            }
        }
        std::cout << std::endl;
        
        printGrid(grid, path, start, goal);
    }
    
    std::cout << "程序执行完成！" << std::endl;
    return 0;
}
