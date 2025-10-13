#ifndef GRID_H
#define GRID_H

#include <vector>

/**
 * Grid class represents a 2D grid for robot navigation
 */
class Grid {
public:
    Grid(int width, int height);
    
    int getWidth() const;
    int getHeight() const;
    
    bool isObstacle(int x, int y) const;
    void setObstacle(int x, int y, bool obstacle);
    
    bool isValid(int x, int y) const;
    
private:
    int width;
    int height;
    std::vector<std::vector<bool>> obstacles;
};

#endif // GRID_H
