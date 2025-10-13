#include "Grid.h"

Grid::Grid(int width, int height) 
    : width(width), height(height), obstacles(height, std::vector<bool>(width, false)) {
}

int Grid::getWidth() const {
    return width;
}

int Grid::getHeight() const {
    return height;
}

bool Grid::isObstacle(int x, int y) const {
    if (!isValid(x, y)) {
        return true;
    }
    return obstacles[y][x];
}

void Grid::setObstacle(int x, int y, bool obstacle) {
    if (isValid(x, y)) {
        obstacles[y][x] = obstacle;
    }
}

bool Grid::isValid(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}
