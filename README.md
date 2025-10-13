# RobotFindWaySystem
机器人路径自动规划 / Robot Path Automatic Planning

## 项目简介 / Project Overview

本项目实现了一个基于A*算法的机器人路径规划系统，可以在带障碍物的网格地图中自动寻找从起点到终点的最优路径。

This project implements a robot path planning system based on the A* algorithm, which can automatically find the optimal path from start to goal in a grid map with obstacles.

## 功能特性 / Features

- ✅ 使用A*算法进行路径规划 / A* algorithm for pathfinding
- ✅ 支持自定义网格大小 / Customizable grid size
- ✅ 支持添加障碍物 / Support for adding obstacles
- ✅ 路径可视化显示 / Path visualization
- ✅ CMake构建系统 / CMake build system
- ✅ 支持CLion开发 / CLion IDE support

## 开发环境要求 / Requirements

- CMake 3.10 或更高版本 / CMake 3.10 or higher
- C++11 或更高版本的编译器 / C++11 or higher compiler
- CLion IDE (推荐 / Recommended)

## 在CLion中打开项目 / Opening in CLion

1. 启动CLion / Launch CLion
2. 选择 "Open" / Select "Open"
3. 选择项目根目录（包含CMakeLists.txt的目录）/ Select project root directory (containing CMakeLists.txt)
4. CLion会自动检测CMake配置并加载项目 / CLion will automatically detect CMake configuration and load the project

## 编译和运行 / Build and Run

### 使用CLion / Using CLion

1. CLion会自动配置CMake项目 / CLion will automatically configure the CMake project
2. 点击运行按钮（绿色三角形）或按 Shift+F10 / Click the Run button (green triangle) or press Shift+F10
3. 程序将编译并运行 / The program will compile and run

### 使用命令行 / Using Command Line

```bash
# 创建构建目录 / Create build directory
mkdir build
cd build

# 配置CMake / Configure CMake
cmake ..

# 编译 / Build
make

# 运行 / Run
./bin/RobotFindWaySystem
```

## 项目结构 / Project Structure

```
RobotFindWaySystem/
├── CMakeLists.txt          # CMake配置文件 / CMake configuration
├── include/                # 头文件目录 / Header files
│   ├── Grid.h             # 网格类定义 / Grid class
│   └── RobotPathFinder.h  # 路径查找器类 / Path finder class
├── src/                    # 源代码目录 / Source files
│   ├── main.cpp           # 主程序 / Main program
│   ├── Grid.cpp           # 网格类实现 / Grid implementation
│   └── RobotPathFinder.cpp # 路径查找实现 / Path finder implementation
└── README.md              # 本文件 / This file
```

## 核心类说明 / Core Classes

### Grid
网格类，用于表示机器人导航的2D地图。
Grid class represents the 2D map for robot navigation.

- `Grid(int width, int height)` - 创建指定大小的网格 / Create grid with specified size
- `setObstacle(int x, int y, bool obstacle)` - 设置障碍物 / Set obstacle
- `isObstacle(int x, int y)` - 检查是否为障碍物 / Check if obstacle
- `isValid(int x, int y)` - 检查坐标是否有效 / Check if coordinates are valid

### RobotPathFinder
路径查找器，实现A*算法。
Path finder implementing A* algorithm.

- `RobotPathFinder(const Grid& grid)` - 构造函数 / Constructor
- `findPath(Point start, Point goal)` - 查找从起点到终点的路径 / Find path from start to goal

## 示例输出 / Example Output

```
=== Robot Path Finding System ===
使用A*算法进行机器人路径规划

Grid size: 10x10
Start position: (1, 1)
Goal position: (8, 8)

查找路径中...
找到路径！长度: 15 步

Grid visualization:
S = Start, G = Goal, * = Path, # = Obstacle, . = Empty

. . . . . . . . . . 
. S * # . . . . . . 
. . * # . . . . . . 
...
```

## 自定义开发 / Customization

您可以修改 `main.cpp` 来：
You can modify `main.cpp` to:

- 改变网格大小 / Change grid size
- 添加不同的障碍物模式 / Add different obstacle patterns
- 修改起点和终点位置 / Modify start and goal positions
- 实现其他路径规划算法 / Implement other pathfinding algorithms

## 许可证 / License

本项目为开源项目。
This project is open source.
