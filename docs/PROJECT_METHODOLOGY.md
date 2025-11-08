# RobotScheduler 项目方法与思路详解

## 一、项目概述

本项目是一个**大规模机器人任务调度与路径规划系统**，能够在10000×10000的网格地图上同时管理1000-3000个机器人，处理60-15000个任务，并避开5000-20000个障碍物。

### 核心特性
- 大规模并发：支持数千个机器人同时工作
- 实时任务分配：动态分配任务给最近的空闲机器人
- 智能路径规划：基于A*算法的避障路径规划
- 多线程架构：任务生成、调度、执行分离
- 可视化监控：实时显示机器人状态和任务进度

---

## 二、系统架构

### 2.1 整体架构设计

```
┌─────────────────────────────────────────────────────┐
│                   Frontend (Qt)                      │
│  ┌──────────────┐         ┌──────────────────┐     │
│  │  TaskPage    │────────▶│  MonitorPage     │     │
│  │  (参数配置)  │         │  (实时监控)      │     │
│  └──────────────┘         └──────────────────┘     │
└─────────────────┬───────────────────┬───────────────┘
                  │                   │
┌─────────────────▼───────────────────▼───────────────┐
│                   Backend                            │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────┐ │
│  │  Coordinator │  │ PathPlanner  │  │ Scheduler │ │
│  │  (协调器)    │  │ (路径规划)   │  │ (调度器)  │ │
│  └──────────────┘  └──────────────┘  └───────────┘ │
└─────────────────┬───────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────┐
│                      Core                            │
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐           │
│  │ Map  │  │Robot │  │Task  │  │Position│          │
│  └──────┘  └──────┘  └──────┘  └──────┘           │
└─────────────────────────────────────────────────────┘
```

### 2.2 核心模块说明

#### 1. **Map (地图管理)**
- 10000×10000格子的网格地图
- 分块管理：8×8格子为一个块（1250×1250块）
- 障碍物存储和检测
- 机器人位置注册和查询

#### 2. **Robot (机器人)**
- 四种类型：AGV、MIR、HUMANOID、QUADRUPED
- 五种状态：IDLE、MOVING_TO_TASK、EXECUTING_TASK、MOVING_TO_END、COMPLETED
- 路径跟踪和移动
- 任务执行

#### 3. **Task (任务)**
- 包含起点和终点
- 记录分配时间和完成时间
- 状态追踪（未分配、已分配、已完成）

#### 4. **Coordinator (协调器)**
- 机器人管理
- 任务创建和分配
- 调度循环
- 路径规划接口

#### 5. **PathPlanner (路径规划器)**
- A*算法实现
- 曼哈顿距离启发式
- 障碍物避让
- 机器人避让

#### 6. **Scheduler (调度器)**
- 任务生成线程
- 任务执行监控线程
- 完成度检测

---

## 三、核心算法与方法

### 3.1 任务调度算法

#### 方法思路
采用**最近空闲机器人优先**策略：
1. 扫描所有未分配的任务
2. 为每个任务找到距离最近的空闲机器人
3. 分配任务并规划路径

#### 伪代码

```
算法: 任务分配 (assignTasks)
输入: 任务列表 tasks, 机器人列表 robots
输出: 更新后的任务和机器人状态

function assignTasks():
    // 1. 收集未分配任务
    unassignedTasks = []
    for each task in tasks:
        if not task.isAssigned and not task.isCompleted:
            unassignedTasks.add(task)
    
    // 2. 为每个未分配任务寻找最近的空闲机器人
    for each taskId in unassignedTasks:
        task = tasks[taskId]
        nearestRobot = findNearestIdleRobot(task.startPoint)
        
        if nearestRobot exists:
            assignTaskToRobot(taskId, nearestRobot.id)

function findNearestIdleRobot(targetPosition):
    nearestRobot = null
    minDistance = INFINITY
    
    for each robot in robots:
        if robot.state == IDLE:
            // 使用曼哈顿距离
            distance = |robot.x - target.x| + |robot.y - target.y|
            if distance < minDistance:
                minDistance = distance
                nearestRobot = robot
    
    return nearestRobot
```

---

### 3.2 A*路径规划算法

#### 方法思路
使用**A*算法**进行路径规划，特点：
- 启发式搜索：利用曼哈顿距离估算到终点的距离
- 优先队列：优先探索f值（g+h）最小的节点
- 四方向移动：上、下、左、右
- 避障：自动避开地图障碍物

#### 核心公式
- **g(n)**: 从起点到当前节点的实际代价
- **h(n)**: 从当前节点到终点的启发式估计（曼哈顿距离）
- **f(n) = g(n) + h(n)**: 总代价估计

#### 伪代码

```
算法: A*路径规划 (findPath)
输入: 起点 start, 终点 end, 地图 map
输出: 路径 path (Position列表)

function findPath(start, end):
    if start == end:
        return [start]
    
    // 1. 初始化
    openSet = PriorityQueue()  // 优先队列，按f值排序
    closedSet = HashMap()       // 已访问节点集合
    cameFrom = HashMap()        // 记录路径来源
    
    // 2. 起点加入开放集
    startNode = Node(start, g=0, h=heuristic(start, end))
    openSet.push(startNode)
    
    // 3. 主循环
    while not openSet.isEmpty():
        current = openSet.pop()  // 取出f值最小的节点
        
        // 检查是否已访问
        if current.pos in closedSet:
            continue
        
        closedSet[current.pos] = current.g
        
        // 到达终点
        if current.pos == end:
            return reconstructPath(cameFrom, start, end)
        
        // 4. 扩展邻居节点
        for each neighbor in getNeighbors(current.pos):
            if neighbor in closedSet:
                continue
            
            // 计算新的g值
            tentativeG = current.g + 1
            
            // 创建邻居节点并加入开放集
            neighborNode = Node(neighbor, 
                               g=tentativeG, 
                               h=heuristic(neighbor, end))
            openSet.push(neighborNode)
            cameFrom[neighbor] = current.pos
    
    return []  // 无路径

function heuristic(a, b):
    // 曼哈顿距离
    return |a.x - b.x| + |a.y - b.y|

function getNeighbors(pos):
    neighbors = []
    directions = [(0,1), (1,0), (0,-1), (-1,0)]  // 上右下左
    
    for each dir in directions:
        neighbor = Position(pos.x + dir.x, pos.y + dir.y)
        
        // 检查合法性
        if map.isValidPosition(neighbor) and 
           not map.isObstacle(neighbor):
            neighbors.add(neighbor)
    
    return neighbors

function reconstructPath(cameFrom, start, end):
    path = []
    current = end
    
    // 从终点回溯到起点
    while current != start:
        path.add(current)
        current = cameFrom[current]
    
    path.add(start)
    path.reverse()
    return path
```

---

### 3.3 机器人移动控制

#### 方法思路
机器人按照预规划的路径逐步移动：
1. 跟踪路径索引
2. 每次移动一步
3. 到达目标点时触发状态转换

#### 伪代码

```
算法: 机器人移动 (handleRobotMovement)
输入: 机器人列表 robots
输出: 更新后的机器人位置和状态

function handleRobotMovement():
    for each robot in robots:
        if robot.state == MOVING_TO_TASK:
            // 移动一步
            hasMoreSteps = robot.moveStep()
            
            if not hasMoreSteps and robot.isAtTarget():
                // 到达任务起点，开始前往终点
                task = getTask(robot.currentTaskId)
                robot.state = MOVING_TO_END
                robot.target = task.endPoint
                
                // 重新规划路径到终点
                path = pathPlanner.findPath(robot.position, task.endPoint)
                robot.setPath(path)
        
        else if robot.state == MOVING_TO_END:
            hasMoreSteps = robot.moveStep()
            
            if not hasMoreSteps and robot.isAtTarget():
                // 到达任务终点，完成任务
                task = getTask(robot.currentTaskId)
                completeTask(task.id)
                robot.state = IDLE
                robot.currentTaskId = -1

function moveStep():
    // 按路径索引移动
    if path is empty or pathIndex >= path.size():
        return false  // 无法继续移动
    
    position = path[pathIndex]
    pathIndex++
    
    return pathIndex < path.size()  // 是否还有更多步骤
```

---

### 3.4 地图分块管理

#### 方法思路
为了优化大规模地图的性能，采用**分块策略**：
- 地图划分为1250×1250个块
- 每个块大小为8×8格子
- 快速查询某个位置附近的机器人
- 减少碰撞检测的计算量

#### 伪代码

```
算法: 地图分块管理
常量: MAP_SIZE = 10000, BLOCK_SIZE = 8, GRID_SIZE = 1250

数据结构:
    grid[MAP_SIZE][MAP_SIZE]              // 障碍物网格
    robotPositions: HashMap<robotId, Position>  // 机器人位置
    blockRobots[GRID_SIZE][GRID_SIZE][]   // 每个块中的机器人ID列表

function registerRobotPosition(robotId, position):
    // 1. 记录机器人位置
    robotPositions[robotId] = position
    
    // 2. 计算所在块
    blockX = position.x / BLOCK_SIZE
    blockY = position.y / BLOCK_SIZE
    
    // 3. 将机器人ID加入对应块
    if blockX < GRID_SIZE and blockY < GRID_SIZE:
        blockRobots[blockX][blockY].add(robotId)

function unregisterRobotPosition(robotId, position):
    // 移除机器人位置记录
    robotPositions.remove(robotId)
    
    // 从块中移除
    blockX = position.x / BLOCK_SIZE
    blockY = position.y / BLOCK_SIZE
    
    if blockX < GRID_SIZE and blockY < GRID_SIZE:
        blockRobots[blockX][blockY].remove(robotId)

function getOccupiedRobotsInBlock(position):
    blockX = position.x / BLOCK_SIZE
    blockY = position.y / BLOCK_SIZE
    
    if blockX < GRID_SIZE and blockY < GRID_SIZE:
        return blockRobots[blockX][blockY]
    
    return []

function isPositionOccupied(position):
    // 快速检查该位置是否被占用
    return getRobotPosition(position) != null

function isPositionValid(position):
    // 检查位置是否合法（在边界内、无障碍、无机器人）
    return isValidPosition(position) and 
           not isObstacle(position) and 
           not isPositionOccupied(position)
```

---

### 3.5 多线程调度架构

#### 方法思路
采用**多线程并发**架构，提高系统效率：
1. **任务生成线程**：每秒生成新任务
2. **调度线程**：循环执行任务分配和机器人更新
3. **主线程**：UI渲染和用户交互

#### 线程安全保证
- 使用互斥锁（mutex）保护共享数据
- 条件变量（condition_variable）协调线程
- 原子操作（atomic）控制运行状态

#### 伪代码

```
算法: 协调器调度循环 (Coordinator)

数据结构:
    robots: List<Robot>                  // 机器人列表
    tasks: HashMap<taskId, Task>         // 任务映射
    robotsMutex: Mutex                   // 机器人互斥锁
    tasksMutex: Mutex                    // 任务互斥锁
    running: AtomicBool                  // 运行状态
    schedulerThread: Thread              // 调度线程

function startScheduling():
    if running.exchange(true):
        return  // 已经在运行
    
    // 启动调度线程
    schedulerThread = new Thread(schedulerLoop)

function stopScheduling():
    if not running.exchange(false):
        return  // 已经停止
    
    cv.notify_all()
    schedulerThread.join()

function schedulerLoop():
    while running:
        // 1. 更新所有机器人位置
        updateRobotPositions()
        
        // 2. 处理机器人移动
        handleRobotMovement()
        
        // 3. 分配未分配的任务
        assignTasks()
        
        // 4. 等待100ms
        sleep(100ms)
```

```
算法: 任务调度器 (Scheduler)

数据结构:
    totalTasks: int                // 总任务数
    generatedTasks: AtomicInt      // 已生成任务数
    running: AtomicBool            // 运行状态
    generatorThread: Thread        // 生成线程
    executorThread: Thread         // 执行监控线程

function start():
    if running.exchange(true):
        return
    
    generatorThread = new Thread(taskGenerator)
    executorThread = new Thread(taskExecutor)

function taskGenerator():
    while running and generatedTasks < totalTasks:
        // 1. 生成随机起点和终点
        start = Position(random(0, MAP_SIZE), random(0, MAP_SIZE))
        end = Position(random(0, MAP_SIZE), random(0, MAP_SIZE))
        
        // 2. 检查位置有效性
        if map.isValidPosition(start) and map.isValidPosition(end):
            coordinator.createTask(start, end)
            generatedTasks++
        
        // 3. 等待1秒
        sleep(1s)

function taskExecutor():
    while running:
        sleep(100ms)
        
        // 检查是否所有任务完成
        if generatedTasks >= totalTasks and 
           coordinator.getPendingTaskCount() == 0:
            complete = true
            break
```

---

## 四、性能优化策略

### 4.1 空间分块
- **目的**：减少碰撞检测和邻居查询的复杂度
- **方法**：将10000×10000地图分为1250×1250个块
- **效果**：查询时间从O(n)降低到O(n/k)，其中k是块大小

### 4.2 路径缓存
- **目的**：避免重复计算相同路径
- **方法**：机器人持有完整路径，按索引移动
- **效果**：移动时间复杂度O(1)

### 4.3 并发处理
- **目的**：充分利用多核CPU
- **方法**：任务生成、调度、UI分离为独立线程
- **效果**：提高系统吞吐量

### 4.4 优先队列优化
- **目的**：加速A*算法中的节点选择
- **方法**：使用二叉堆实现优先队列
- **效果**：节点操作时间复杂度O(log n)

---

## 五、系统工作流程

### 5.1 完整流程图

```
用户输入参数
    ↓
生成地图和机器人
    ↓
启动调度器 ─────┐
    │           │
    ↓           │
任务生成线程    │
(每秒生成任务)  │
    │           │
    ↓           │
协调器调度线程 ←┘
    │
    ├─→ 更新机器人位置
    ├─→ 处理机器人移动
    └─→ 分配未分配任务
         │
         ├─→ 查找最近空闲机器人
         ├─→ A*路径规划
         └─→ 分配任务和路径
              │
              ↓
         机器人执行任务
              │
              ├─→ 前往起点
              ├─→ 前往终点
              └─→ 完成任务
```

### 5.2 任务生命周期

```
1. 创建阶段
   - 生成随机起点和终点
   - 验证位置有效性
   - 创建Task对象
   - 状态：未分配

2. 分配阶段
   - 查找最近空闲机器人
   - 规划路径到起点
   - 分配给机器人
   - 状态：已分配

3. 执行阶段
   - 机器人移动到起点
   - 机器人移动到终点
   - 状态：执行中

4. 完成阶段
   - 标记任务完成
   - 释放机器人
   - 记录完成时间
   - 状态：已完成
```

---

## 六、关键数据结构

### 6.1 Position（位置）
```cpp
struct Position {
    int x, y;
    
    bool operator==(const Position& other)
    bool operator!=(const Position& other)
}
```

### 6.2 Robot（机器人）
```cpp
class Robot {
    int id;                    // 机器人ID
    RobotType type;            // 类型（AGV/MIR/HUMANOID/QUADRUPED）
    Position position;         // 当前位置
    RobotState state;          // 状态
    Position target;           // 目标位置
    vector<Position> path;     // 路径
    int currentTaskId;         // 当前任务ID
    int pathIndex;             // 路径索引
}
```

### 6.3 Task（任务）
```cpp
struct Task {
    int id;                    // 任务ID
    Position startPoint;       // 起点
    Position endPoint;         // 终点
    int assignedRobotId;       // 分配的机器人ID
    TimePoint createdTime;     // 创建时间
    TimePoint assignedTime;    // 分配时间
    TimePoint completedTime;   // 完成时间
    bool isCompleted;          // 是否完成
}
```

### 6.4 Node（A*节点）
```cpp
struct Node {
    Position pos;              // 位置
    int g;                     // 从起点到当前的实际代价
    int h;                     // 到终点的启发式估计
    Position parent;           // 父节点
    
    int f() { return g + h; }  // 总代价
}
```

---

## 七、技术栈总结

### 编程语言
- **C++17**: 核心逻辑实现

### 框架和库
- **Qt5**: 用户界面
- **STL**: 标准模板库（vector、map、queue等）
- **C++线程库**: thread、mutex、atomic

### 算法
- **A*算法**: 路径规划
- **贪心算法**: 最近机器人分配
- **分块管理**: 空间优化

### 并发模型
- **多线程**: 任务生成、调度、UI独立
- **互斥锁**: 数据保护
- **条件变量**: 线程协调

---

## 八、扩展性考虑

### 8.1 可扩展点
1. **机器人类型**：可增加新的机器人类型和特性
2. **任务优先级**：支持任务优先级队列
3. **路径算法**：可替换为Dijkstra、JPS等
4. **避障策略**：可添加动态避障
5. **通信协议**：可添加机器人间通信

### 8.2 性能瓶颈
1. **A*搜索**：大地图上可能较慢
   - 解决方案：分层路径规划、JPS算法
2. **任务分配**：O(n×m)复杂度
   - 解决方案：空间索引、K-D树
3. **UI渲染**：大量机器人时可能卡顿
   - 解决方案：视口裁剪、LOD技术

---

## 九、总结

本项目采用**分层架构**和**多线程并发**设计，实现了一个高性能的机器人任务调度系统。核心创新点包括：

1. **分块地图管理**：高效处理10000×10000大地图
2. **A*路径规划**：快速生成无碰撞路径
3. **贪心任务分配**：最近空闲机器人优先
4. **多线程架构**：任务生成和调度并行执行
5. **实时可视化**：直观展示系统运行状态

该系统可处理大规模机器人集群的协同作业，适用于仓储物流、智能制造等场景。
