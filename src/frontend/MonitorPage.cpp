#include "MonitorPage.h"
#include <QVBoxLayout>
#include <QTimer>
#include <QPainter>
#include <QMessageBox>
#include <random>

MonitorPage::MonitorPage(Map* map, Coordinator* coordinator, int totalTasks, QWidget *parent)
    : QWidget(parent),map(map), coordinator(coordinator), totalTasks(totalTasks),
      completedTasks(0), updateTimer(new QTimer(this)),
      exitButton(new QPushButton("退出",this)),
      statusLabel(new QLabel("运行中...",this)),
      simulationComplete(false) {

    setupUI();

    // Start the coordinator
    coordinator->startScheduling();

    // Start update timer - Changed from 500ms to 100ms for smoother display
    connect(updateTimer, &QTimer::timeout, this, &MonitorPage::updateDisplay);
    updateTimer->start(50);

    // Generate tasks periodically - Changed from 1000ms to 3000ms to slow down task generation
    QTimer* taskGenerator = new QTimer(this);
    connect(taskGenerator, &QTimer::timeout, [this, map, coordinator]() {
        if (!simulationComplete && coordinator->getPendingTaskCount() < 10) {
            // Generate random task
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, Map::MAP_SIZE - 1);

            Position start(dis(gen),dis(gen));
            Position end(dis(gen), dis(gen));

            // Ensure positions are valid
            if (map->isValidPosition(start) && map->isValidPosition(end)) {
                coordinator->createTask(start, end);
            }
        }
    });
    taskGenerator->start(2000);

    setWindowTitle("机器人监控系统");
    resize(WINDOW_SIZE, WINDOW_SIZE + 100);
}

MonitorPage::~MonitorPage() {
    if (coordinator) {
        coordinator->stopScheduling();
    }
    updateTimer->stop();
}

void MonitorPage::setupUI() {
    QVBoxLayout* layout = new QVBoxLayout(this);

    // Add status label
    layout->addWidget(statusLabel);

    // Add the map display area (this widget itself)
    layout->addStretch();

    // Add exit button
    layout->addWidget(exitButton);

    connect(exitButton, &QPushButton::clicked, this, &MonitorPage::onExitClicked);
}

void MonitorPage::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw background
    painter.fillRect(rect(), Qt::white);

    // Draw map components
    drawMap(painter);
    drawRobots(painter);
    drawTasks(painter);

    // Draw status info
    painter.setPen(Qt::black);
    painter.drawText(10, 20, QString("机器人数量: %1").arg(coordinator->getActiveRobotCount()));
    painter.drawText(10, 40, QString("待处理任务: %1").arg(coordinator->getPendingTaskCount()));
    painter.drawText(10, 60, QString("已完成任务: %1").arg(coordinator->getCompletedTaskCount()));
}

void MonitorPage::drawMap(QPainter& painter) {
    // Draw obstacles
    auto obstacles = map->getObstacles();
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::darkGray);

    for (const auto& obstacle : obstacles) {
        int x = obstacle.x * SCALE_FACTOR;
        int y = obstacle.y * SCALE_FACTOR;
        painter.drawRect(x, y, OBSTACLE_SIZE, OBSTACLE_SIZE);
    }
}

void MonitorPage::drawRobots(QPainter& painter) {
    auto robotPositions = map->getRobotPositions();
    painter.setPen(Qt::NoPen);

    for (const auto& robotPos : robotPositions) {
        int robotId = robotPos.first;
        Position pos = robotPos.second;

        auto robot = coordinator->getRobot(robotId);
        if (!robot) continue;

        // Draw robot path (new feature)
        auto path = robot->getPath();
        if (!path.empty()) {
            painter.setPen(QPen(QColor(200, 200, 200, 128), 1, Qt::DashLine));
            for (size_t i = 1; i < path.size(); ++i) {
                int x1 = path[i-1].x * SCALE_FACTOR;
                int y1 = path[i-1].y * SCALE_FACTOR;
                int x2 = path[i].x * SCALE_FACTOR;
                int y2 = path[i].y * SCALE_FACTOR;
                painter.drawLine(x1, y1, x2, y2);
            }
            painter.setPen(Qt::NoPen);
        }

        // Determine robot color based on state
        QColor robotColor;
        switch (robot->getState()) {
            case RobotState::IDLE:
                robotColor = Qt::blue;
                break;
            case RobotState::MOVING_TO_TASK:
                robotColor = Qt::green;
                break;
            case RobotState::EXECUTING_TASK:
                robotColor = Qt::yellow;
                break;
            case RobotState::MOVING_TO_END:
                robotColor = Qt::cyan;
                break;
            case RobotState::COMPLETED:
                robotColor = Qt::red;
                break;
        }

        painter.setBrush(robotColor);
        int x = pos.x * SCALE_FACTOR;
        int y = pos.y * SCALE_FACTOR;
        painter.drawEllipse(x - ROBOT_SIZE/2, y - ROBOT_SIZE/2, ROBOT_SIZE, ROBOT_SIZE);
    }
}

void MonitorPage::drawTasks(QPainter& painter) {
    // Draw tasks (start and end points)
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);

    // For simplicity, just draw a few recent tasks
    // In a real implementation, you'd want to track active tasks
    // This is a simplified visualization
}

void MonitorPage::updateDisplay() {
    // Update status label
    int pending = coordinator->getPendingTaskCount();
    int completed = coordinator->getCompletedTaskCount();
    statusLabel->setText(QString("运行中... 待处理: %1, 已完成: %2").arg(pending).arg(completed));

    // Check if simulation is complete
    checkSimulationComplete();

    // Repaint the widget
    update();
}

void MonitorPage::checkSimulationComplete() {
    if (!simulationComplete && coordinator->isSimulationComplete() &&
        coordinator->getPendingTaskCount() == 0) {
        simulationComplete = true;
        updateTimer->stop();

        QMessageBox::information(this, "任务完成",
                               QString("所有任务已完成！\n总共处理了 %1 个任务").arg(totalTasks));
    }
}

void MonitorPage::onExitClicked() {
    if (coordinator) {
        coordinator->stopScheduling();
    }
    updateTimer->stop();
    close();
}