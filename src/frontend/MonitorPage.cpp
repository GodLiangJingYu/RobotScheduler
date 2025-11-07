#include "MonitorPage.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTimer>
#include <QPainter>
#include <QMessageBox>
#include <QSlider>
#include <QProgressBar>
#include <random>

MonitorPage::MonitorPage(Map* map, Coordinator* coordinator, int totalTasks, QWidget *parent)
    : QWidget(parent),
      map(map),
      coordinator(coordinator),
      totalTasks(totalTasks),
      completedTasks(0),
      simulationComplete(false),
      isPaused(false),
      updateInterval(50) {

    updateTimer = new QTimer(this);
    exitButton = new QPushButton("Exit", this);
    pauseResumeButton = new QPushButton("Pause", this);
    statusLabel = new QLabel("Running...", this);
    robotCountLabel = new QLabel("Robots: 0", this);
    pendingTaskLabel = new QLabel("Pending Tasks: 0", this);
    completedTaskLabel = new QLabel("Completed Tasks: 0", this);
    speedLabel = new QLabel("Speed: 1x", this);
    progressBar = new QProgressBar(this);
    speedSlider = new QSlider(Qt::Horizontal, this);

    setupUI();

    coordinator->startScheduling();

    connect(updateTimer, &QTimer::timeout, this, &MonitorPage::updateDisplay);
    updateTimer->start(updateInterval);

    // Task generator: 5 tasks every 100ms = 50 tasks/second
    QTimer* taskGenerator = new QTimer(this);
    connect(taskGenerator, &QTimer::timeout, [this, map, coordinator]() {
        if (!simulationComplete && coordinator->getPendingTaskCount() < 50) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(1000, Map::MAP_SIZE - 1000);

            Position start(dis(gen), dis(gen));
            Position end(dis(gen), dis(gen));

            if (!map->isObstacle(start) && !map->isObstacle(end)) {
                coordinator->createTask(start, end);
            }
        }
    });
    taskGenerator->start(200);  // Every 200ms, 1 task = 5 tasks/second

    setWindowTitle("Robot Monitoring System");
    resize(WINDOW_SIZE, WINDOW_SIZE + INFO_PANEL_HEIGHT + CONTROL_PANEL_HEIGHT);
}

MonitorPage::~MonitorPage() {
    if (coordinator) {
        coordinator->stopScheduling();
    }
    updateTimer->stop();
}

void MonitorPage::setupUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(5);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    QWidget* infoPanel = new QWidget(this);
    infoPanel->setMaximumHeight(INFO_PANEL_HEIGHT);
    infoPanel->setStyleSheet("QWidget { background-color: #f0f0f0; border-radius: 5px; }");
    QGridLayout* infoLayout = new QGridLayout(infoPanel);

    QFont labelFont;
    labelFont.setPointSize(10);
    labelFont.setBold(true);

    statusLabel->setFont(labelFont);
    statusLabel->setStyleSheet("QLabel { color: #2c3e50; padding: 5px; }");
    robotCountLabel->setFont(labelFont);
    robotCountLabel->setStyleSheet("QLabel { color: #27ae60; padding: 5px; }");
    pendingTaskLabel->setFont(labelFont);
    pendingTaskLabel->setStyleSheet("QLabel { color: #e74c3c; padding: 5px; }");
    completedTaskLabel->setFont(labelFont);
    completedTaskLabel->setStyleSheet("QLabel { color: #3498db; padding: 5px; }");

    infoLayout->addWidget(statusLabel, 0, 0, 1, 2);
    infoLayout->addWidget(robotCountLabel, 1, 0);
    infoLayout->addWidget(pendingTaskLabel, 1, 1);
    infoLayout->addWidget(completedTaskLabel, 2, 0);

    progressBar->setMaximum(totalTasks);
    progressBar->setValue(0);
    progressBar->setTextVisible(true);
    progressBar->setFormat("Progress: %v / %m (%p%)");
    progressBar->setStyleSheet(
        "QProgressBar { border: 2px solid #bdc3c7; border-radius: 5px; text-align: center; }"
        "QProgressBar::chunk { background-color: #3498db; }"
    );
    infoLayout->addWidget(progressBar, 2, 1);

    mainLayout->addWidget(infoPanel);
    mainLayout->addStretch();

    QWidget* controlPanel = new QWidget(this);
    controlPanel->setMaximumHeight(CONTROL_PANEL_HEIGHT);
    controlPanel->setStyleSheet("QWidget { background-color: #ecf0f1; border-radius: 5px; }");
    QHBoxLayout* controlLayout = new QHBoxLayout(controlPanel);

    QLabel* speedTitleLabel = new QLabel("Simulation Speed:", this);
    speedLabel->setStyleSheet("QLabel { color: #2c3e50; font-weight: bold; }");
    controlLayout->addWidget(speedTitleLabel);

    speedSlider->setMinimum(1);
    speedSlider->setMaximum(10);
    speedSlider->setValue(5);
    speedSlider->setTickPosition(QSlider::TicksBelow);
    speedSlider->setTickInterval(1);
    connect(speedSlider, &QSlider::valueChanged, this, &MonitorPage::onSpeedChanged);
    controlLayout->addWidget(speedSlider);
    controlLayout->addWidget(speedLabel);

    pauseResumeButton->setStyleSheet(
        "QPushButton { background-color: #f39c12; color: white; font-weight: bold; "
        "padding: 8px 20px; border-radius: 5px; }"
        "QPushButton:hover { background-color: #e67e22; }"
    );
    connect(pauseResumeButton, &QPushButton::clicked, this, &MonitorPage::onPauseResumeClicked);
    controlLayout->addWidget(pauseResumeButton);

    exitButton->setStyleSheet(
        "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; "
        "padding: 8px 20px; border-radius: 5px; }"
        "QPushButton:hover { background-color: #c0392b; }"
    );
    connect(exitButton, &QPushButton::clicked, this, &MonitorPage::onExitClicked);
    controlLayout->addWidget(exitButton);

    mainLayout->addWidget(controlPanel);
}

void MonitorPage::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int mapY = INFO_PANEL_HEIGHT + 20;
    int mapHeight = height() - INFO_PANEL_HEIGHT - CONTROL_PANEL_HEIGHT - 30;

    painter.fillRect(10, mapY, WINDOW_SIZE - 20, mapHeight, QColor(250, 250, 250));
    painter.setPen(QPen(QColor(200, 200, 200), 2));
    painter.drawRect(10, mapY, WINDOW_SIZE - 20, mapHeight);

    painter.setClipRect(10, mapY, WINDOW_SIZE - 20, mapHeight);
    painter.translate(10, mapY);

    drawGrid(painter);
    drawMap(painter);
    drawRobots(painter);
    drawTasks(painter);

    painter.resetTransform();
    drawLegend(painter);
}

void MonitorPage::drawGrid(QPainter& painter) {
    painter.setPen(QPen(QColor(230, 230, 230), 1));

    int gridSpacing = 50;
    for (int x = 0; x < WINDOW_SIZE; x += gridSpacing) {
        painter.drawLine(x, 0, x, WINDOW_SIZE);
    }
    for (int y = 0; y < WINDOW_SIZE; y += gridSpacing) {
        painter.drawLine(0, y, WINDOW_SIZE, y);
    }
}

void MonitorPage::drawMap(QPainter& painter) {
    auto obstacles = map->getObstacles();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(80, 80, 80));

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

        auto path = robot->getPath();
        if (!path.empty() && path.size() > 1) {
            painter.setPen(QPen(QColor(150, 150, 150, 100), 1, Qt::DashLine));
            for (size_t i = 1; i < std::min(path.size(), size_t(50)); ++i) {
                int x1 = path[i-1].x * SCALE_FACTOR;
                int y1 = path[i-1].y * SCALE_FACTOR;
                int x2 = path[i].x * SCALE_FACTOR;
                int y2 = path[i].y * SCALE_FACTOR;
                painter.drawLine(x1, y1, x2, y2);
            }
            painter.setPen(Qt::NoPen);
        }

        QColor robotColor;
        switch (robot->getState()) {
            case RobotState::IDLE:
                robotColor = QColor(52, 152, 219);
                break;
            case RobotState::MOVING_TO_TASK:
                robotColor = QColor(46, 204, 113);
                break;
            case RobotState::EXECUTING_TASK:
                robotColor = QColor(241, 196, 15);
                break;
            case RobotState::MOVING_TO_END:
                robotColor = QColor(155, 89, 182);
                break;
            case RobotState::COMPLETED:
                robotColor = QColor(231, 76, 60);
                break;
        }

        painter.setBrush(robotColor);
        int x = pos.x * SCALE_FACTOR;
        int y = pos.y * SCALE_FACTOR;

        painter.drawEllipse(QPointF(x, y), ROBOT_SIZE/2.0, ROBOT_SIZE/2.0);

        painter.setPen(QPen(Qt::white, 1));
        painter.drawEllipse(QPointF(x, y), ROBOT_SIZE/2.0 - 2, ROBOT_SIZE/2.0 - 2);
        painter.setPen(Qt::NoPen);
    }
}

void MonitorPage::drawTasks(QPainter& painter) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(231, 76, 60, 150));
}

void MonitorPage::drawLegend(QPainter& painter) {
    int legendX = WINDOW_SIZE - 180;
    int legendY = INFO_PANEL_HEIGHT + 30;

    painter.fillRect(legendX, legendY, 160, 150, QColor(255, 255, 255, 230));
    painter.setPen(QPen(QColor(100, 100, 100), 1));
    painter.drawRect(legendX, legendY, 160, 150);

    QFont font = painter.font();
    font.setPointSize(8);
    font.setBold(true);
    painter.setFont(font);
    painter.setPen(Qt::black);
    painter.drawText(legendX + 10, legendY + 15, "Robot States:");

    int yOffset = legendY + 35;
    QList<QPair<QString, QColor>> states = {
        {"Idle", QColor(52, 152, 219)},
        {"Moving to Task", QColor(46, 204, 113)},
        {"Executing Task", QColor(241, 196, 15)},
        {"Moving to End", QColor(155, 89, 182)},
        {"Completed", QColor(231, 76, 60)}
    };

    for (const auto& state : states) {
        painter.setBrush(state.second);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(legendX + 15, yOffset - 5, 10, 10);

        font.setBold(false);
        painter.setFont(font);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 35, yOffset + 5, state.first);
        yOffset += 20;
    }
}

void MonitorPage::updateDisplay() {
    if (isPaused) return;

    int pending = coordinator->getPendingTaskCount();
    int completed = coordinator->getCompletedTaskCount();
    int robotCount = coordinator->getActiveRobotCount();

    statusLabel->setText(QString("Status: Running"));
    robotCountLabel->setText(QString("Robots: %1").arg(robotCount));
    pendingTaskLabel->setText(QString("Pending Tasks: %1").arg(pending));
    completedTaskLabel->setText(QString("Completed Tasks: %1").arg(completed));
    progressBar->setValue(completed);

    checkSimulationComplete();
    update();
}

void MonitorPage::checkSimulationComplete() {
    // Only mark as complete if we've actually completed some tasks
    if (!simulationComplete &&
        coordinator->getCompletedTaskCount() > 0 &&
        coordinator->isSimulationComplete() &&
        coordinator->getPendingTaskCount() == 0) {
        simulationComplete = true;
        updateTimer->stop();

        statusLabel->setText("Status: Completed!");
        statusLabel->setStyleSheet("QLabel { color: #27ae60; padding: 5px; font-weight: bold; }");

        QMessageBox::information(this, "Task Completed",
                               QString("All tasks completed!\nTotal: %1 tasks").arg(totalTasks));
        }
}

void MonitorPage::onPauseResumeClicked() {
    isPaused = !isPaused;
    if (isPaused) {
        pauseResumeButton->setText("Resume");
        pauseResumeButton->setStyleSheet(
            "QPushButton { background-color: #27ae60; color: white; font-weight: bold; "
            "padding: 8px 20px; border-radius: 5px; }"
            "QPushButton:hover { background-color: #229954; }"
        );
        statusLabel->setText("Status: Paused");
    } else {
        pauseResumeButton->setText("Pause");
        pauseResumeButton->setStyleSheet(
            "QPushButton { background-color: #f39c12; color: white; font-weight: bold; "
            "padding: 8px 20px; border-radius: 5px; }"
            "QPushButton:hover { background-color: #e67e22; }"
        );
        statusLabel->setText("Status: Running");
    }
}

void MonitorPage::onSpeedChanged(int value) {
    double speed = value / 5.0;
    speedLabel->setText(QString("Speed: %1x").arg(speed, 0, 'f', 1));
    updateInterval = static_cast<int>(50 / speed);
    updateTimer->setInterval(updateInterval);
}

void MonitorPage::onExitClicked() {
    if (coordinator) {
        coordinator->stopScheduling();
    }
    updateTimer->stop();
    close();
}