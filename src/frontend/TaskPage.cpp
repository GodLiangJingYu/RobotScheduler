#include "TaskPage.h"
#include "MonitorPage.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QIntValidator>
#include <QMessageBox>
#include <random>
#include <ctime>


TaskPage::TaskPage(Map* map, Coordinator* coordinator, QWidget *parent)
    : QWidget(parent), map(map), coordinator(coordinator),
      robotCountEdit(new QLineEdit(this)),
      obstacleCountEdit(new QLineEdit(this)),
      taskCountEdit(new QLineEdit(this)),
      startTaskButton(new QPushButton("开始任务", this)),
      robotLabel(new QLabel("机器人数量 (1000-3000):", this)),
      obstacleLabel(new QLabel("障碍物数量 (5000-20000):", this)),
      taskLabel(new QLabel("任务数量 (60-15000):", this)) {

    setupUI();
}

void TaskPage::setupUI() {
    setWindowTitle("任务配置");
    resize(400, 300);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QHBoxLayout* robotLayout = new QHBoxLayout();
    robotLayout->addWidget(robotLabel);
    robotLayout->addWidget(robotCountEdit);
    robotCountEdit->setValidator(new QIntValidator(1000, 3000, this));

    QHBoxLayout* obstacleLayout = new QHBoxLayout();
    obstacleLayout->addWidget(obstacleLabel);
    obstacleLayout->addWidget(obstacleCountEdit);
    obstacleCountEdit->setValidator(new QIntValidator(5000, 20000, this));

    QHBoxLayout* taskLayout = new QHBoxLayout();
    taskLayout->addWidget(taskLabel);
    taskLayout->addWidget(taskCountEdit);
    taskCountEdit->setValidator(new QIntValidator(60, 15000, this));

    mainLayout->addLayout(robotLayout);
    mainLayout->addLayout(obstacleLayout);
    mainLayout->addLayout(taskLayout);
    mainLayout->addWidget(startTaskButton);

    connect(startTaskButton, &QPushButton::clicked,
            this, &TaskPage::onStartTaskClicked);
}

void TaskPage::onStartTaskClicked() {
    bool ok1, ok2, ok3;
    int robotCount = robotCountEdit->text().toInt(&ok1);
    int obstacleCount = obstacleCountEdit->text().toInt(&ok2);
    int taskCount = taskCountEdit->text().toInt(&ok3);

    if (!ok1 || !ok2 || !ok3) {
        QMessageBox::warning(this, "输入错误", "请输入有效的数字");
        return;
    }

    if (robotCount < 1000 || robotCount > 3000 ||
        obstacleCount < 5000 || obstacleCount > 20000 ||
        taskCount < 60 || taskCount > 15000) {
        QMessageBox::warning(this, "输入错误", "参数超出范围");
        return;
    }

    // 生成障碍物
    map->generateRandomObstacles(obstacleCount);

    // 生成机器人
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(-100, 100);

    for (int i = 0; i < robotCount; ++i) {
        Position center(Map::MAP_SIZE/2, Map::MAP_SIZE/2);
        Position pos(center.x + dis(gen), center.y + dis(gen));

        RobotType type = static_cast<RobotType>(i % 4);
        auto robot = std::make_shared<Robot>(i, type, pos);
        coordinator->addRobot(robot);
    }

    // 创建监控页面
    MonitorPage* monitorPage = new MonitorPage(map, coordinator, taskCount, nullptr);
    monitorPage->setAttribute(Qt::WA_DeleteOnClose);
    monitorPage->show();

    QTimer::singleShot(100, this, [this]() {
        this->close();
    });
}