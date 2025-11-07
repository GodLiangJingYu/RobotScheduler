#ifndef MONITORPAGE_H
#define MONITORPAGE_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QProgressBar>
#include <QSlider>
#include <QGridLayout>
#include "../backend/Coordinator.h"

class MonitorPage : public QWidget {
    Q_OBJECT

public:
    MonitorPage(Map* map, Coordinator* coordinator, int totalTasks, QWidget *parent = nullptr);
    ~MonitorPage();

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void updateDisplay();
    void onExitClicked();
    void onPauseResumeClicked();
    void onSpeedChanged(int value);

private:
    void setupUI();
    void drawMap(QPainter& painter);
    void drawRobots(QPainter& painter);
    void drawTasks(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawLegend(QPainter& painter);
    void checkSimulationComplete();

    Map* map;
    Coordinator* coordinator;
    int totalTasks;
    int completedTasks;

    QTimer* updateTimer;
    QPushButton* exitButton;
    QPushButton* pauseResumeButton;
    QLabel* statusLabel;
    QLabel* robotCountLabel;
    QLabel* pendingTaskLabel;
    QLabel* completedTaskLabel;
    QLabel* speedLabel;
    QProgressBar* progressBar;
    QSlider* speedSlider;

    static constexpr double SCALE_FACTOR = 0.05;
    static constexpr int WINDOW_SIZE = 900;
    static constexpr int ROBOT_SIZE = 6;
    static constexpr int OBSTACLE_SIZE = 6;
    static constexpr int INFO_PANEL_HEIGHT = 120;
    static constexpr int CONTROL_PANEL_HEIGHT = 60;

    bool simulationComplete;
    bool isPaused;
    int updateInterval;
};

#endif // MONITORPAGE_H