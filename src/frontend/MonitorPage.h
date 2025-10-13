#ifndef MONITORPAGE_H
#define MONITORPAGE_H

#include <QWidget>
#include <QTimer>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QMessageBox>
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

private:
    void setupUI();
    void drawMap(QPainter& painter);
    void drawRobots(QPainter& painter);
    void drawTasks(QPainter& painter);
    void checkSimulationComplete();

    Map* map;
    Coordinator* coordinator;
    int totalTasks;
    int completedTasks;

    QTimer* updateTimer;
    QPushButton* exitButton;
    QLabel* statusLabel;

    // Scaling factors for visualization
    static constexpr double SCALE_FACTOR = 0.05; // Scale 10000x10000 to fit in window
    static constexpr int WINDOW_SIZE = 800; // Display window size
    static constexpr int ROBOT_SIZE = 4;
    static constexpr int OBSTACLE_SIZE = 2;

    bool simulationComplete;
};

#endif // MONITORPAGE_H