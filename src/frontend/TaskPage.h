#ifndef TASKPAGE_H
#define TASKPAGE_H

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include "../backend/Coordinator.h"

class TaskPage : public QWidget {
    Q_OBJECT

public:
    TaskPage(Map* map, Coordinator* coordinator, QWidget *parent = nullptr);

private slots:
    void onStartTaskClicked();

private:
    void setupUI();

    Map* map;
    Coordinator* coordinator;

    QLineEdit* robotCountEdit;
    QLineEdit* obstacleCountEdit;
    QLineEdit* taskCountEdit;
    QPushButton* startTaskButton;

    QLabel* robotLabel;
    QLabel* obstacleLabel;
    QLabel* taskLabel;
};

#endif // TASKPAGE_H