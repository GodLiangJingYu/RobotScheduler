#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include "../backend/Coordinator.h"
#include "../database/DatabaseManager.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onBeginScheduleClicked();

private:
    void setupUI();

    QWidget* centralWidget;
    QPushButton* beginScheduleButton;

    // Core components
    std::unique_ptr<Map> map;
    std::unique_ptr<Coordinator> coordinator;
    std::unique_ptr<DatabaseManager> dbManager;
};

#endif // MAINWINDOW_H