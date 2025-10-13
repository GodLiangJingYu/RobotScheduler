#include "MainWindow.h"
#include "TaskPage.h"
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      centralWidget(new QWidget(this)),
      beginScheduleButton(new QPushButton("开始调度", this)),
      map(std::make_unique<Map>()),
      coordinator(std::make_unique<Coordinator>(map.get())),
      dbManager(std::make_unique<DatabaseManager>()) {

    setupUI();
    dbManager->initialize();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUI() {
    setWindowTitle("机器人调度系统");
    resize(800, 600);

    QVBoxLayout* layout = new QVBoxLayout(centralWidget);
    layout->addWidget(beginScheduleButton);
    layout->addStretch();

    setCentralWidget(centralWidget);

    connect(beginScheduleButton, &QPushButton::clicked,
            this, &MainWindow::onBeginScheduleClicked);
}

void MainWindow::onBeginScheduleClicked() {
    TaskPage* taskPage = new TaskPage(map.get(), coordinator.get(), this);
    taskPage->show();
    this->hide();
}