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
    TaskPage* taskPage = new TaskPage(map.get(), coordinator.get(), nullptr);  // 改为 nullptr
    taskPage->setAttribute(Qt::WA_DeleteOnClose);  // 关闭时自动删除
    taskPage->show();
    this->close();  // 关闭主窗口而不是隐藏
}