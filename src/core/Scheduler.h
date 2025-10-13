#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "Robot.h"
#include "Task.h"
#include "Map.h"
#include "../backend/Coordinator.h"
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class Scheduler {
public:
    Scheduler(Coordinator* coordinator, Map* map);
    ~Scheduler();

    void start();
    void stop();
    void generateTasks(int count);

    bool isRunning() const { return running; }
    bool isComplete() const { return complete; }

private:
    void taskGenerator();
    void taskExecutor();

    Coordinator* coordinator;
    Map* map;
    std::atomic<bool> running;
    std::atomic<bool> complete;

    std::thread generatorThread;
    std::thread executorThread;
    std::mutex taskMutex;
    std::condition_variable cv;

    int totalTasks;
    int generatedTasks;
};

#endif // SCHEDULER_H