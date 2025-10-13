#include "Scheduler.h"
#include <random>
#include <chrono>

Scheduler::Scheduler(Coordinator* coordinator, Map* map)
    : coordinator(coordinator), map(map), running(false), complete(false),
      totalTasks(0), generatedTasks(0) {}

Scheduler::~Scheduler() {
    stop();
}

void Scheduler::start() {
    if (running.exchange(true)) return;

    generatorThread = std::thread(&Scheduler::taskGenerator, this);
    executorThread = std::thread(&Scheduler::taskExecutor, this);
}

void Scheduler::stop() {
    running = false;
    cv.notify_all();

    if (generatorThread.joinable()) {
        generatorThread.join();
    }
    if (executorThread.joinable()) {
        executorThread.join();
    }
}

void Scheduler::generateTasks(int count) {
    totalTasks = count;
    generatedTasks = 0;
}

void Scheduler::taskGenerator() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, Map::MAP_SIZE - 1);

    while (running && generatedTasks < totalTasks) {
        // Generate random start and end points
        Position start(dis(gen), dis(gen));
        Position end(dis(gen), dis(gen));

        // Ensure positions are valid
        if (map->isValidPosition(start) && map->isValidPosition(end)) {
            coordinator->createTask(start, end);
            generatedTasks++;
        }

        // Wait 1 second between task generation
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void Scheduler::taskExecutor() {
    while (running) {
        // Simple task execution logic
        // In a real system, this would be more sophisticated
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (generatedTasks >= totalTasks && coordinator->getPendingTaskCount() == 0) {
            complete = true;
            break;
        }
    }
}