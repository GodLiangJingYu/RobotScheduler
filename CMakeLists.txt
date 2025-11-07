set(CMAKE_PREFIX_PATH "D:/msys64/ucrt64")
cmake_minimum_required(VERSION 3.20)
project(RobotScheduler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# SQLite
find_package(PkgConfig REQUIRED)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SQLITE3 REQUIRED sqlite3)
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Source files
set(SOURCES
        src/main.cpp
        src/core/Robot.cpp
        src/core/Task.cpp
        src/core/Map.cpp
        src/core/Scheduler.cpp
        src/frontend/MainWindow.cpp
        src/frontend/TaskPage.cpp
        src/frontend/MonitorPage.cpp
        src/backend/Coordinator.cpp
        src/backend/PathPlanner.cpp
        src/database/DatabaseManager.cpp
)

set(HEADERS
        src/core/Robot.h
        src/core/Task.h
        src/core/Map.h
        src/core/Scheduler.h
        src/frontend/MainWindow.h
        src/frontend/TaskPage.h
        src/frontend/MonitorPage.h
        src/backend/Coordinator.h
        src/backend/PathPlanner.h
        src/database/DatabaseManager.h
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
        Qt5::Core
        Qt5::Widgets
        Qt5::Gui
        ${SQLITE3_LIBRARIES}
)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/assets/config.json
        ${CMAKE_CURRENT_BINARY_DIR}/config.json
        COPYONLY
)

target_include_directories(${PROJECT_NAME} PRIVATE ${SQLITE3_INCLUDE_DIRS})