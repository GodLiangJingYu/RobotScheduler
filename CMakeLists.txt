cmake_minimum_required(VERSION 3.10)
project(RobotFindWaySystem VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/main.cpp
    src/RobotPathFinder.cpp
    src/Grid.cpp
)

# Header files
set(HEADERS
    include/RobotPathFinder.h
    include/Grid.h
)

# Add executable
add_executable(RobotFindWaySystem ${SOURCES} ${HEADERS})

# Set output directory
set_target_properties(RobotFindWaySystem PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
