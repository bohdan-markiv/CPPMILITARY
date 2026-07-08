cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 07. Його можна збирати як через root CMakeLists.txt, так
# і напряму з цієї директорії.
project(ballistics CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(helpers src/helpers.cpp)
target_include_directories(helpers PUBLIC include)
target_compile_options(helpers PRIVATE -Wall -Wextra -pedantic)

add_library(factory src/config/ComponentFactory.cpp)
target_include_directories(factory PUBLIC include)
target_compile_options(factory PRIVATE -Wall -Wextra -pedantic)

add_library(missionprocessor src/MissionProcessor.cpp)
target_include_directories(missionprocessor PUBLIC include)
target_compile_options(missionprocessor PRIVATE -Wall -Wextra -pedantic)

add_library(ballistictable src/solvers/BallisticTable.cpp)
target_include_directories(ballistictable PUBLIC include)
target_compile_options(ballistictable PRIVATE -Wall -Wextra -pedantic)

add_library(state src/state/StateClasses.cpp)
target_include_directories(state PUBLIC include)
target_compile_options(state PRIVATE -Wall -Wextra -pedantic)

add_library(analyticalsolver src/solvers/AnalyticalSolver.cpp)
target_include_directories(analyticalsolver PUBLIC include)
target_compile_options(analyticalsolver PRIVATE -Wall -Wextra -pedantic)
add_library(tablesolver src/solvers/TableSolver.cpp)
target_include_directories(tablesolver PUBLIC include)
target_compile_options(tablesolver PRIVATE -Wall -Wextra -pedantic)
add_library(fileconfigloader src/config/FileConfigLoader.cpp)
target_include_directories(fileconfigloader PUBLIC include)
target_compile_options(fileconfigloader PRIVATE -Wall -Wextra -pedantic)
add_library(jsonconfigloader src/providers/JsonTargetProvider.cpp)
target_include_directories(jsonconfigloader PUBLIC include)
target_compile_options(jsonconfigloader PRIVATE -Wall -Wextra -pedantic)

add_library(dronephysics src/threads/DronePhysics.cpp)
target_include_directories(dronephysics PUBLIC include)
target_compile_options(dronephysics PRIVATE -Wall -Wextra -pedantic)

# Один executable для задачі з ballistics.
add_executable(ballistics src/main.cpp)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_include_directories(ballistics PRIVATE include)

target_compile_options(ballistics PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(
    ballistics
    PRIVATE factory
            helpers
            analyticalsolver
            missionprocessor
            fileconfigloader
            jsonconfigloader
            tablesolver
            ballistictable
            state
            dronephysics)
target_link_libraries(analyticalsolver PRIVATE helpers)
