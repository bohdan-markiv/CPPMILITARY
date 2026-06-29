cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 04. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(drone_autopilot CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_library(uart src/uart.cpp)
target_include_directories(uart PUBLIC include)
target_compile_options(uart PRIVATE -Wall -Wextra -pedantic)
add_library(gpio src/gpio.cpp)
target_include_directories(gpio PUBLIC include)
target_compile_options(gpio PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(gpio PUBLIC gpiod)  
add_library(guidance src/guidance.cpp)
target_include_directories(guidance PUBLIC include)
target_compile_options(guidance PRIVATE -Wall -Wextra -pedantic)
# Один executable для задачі з UGV odometry.
add_executable(drone_autopilot src/main.cpp)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(drone_autopilot PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(
    drone_autopilot
    PRIVATE 
    uart
    gpio
    guidance
    )