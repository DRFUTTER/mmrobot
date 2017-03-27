cmake_minimum_required(VERSION 2.6)
project (SmartMotor)
#add_compile_options(-std=c++11)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(include)

#add_executable(testSmartMotor testSmartMotor.cpp src/rs232.c src/smartmotor.cpp include/rs232.h include/smartmotor.h)
add_executable(testSmartMotor src/testCom.cpp src/rs232.c src/smartmotor.cpp include/rs232.h include/smartmotor.h)

