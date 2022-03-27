cmake_minimum_required(VERSION 3.20)
project(Rigid_Body_Simulation)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rigid_Body_Simulation main.cpp VisualizerOpenGL.cpp VisualizerOpenGL.h Model.cpp Model.h State.h Calculator.cpp Calculator.h State.cpp)
target_link_libraries(Rigid_Body_Simulation -lglut -lGLU -lGL)
