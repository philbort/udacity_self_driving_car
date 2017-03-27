project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/tools.cpp
    src/LaserMeasurement.cpp
    src/LaserMeasurement.h
    src/RadarMeasurement.cpp
    src/RadarMeasurement.h)

add_executable(ExtendedKF ${sources})