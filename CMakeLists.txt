cmake_minimum_required(VERSION 3.15)

project(ShortPathAlgorithms LANGUAGES CXX)

set(CXX_STANDARD 11)
set(CXX_STANDARD_REQUIRED true)

add_executable(Dijkstra src/dijkstra.cpp)
add_executable(D src/d.cpp)
add_executable(A src/a.cpp)