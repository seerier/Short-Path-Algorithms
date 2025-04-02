cmake_minimum_required(VERSION 3.15)

project(ShortPathAlgorithms LANGUAGES CXX)

set(CXX_STANDARD 11)
set(CXX_STANDARD_REQUIRED true)

add_executable(Dijkstra dijkstra/dijkstra.cpp)
add_executable(D d/d.cpp)
add_executable(A a/a.cpp)