cmake_minimum_required(VERSION 3.0.0)
project(rda5807 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

include(CTest)
enable_testing()

add_executable(rda5807 main.cpp)
target_link_libraries(rda5807 explain)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

