cmake_minimum_required(VERSION 3.10)
project(path_planning_harness CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_SOURCE_DIR}/include)

# 환경(하네스) 소스 — 사용자는 건드릴 필요 없음.
set(ENV_SOURCES
    src/grid.cpp
    src/validator.cpp
    src/renderer.cpp
    src/scenario.cpp
)

# 사용자가 알고리즘을 채우는 파일.
set(USER_SOURCES
    user/my_planner.cpp
)

add_executable(run_tests
    tests/run_tests.cpp
    ${ENV_SOURCES}
    ${USER_SOURCES}
)

enable_testing()
add_test(NAME scenarios COMMAND run_tests)
