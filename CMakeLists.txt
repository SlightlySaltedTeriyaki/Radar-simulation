cmake_minimum_required(VERSION 3.16)
project(RadarSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release build flags — zkus měnit a sleduj rozdíl ve výkonu
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

add_executable(radarsim
    src/main.cpp
    src/signal_generator.cpp
    src/signal_processor.cpp
    src/target_tracker.cpp
)

target_include_directories(radarsim PRIVATE include)

add_executable(tests
    tests/test_main.cpp
    src/signal_generator.cpp
    src/signal_processor.cpp
    src/target_tracker.cpp
)
target_include_directories(tests PRIVATE include)

# Zapni pro detekci paměťových chyb (jen na Linuxu / WSL):
# target_compile_options(radarsim PRIVATE -fsanitize=address)
# target_link_options(radarsim PRIVATE -fsanitize=address)
