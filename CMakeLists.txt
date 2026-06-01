cmake_minimum_required(VERSION 3.16)
project(RadarSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- Benchmark ---
add_executable(radarsim
    src/main.cpp
    src/signal_generator.cpp
    src/signal_processor.cpp
    src/target_tracker.cpp
)
target_include_directories(radarsim PRIVATE include)
if(MSVC)
    target_compile_options(radarsim PRIVATE $<$<CONFIG:Release>:/O2 /Oi /Ot /Gy>)
else()
    target_compile_options(radarsim PRIVATE $<$<CONFIG:Release>:-O3 -march=native>)
endif()

# --- Tests ---
add_executable(tests
    tests/test_main.cpp
    src/signal_generator.cpp
    src/signal_processor.cpp
    src/target_tracker.cpp
)
target_include_directories(tests PRIVATE include)

# --- GUI dependencies (downloaded automatically at configure time) ---
include(FetchContent)

set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.9
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90.4
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
)
FetchContent_GetProperties(implot)
if(NOT implot_POPULATED)
    FetchContent_Populate(implot)
endif()

find_package(OpenGL REQUIRED)

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)

# --- GUI executable ---
add_executable(radarsim_gui
    src/main_gui.cpp
    src/signal_generator.cpp
    src/signal_processor.cpp
    src/target_tracker.cpp
)
target_include_directories(radarsim_gui PRIVATE include)
target_link_libraries(radarsim_gui PRIVATE imgui_lib)
if(MSVC)
    target_compile_options(radarsim_gui PRIVATE $<$<CONFIG:Release>:/O2 /Oi /Ot /Gy>)
else()
    target_compile_options(radarsim_gui PRIVATE $<$<CONFIG:Release>:-O3 -march=native>)
endif()
