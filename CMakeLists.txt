cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(hw3 LANGUAGES C CXX)

if (NOT (CMAKE_BUILD_TYPE OR CMAKE_CONFIGURATION_TYPES))
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "RelWithDebInfo" "Debug")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/glfw)

add_executable(hw3
    main.cpp
    src/fluid.cpp
    src/scene.cpp
    src/simulation_tests.cpp
    vendor/src/imgui.cpp
    vendor/src/imgui_draw.cpp
    vendor/src/imgui_tables.cpp
    vendor/src/imgui_widgets.cpp
    vendor/src/imgui_impl_glfw.cpp
    vendor/src/imgui_impl_opengl3.cpp
)

target_compile_definitions(hw3 PRIVATE
    GLFW_INCLUDE_NONE
    IMGUI_IMPL_OPENGL_LOADER_GLAD2
    EIGEN_MPL2_ONLY
    EIGEN_NO_DEBUG
    EIGEN_DONT_VECTORIZE
)

target_include_directories(hw3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/include
)

find_package(Threads REQUIRED)
target_link_libraries(hw3 PRIVATE Threads::Threads glfw)
if(WIN32)
    target_link_libraries(hw3 PRIVATE imm32)
endif()

if(MSVC)
    target_compile_options(hw3 PRIVATE /W3 /MP)
else()
    target_compile_options(hw3 PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

install(TARGETS hw3 RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

enable_testing()
add_test(NAME fluid_pipeline COMMAND hw3 --self-test)
