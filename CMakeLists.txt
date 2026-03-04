cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0167 NEW)
project(avatar VERSION 0.1.0)
include(${CMAKE_SOURCE_DIR}/cmake/options.cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DCOAL_DISABLE_HPP_FCL_WARNINGS)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(PkgConfig REQUIRED)

if(BUILD_WITH_MUJOCO)
    find_package(tinyxml2 REQUIRED)
    find_package(glfw3 REQUIRED)
    set(MUJOCO_ROOT "$ENV{HOME}/Workspace/tools/mujoco")
    add_library(mujoco SHARED IMPORTED)
    set_target_properties(mujoco PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_ROOT}/Versions/Current/libmujoco.3.2.0.dylib"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}"
    )
    set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "" FORCE)
    find_package(pinocchio REQUIRED)
endif()

if(BUILD_WITH_FRANKA)
    find_package(Franka REQUIRED)
endif()

if(BUILD_STREAMER)
    pkg_check_modules(GST REQUIRED
        gstreamer-1.0
        gstreamer-app-1.0
        gstreamer-video-1.0
    )
endif()

set(SOURCES
    src/main.cpp
    src/avatar.cpp
    src/arm_control.cpp
    src/head_control.cpp
    src/interpolator.cpp
    src/transmission.cpp
)

if(BUILD_WITH_MUJOCO)
    list(APPEND SOURCES
        src/sim_env/simulation.cpp
        src/sim_env/robot.cpp
        src/sim_env/gripper.cpp
        src/sim_env/scene_builder.cpp
        src/sim_env/model.cpp
        src/sim_env/tum_head_driver.cpp
    )
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "/Library/Frameworks"
    INSTALL_RPATH "/Library/Frameworks"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${MUJOCO_ROOT}
    ${GLFW_INCLUDE_DIRS}
    ${Poco_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Boost::filesystem
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    tinyxml2::tinyxml2
    glfw
    Poco::Net
    Poco::Foundation
)

if(BUILD_WITH_MUJOCO)
    target_link_libraries(${PROJECT_NAME} PRIVATE mujoco pinocchio::pinocchio)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_MUJOCO)
endif()

if(BUILD_WITH_FRANKA)
    target_link_libraries(${PROJECT_NAME} PRIVATE Franka::Franka)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_FRANKA)
endif()

if(BUILD_STREAMER)
    pkg_check_modules(GST REQUIRED
        gstreamer-1.0
        gstreamer-app-1.0
        gstreamer-video-1.0
    )

    add_executable(avatar_streamer
        src/streamer/streamer_main.cpp
        src/streamer/video_streamer.cpp
        src/streamer/camera_source.cpp
    )

    target_include_directories(avatar_streamer PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${GST_INCLUDE_DIRS}
    )

    target_link_libraries(avatar_streamer PRIVATE
        yaml-cpp::yaml-cpp
        ${GST_LIBRARIES}
    )

    target_link_directories(avatar_streamer PRIVATE
        ${GST_LIBRARY_DIRS}
    )

    target_compile_options(avatar_streamer PRIVATE
        ${GST_CFLAGS_OTHER}
    )

    target_compile_definitions(avatar_streamer PRIVATE BUILD_STREAMER)
endif()