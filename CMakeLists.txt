cmake_minimum_required(VERSION 2.8.12)

project(Simulator2017)

# Add our own CMake scripts to the module path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set some nicer output dirs.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set(SRCS
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
	"${PROJECT_SOURCE_DIR}/src/bitmap.c"
    "${PROJECT_SOURCE_DIR}/src/CollisionSystem.cpp"
    "${PROJECT_SOURCE_DIR}/src/CollisionThread.cpp"
    "${PROJECT_SOURCE_DIR}/src/DrawFunc.cpp"
    "${PROJECT_SOURCE_DIR}/src/Geometry.cpp"
    "${PROJECT_SOURCE_DIR}/src/GlutVizualisation.cpp"
	"${PROJECT_SOURCE_DIR}/src/Grid.cpp"
	"${PROJECT_SOURCE_DIR}/src/GridCollision.cpp"
    "${PROJECT_SOURCE_DIR}/src/MapHandler.cpp"
    "${PROJECT_SOURCE_DIR}/src/matrix.cpp"
    "${PROJECT_SOURCE_DIR}/src/Model.cpp"
    "${PROJECT_SOURCE_DIR}/src/ObstacleMap.cpp"
    "${PROJECT_SOURCE_DIR}/src/OptimalTrajectory.cpp"
    "${PROJECT_SOURCE_DIR}/src/planner.cpp"
    "${PROJECT_SOURCE_DIR}/src/PlannerTypes.cpp"
    "${PROJECT_SOURCE_DIR}/src/RTError.cpp"
    "${PROJECT_SOURCE_DIR}/src/Simulator.cpp"
    "${PROJECT_SOURCE_DIR}/src/Trajectory.cpp"
	"${PROJECT_SOURCE_DIR}/src/WerlingPoints.cpp"
	)

# Only used by generators like Visual Studio that won't list the header files in the project otherwise.
set(HDRS
    "${PROJECT_SOURCE_DIR}/include/bitmap.h"
    "${PROJECT_SOURCE_DIR}/include/CollisionSystem.h"
    "${PROJECT_SOURCE_DIR}/include/CollisionThread.h"
    "${PROJECT_SOURCE_DIR}/include/DrawFunc.h"
    "${PROJECT_SOURCE_DIR}/include/Geometry.h"
    "${PROJECT_SOURCE_DIR}/include/globaldefines.h"
    "${PROJECT_SOURCE_DIR}/include/GlutVizualisation.h"
    "${PROJECT_SOURCE_DIR}/include/Grid.h"
    "${PROJECT_SOURCE_DIR}/include/GridCollision.h"
    "${PROJECT_SOURCE_DIR}/include/MapHandler.h"
    "${PROJECT_SOURCE_DIR}/include/matrix.h"
    "${PROJECT_SOURCE_DIR}/include/Model.h"
    "${PROJECT_SOURCE_DIR}/include/ObstacleMap.h"
    "${PROJECT_SOURCE_DIR}/include/OptimalTrajectory.h"
    "${PROJECT_SOURCE_DIR}/include/Planner.h"
    "${PROJECT_SOURCE_DIR}/include/PlannerTypes.h"
    "${PROJECT_SOURCE_DIR}/include/RTError.h"
    "${PROJECT_SOURCE_DIR}/include/Simulator.h"
	"${PROJECT_SOURCE_DIR}/include/Trajectory.h"
	"${PROJECT_SOURCE_DIR}/include/WerlingPoints.h"
    )

add_executable(Simulator2017 ${SRCS} ${HDRS})

target_include_directories(Simulator2017 PRIVATE "${PROJECT_SOURCE_DIR}/include")


# TODO: "proper way" but the find script sucks :)
#find_package(FreeGLUT REQUIRED)
#message("GLUT libraries: ${FREEGLUT_LIBRARIES}")
#message("GLUT Include dir: ${FREEGLUT_INCLUDE_DIR}")

# For now we hard code it
set(FREEGLUT_LIBRARIES "" CACHE STRING "Path to glut library")
set(FREEGLUT_INCLUDE_DIR "" CACHE STRING "Path to glut include dir")
if (NOT FREEGLUT_LIBRARIES)
    message(FATAL_ERROR "\nYou must specify the path to the freeglut library with -DFREEGLUT_LIBRARIES=<path>")
endif()

if (NOT FREEGLUT_INCLUDE_DIR)
    message(FATAL_ERROR "\nYou must specify the path to the freeglut library with -DFREEGLUT_INCLUDE_DIR=<path>")
endif()

target_link_libraries(Simulator2017 ${FREEGLUT_LIBRARIES})
target_include_directories(Simulator2017 PRIVATE ${FREEGLUT_INCLUDE_DIR})
