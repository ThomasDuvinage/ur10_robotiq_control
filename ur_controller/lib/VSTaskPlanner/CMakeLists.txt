cmake_minimum_required(VERSION 3.12)

project(VSTaskPlanner VERSION 0.0.1) 

# Version du langage
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include)

# Définir la bibliothèque
add_library(${PROJECT_NAME} SHARED
    src/VSTask.cpp
    src/VSTasksPlanner.cpp
)

# Ajouter des en-têtes inclus
target_include_directories(${PROJECT_NAME}
     PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  yaml-cpp
  Eigen3::Eigen
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

add_executable(VSTasksPlannerDemo src/demo.cpp)
target_link_libraries(VSTasksPlannerDemo PUBLIC ${PROJECT_NAME})