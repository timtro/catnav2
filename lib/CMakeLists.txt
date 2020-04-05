add_library(Nav2remote STATIC)
target_sources(
  Nav2remote
  PRIVATE Nav2remote.cpp
  INTERFACE Nav2remote.hpp ../include/nav2_Pose.hpp)
target_link_libraries(Nav2remote PRIVATE project_options project_warnings)

add_library(Obstacle STATIC)
target_sources(
  Obstacle
  PRIVATE Obstacle.cpp
  INTERFACE Obstacle.hpp)
target_link_libraries(Obstacle PRIVATE project_options project_warnings)
