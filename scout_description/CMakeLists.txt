cmake_minimum_required(VERSION 3.8)

project(scout_description)

find_package(ament_cmake REQUIRED)

# The share dir is what resolves package://scout_description/... at runtime,
# so the meshes must land here for Foxglove to fetch them.
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
