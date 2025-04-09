cmake_minimum_required(VERSION 3.5)
project(roam_bot)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY diffbot/urdf diffbot/rviz
  DESTINATION share/${PROJECT_NAME}/diffbot
)

ament_package()
