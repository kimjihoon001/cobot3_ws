cmake_minimum_required(VERSION 3.8)
project(harvest_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf srdf meshes
        DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/grasp_proto.py
        scripts/nav_harvest_demo.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
