cmake_minimum_required(VERSION 3.5)
project(consai2r2_gameviewer)

# Load ament and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/consai2r2_gameviewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
