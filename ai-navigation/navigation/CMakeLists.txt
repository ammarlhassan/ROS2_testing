cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install this Python package via setup.py
ament_python_install_package(${PROJECT_NAME})

# Install only the launch/ folder (rviz/ removed since it doesn't exist)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()