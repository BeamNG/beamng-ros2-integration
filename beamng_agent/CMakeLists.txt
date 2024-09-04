cmake_minimum_required(VERSION 3.5)
project(beamng_agent)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(beamng_msgs REQUIRED)  # Ensure this is included

# Install Python modules (if your package contains Python scripts)
ament_python_install_package(${PROJECT_NAME})

# Install executables (adjust the path to your script)
install(PROGRAMS
  scripts/beamng_agent.py  # Adjust this if the script is located elsewhere or has a different name
  DESTINATION lib/${PROJECT_NAME})

# Install package.xml to the share directory
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package for installation
ament_package()
