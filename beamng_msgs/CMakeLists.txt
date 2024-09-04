cmake_minimum_required(VERSION 3.5)
project(beamng_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeSimulationState.srv"
  "srv/SpawnVehicle.srv"
  "srv/VehicleCommand.srv"
  "srv/SetControlMode.srv"
  "srv/StartScenario.srv"
  "srv/TeleportVehicle.srv"
  "srv/PublishRoadNetwork.srv"
  "srv/GetCurrentVehiclesInfo.srv"
  "srv/GetScenarioState.srv"
  "srv/StartCosimulation.srv"
  "srv/StopCosimulation.srv"
  "msg/CosimulationData.msg"
  "msg/CosimulationInput.msg"
  "msg/CubicPolynomial.msg"
  "msg/ElectricsSensor.msg"
  "msg/DamageSensor.msg"
  "msg/Float64.msg"
  "msg/GForceSensor.msg"
  "msg/IdealRadarSensor.msg"
  "msg/IdealRadarSensorVehicle.msg"
  "msg/MeshSensor.msg"
  "msg/MeshSensorBeam.msg"
  "msg/MeshSensorNode.msg"
  "msg/PowertrainSensor.msg"
  "msg/PowertrainSensorDevice.msg"
  "msg/RadarReturn.msg"
  "msg/RadarScan.msg"
  "msg/RoadsSensor.msg"
  "msg/ScenarioState.msg"
  "msg/StateSensor.msg"
  "msg/TimeSensor.msg"
  "msg/VehicleControl.msg"
  "msg/VehicleInfo.msg"
  "msg/WheelThermals.msg"
  "action/Step.action"

  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
