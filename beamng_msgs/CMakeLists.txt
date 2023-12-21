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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeSimulationState.srv"
  "srv/SpawnVehicle.srv"
  "srv/StartScenario.srv"
  "srv/TeleportVehicle.srv"
  "srv/PublishRoadNetwork.srv"
#  "srv/GetCurrentVehiclesInfo.srv"  #need to be fixed
#  "srv/GetScenarioState.srv"   #need to be fixed
#  "msg/DeformGroupInfo.msg" #need to be fixed
  "msg/ElectricsSensor.msg" 
  "msg/DamageSensor.msg" 
  "msg/GForceSensor.msg"
 "msg/PartDamageInfo.msg"
  "msg/ScenarioState.msg"
  "msg/StateSensor.msg"
  "msg/TimeSensor.msg"  
  "msg/VehicleControl.msg"
  "msg/VehicleInfo.msg"
  "action/Step.action"


)

# install(FILES
# DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
