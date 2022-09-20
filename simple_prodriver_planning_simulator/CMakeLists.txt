cmake_minimum_required(VERSION 3.14)
project(simple_prodriver_planning_simulator)

find_package(autoware_cmake REQUIRED)
autoware_package()


# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/simple_prodriver_planning_simulator/simple_prodriver_planning_simulator_core.hpp
  include/simple_prodriver_planning_simulator/visibility_control.hpp
  src/simple_prodriver_planning_simulator/simple_prodriver_planning_simulator_core.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_interface.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_delay_steer_vel.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp
  src/simple_prodriver_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC 
  ${tf2_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast) # RCLCPP_ERROR_THROTTLE() has built-in old-style casts.



target_include_directories(${PROJECT_NAME} 
PUBLIC 
  ${CMAKE_SOURCE_DIR}/../../embo_example/include
  ${CMAKE_SOURCE_DIR}/../../ptcl/include)
target_link_directories(${PROJECT_NAME} 
PUBLIC 
  ${CMAKE_SOURCE_DIR}/../../../../install/embo_example/lib/
  )
target_link_libraries(${PROJECT_NAME} libembo_example.so libptcl.so libptclMutex.so libcobs.so libcrc32.so libcobs.so libptcl.so libptclLoopbackPort.so libptclUdpPort.so libcrc32.so libptclBindings.so libptclMutex.so libptclUtils.so libembo_example.so libptclFilePortUtils.so libptclSi.so)

# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simulation::simple_prodriver_planning_simulator::SimpleProdriverPlanningSimulator"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_simple_prodriver_planning_simulator
    test/test_simple_prodriver_planning_simulator.cpp
    TIMEOUT 120
  )


  target_link_libraries(test_simple_prodriver_planning_simulator
    ${PROJECT_NAME}
  )
  
endif()




ament_auto_package(INSTALL_TO_SHARE param launch)
