cmake_minimum_required(VERSION 3.14)
project(embotech_prodriver_connector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/embotech_prodriver_connector/embotech_prodriver_connector.hpp
  include/embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp
  src/embotech_prodriver_connector.cpp
  src/embotech_prodriver_connector_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/../../embo_example/include
  ${CMAKE_SOURCE_DIR}/../../ptcl/include
)

target_link_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/../../../../install/embo_example/lib/
)

target_link_libraries(${PROJECT_NAME}
  libembo_example.so 
  libptcl.so 
  libptclMutex.so 
  libcobs.so 
  libcrc32.so 
  libcobs.so 
  libptcl.so 
  libptclLoopbackPort.so 
  libptclUdpPort.so 
  libcrc32.so 
  libptclBindings.so 
  libptclMutex.so 
  libptclUtils.so 
  libembo_example.so 
  libptclFilePortUtils.so 
  libptclSi.so
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast) # RCLCPP_ERROR_THROTTLE() has built-in old-style casts.

# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "embotech_prodriver_connector::EmbotechProDriverConnector"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# ament_auto_package(INSTALL_TO_SHARE param launch)
ament_auto_package(
  INSTALL_TO_SHARE
  )
