cmake_minimum_required(VERSION 3.14)
project(embotech_prodriver_connector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

add_compile_options(-Wno-error=unused-variable -Wno-error=unused-parameter)  # suppress errors when building PTCL
# Add PTCL. Expected at ../ptcl or set as PTCL_PATH.
if(NOT PTCL_PATH)
    set(PTCL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../ptcl")
endif()
if(NOT EXISTS "${PTCL_PATH}")
    message(FATAL_ERROR "Could not find ptcl at ${PTCL_PATH}")
endif()
if(NOT TARGET ptcl)
    add_subdirectory("${PTCL_PATH}" ${CMAKE_CURRENT_BINARY_DIR}/ptcl)
endif()

target_link_libraries(${PROJECT_NAME} ptcl ptclUdpPort ptclUtils)

# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "embotech_prodriver_connector::EmbotechProDriverConnector"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_${PROJECT_NAME}
    test/test_embotech_prodriver_connector_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch param)
