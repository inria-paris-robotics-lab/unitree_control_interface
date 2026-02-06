cmake_minimum_required(VERSION 3.8)
project(unitree_control_interface)

# ###################
# # Python Packages #
# ###################
set(PY_PROJECT_NAME ${PROJECT_NAME}_py)
set(COMMON_PROJECT_NAME ${PROJECT_NAME}_common)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PY_PROJECT_NAME})
ament_python_install_package(${COMMON_PROJECT_NAME})

###############
# Cpp Package #
###############
set(CPP_LIB_NAME go2_control_interface_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB CPP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${CPP_LIB_NAME})
file(GLOB CPP_SOURCE_FILES ${CPP_SOURCE_DIR}/src/*)

add_library(${CPP_LIB_NAME} STATIC ${CPP_SOURCE_FILES})

target_include_directories(${CPP_LIB_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CPP_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/>"
)

include_directories(${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${CPP_LIB_NAME} rclcpp unitree_go std_msgs)

install(
  DIRECTORY ${CPP_SOURCE_DIR}/include/
  DESTINATION include/
)

install(
  TARGETS ${CPP_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp unitree_go std_msgs Eigen3)

########################
# Global package files #
########################

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Declare executables for the Python console scripts
file(GLOB COMMON_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_PROJECT_NAME}/*.py)
install(PROGRAMS
  ${COMMON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
