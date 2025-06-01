cmake_minimum_required(VERSION 3.5)
project(o1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED) # Kept as main1 might implicitly use it or it's a general project dependency
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
# find_package(ACADO REQUIRED) # Removed
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  # ${ACADO_INCLUDE_DIRS} # Removed
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(main1 src/main1.cpp src/Visualize.cpp)
# add_executable(controller1 src/controller1.cpp) # Removed

# Configure main executable
ament_target_dependencies(main1
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(main1
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  Boost::program_options
  Boost::system
  Boost::filesystem
)

# Configure controller executable # Removed section
# ament_target_dependencies(controller1 # Removed
# rclcpp # Removed
# std_msgs # Removed
# geometry_msgs # Removed
# pluginlib # Removed
# ) # Removed

# target_link_libraries(controller1 # Removed
# ${OMPL_LIBRARIES} # Removed
# ${OpenCV_LIBRARIES} # Removed
# ${Boost_LIBRARIES} # Removed
# ${ACADO_SHARED_LIBRARIES} # Removed
# Eigen3::Eigen # Removed
# Boost::program_options # Removed
# Boost::system # Removed
# Boost::filesystem # Removed
# ) # Removed

# target_include_directories(controller1 PUBLIC # Removed
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Removed
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}> # Removed
# ) # Removed

# target_compile_features(controller1 PUBLIC c_std_99 cxx_std_17) # Removed

# Install targets
install(TARGETS
  main1
  # controller1 # Removed
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()