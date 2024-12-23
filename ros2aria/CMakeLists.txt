cmake_minimum_required(VERSION 3.8)
project(ros2aria)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Aria QUIET)
if(Aria_FOUND)
  message("CMake package for Aria was found, using that.")
  # Aria package for CMake was found
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # No Aria package for CMake was found, search ourselves
	# The installation package provided by Adept doesn't follow normal Linux
	# installation locations, but the repackaged Debian package and AriaCoda do.
  # If user set ARIA environment variable, look there, otherwise look in
  # /usr/local/.
  if(DEFINED ENV{ARIA})
    message("ARIA environment variable defined, checking there...")
    set(prefix $ENV{ARIA})
  else()
    set(prefix "/usr/local")
  endif()
  message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

	if(EXISTS "${prefix}/Aria/include/Aria.h")
    message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
	  add_definitions(-DADEPT_PKG)
	  include_directories(${prefix}/Aria/include)
	  link_directories(${prefix}/Aria/lib)
  else()
    if(EXISTS "${prefix}/include/Aria.h")
      message("Found ${prefix)/include/Aria.h, assuming Adept ARIA source directory.")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/include)
      link_directories(${prefix}/lib)
    else()
      if(EXISTS "${prefix}/include/Aria/Aria.h")
        message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda or repackaged ARIA.")
        #add_definitions(-DARIACODA)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
      endif()
    endif()
	endif()
endif()

add_executable(ros2aria_run src/Ros2Aria.cpp)
target_compile_features(ros2aria_run PUBLIC c_std_99 cxx_std_17)  # Enforce C++17
target_include_directories(ros2aria_run PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ros2aria_run ${Boost_LIBRARIES} Aria pthread dl rt)
set_target_properties(ros2aria_run PROPERTIES COMPILE_FLAGS "-fPIC")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ros2aria_run
    DESTINATION lib/${PROJECT_NAME}/
)

# ament_export_dependencies(rosidl_default_reuntime)

ament_target_dependencies(ros2aria_run rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros nav_msgs std_srvs rcl_interfaces pcl_ros)

ament_package()
