# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dual_arm_lag_gd_planner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dual_arm_lag_gd_planner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dual_arm_lag_gd_planner_FOUND FALSE)
  elseif(NOT dual_arm_lag_gd_planner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dual_arm_lag_gd_planner_FOUND FALSE)
  endif()
  return()
endif()
set(_dual_arm_lag_gd_planner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dual_arm_lag_gd_planner_FIND_QUIETLY)
  message(STATUS "Found dual_arm_lag_gd_planner: 0.1.0 (${dual_arm_lag_gd_planner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dual_arm_lag_gd_planner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dual_arm_lag_gd_planner_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dual_arm_lag_gd_planner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${dual_arm_lag_gd_planner_DIR}/${_extra}")
endforeach()
