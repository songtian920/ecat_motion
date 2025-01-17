# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ecat_motion_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ecat_motion_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ecat_motion_FOUND FALSE)
  elseif(NOT ecat_motion_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ecat_motion_FOUND FALSE)
  endif()
  return()
endif()
set(_ecat_motion_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ecat_motion_FIND_QUIETLY)
  message(STATUS "Found ecat_motion: 0.0.0 (${ecat_motion_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ecat_motion' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ecat_motion_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ecat_motion_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ecat_motion_DIR}/${_extra}")
endforeach()
