#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "open_manipulator_msgs::open_manipulator_msgs__rosidl_generator_py" for configuration "Debug"
set_property(TARGET open_manipulator_msgs::open_manipulator_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(open_manipulator_msgs::open_manipulator_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libopen_manipulator_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "libopen_manipulator_msgs__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS open_manipulator_msgs::open_manipulator_msgs__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_open_manipulator_msgs::open_manipulator_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libopen_manipulator_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
