#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "interfaces::interfaces__rosidl_typesupport_fastrtps_c" for configuration ""
set_property(TARGET interfaces::interfaces__rosidl_typesupport_fastrtps_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(interfaces::interfaces__rosidl_typesupport_fastrtps_c PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libinterfaces__rosidl_typesupport_fastrtps_c.so"
  IMPORTED_SONAME_NOCONFIG "libinterfaces__rosidl_typesupport_fastrtps_c.so"
  )

list(APPEND _cmake_import_check_targets interfaces::interfaces__rosidl_typesupport_fastrtps_c )
list(APPEND _cmake_import_check_files_for_interfaces::interfaces__rosidl_typesupport_fastrtps_c "${_IMPORT_PREFIX}/lib/libinterfaces__rosidl_typesupport_fastrtps_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
