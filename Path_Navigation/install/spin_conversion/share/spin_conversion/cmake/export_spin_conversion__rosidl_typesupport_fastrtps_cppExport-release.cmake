#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "spin_conversion::spin_conversion__rosidl_typesupport_fastrtps_cpp" for configuration "Release"
set_property(TARGET spin_conversion::spin_conversion__rosidl_typesupport_fastrtps_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spin_conversion::spin_conversion__rosidl_typesupport_fastrtps_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspin_conversion__rosidl_typesupport_fastrtps_cpp.so"
  IMPORTED_SONAME_RELEASE "libspin_conversion__rosidl_typesupport_fastrtps_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS spin_conversion::spin_conversion__rosidl_typesupport_fastrtps_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_spin_conversion::spin_conversion__rosidl_typesupport_fastrtps_cpp "${_IMPORT_PREFIX}/lib/libspin_conversion__rosidl_typesupport_fastrtps_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
