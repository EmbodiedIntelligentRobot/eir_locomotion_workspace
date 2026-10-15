#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "eir_mujoco::eir_mujoco" for configuration "Release"
set_property(TARGET eir_mujoco::eir_mujoco APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(eir_mujoco::eir_mujoco PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libeir_mujoco.so"
  IMPORTED_SONAME_RELEASE "libeir_mujoco.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS eir_mujoco::eir_mujoco )
list(APPEND _IMPORT_CHECK_FILES_FOR_eir_mujoco::eir_mujoco "${_IMPORT_PREFIX}/lib/libeir_mujoco.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
