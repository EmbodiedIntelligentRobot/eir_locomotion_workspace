
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was eir_mujocoConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################
# Older CMake dependency configs may overwrite PACKAGE_PREFIX_DIR.
set_and_check(_eir_mujoco_include "${PACKAGE_PREFIX_DIR}/include")
set_and_check(_eir_mujoco_library "${PACKAGE_PREFIX_DIR}/lib/libmujoco.so.3.14.0")
include(CMakeFindDependencyMacro)
find_dependency(Threads)
find_dependency(Eigen3 3.3)
find_dependency(glfw3)
if(NOT TARGET MuJoCo::mujoco)
  add_library(MuJoCo::mujoco SHARED IMPORTED)
  set_target_properties(MuJoCo::mujoco PROPERTIES
    IMPORTED_LOCATION "${_eir_mujoco_library}"
    INTERFACE_INCLUDE_DIRECTORIES "${_eir_mujoco_include}")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/eir_mujocoTargets.cmake")
check_required_components(eir_mujoco)
