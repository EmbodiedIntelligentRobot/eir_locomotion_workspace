cmake_minimum_required(VERSION 3.18)
project(eir_locomotion_workspace LANGUAGES CXX)

set(EIR_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "Prebuilt EIR SDK")
option(EIR_WITH_HARDWARE "Enable the hardware SDK" OFF)
option(EIR_WITH_MUJOCO "Enable the MuJoCo simulation SDK" ON)
include(cmake/EirSdk.cmake)
include(CTest)

foreach(component IN ITEMS src/eir_loco_runtime apps/eir_ctrl_demo_aiq11)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${component}/CMakeLists.txt")
        message(FATAL_ERROR "Missing ${component}; run git submodule update --init --recursive")
    endif()
    add_subdirectory("${component}")
endforeach()
add_subdirectory(apps/eir_remote_gui)

# The application copies the other GUI modules; include its file-playback module too.
if(TARGET eir_ctrl_demo_aiq11)
    configure_file(apps/eir_remote_gui/file_playback.py
        apps/eir_ctrl_demo_aiq11/remote_gui/file_playback.py COPYONLY)
endif()
