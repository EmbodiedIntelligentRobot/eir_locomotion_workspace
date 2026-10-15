include_guard(GLOBAL)

function(eir_require_sdk_file relative_path)
    if(NOT EXISTS "${EIR_SDK_ROOT}/${relative_path}")
        message(FATAL_ERROR "Missing SDK file: ${EIR_SDK_ROOT}/${relative_path}. Obtain a complete SDK release or rebuild and install the private libraries into the SDK.")
    endif()
endfunction()

foreach(header IN ITEMS EirLowLevelApi.h DataTypes.h Visibility.h)
    eir_require_sdk_file("include/eir_lowlevel_sdk/${header}")
endforeach()
add_library(EIR::lowlevel_sdk INTERFACE IMPORTED GLOBAL)
set_target_properties(EIR::lowlevel_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIR_SDK_ROOT}/include"
    INTERFACE_COMPILE_FEATURES cxx_std_17
)

set(EIR_HARDWARE_FAMILY "aiq11" CACHE STRING "Hardware SDK family: aiq11 or eqr11 (one per build)")
set_property(CACHE EIR_HARDWARE_FAMILY PROPERTY STRINGS aiq11 eqr11)
if(NOT EIR_HARDWARE_FAMILY MATCHES "^(aiq11|eqr11)$")
    message(FATAL_ERROR "EIR_HARDWARE_FAMILY must be aiq11 or eqr11")
endif()
if(EIR_WITH_HARDWARE)
    add_library(EIR::base_sdk SHARED IMPORTED GLOBAL)
    if(EIR_HARDWARE_FAMILY STREQUAL "eqr11")
        set(EIR_EQR_SDK_ROOT "${EIR_SDK_ROOT}/variants/eqr11/${CMAKE_SYSTEM_PROCESSOR}/1.1.0" CACHE PATH "Versioned EQR SDK release")
        foreach(header IN ITEMS EirLowLevelApi.h DataTypes.h Visibility.h)
            file(SHA256 "${EIR_SDK_ROOT}/include/eir_lowlevel_sdk/${header}" shared_hash)
            file(SHA256 "${EIR_EQR_SDK_ROOT}/include/eir_lowlevel_sdk/${header}" eqr_hash)
            if(NOT shared_hash STREQUAL eqr_hash)
                message(FATAL_ERROR "EQR/shared SDK header mismatch: ${header}")
            endif()
        endforeach()
        foreach(file IN ITEMS include/eir_base_sdk_eqr/EqrLowLevelApi.h include/eir_base_sdk_eqr/eir_eqr_abi.h lib/libeir_base_sdk_eqr.so.1)
            if(NOT EXISTS "${EIR_EQR_SDK_ROOT}/${file}")
                message(FATAL_ERROR "Missing EQR SDK release file: ${EIR_EQR_SDK_ROOT}/${file}")
            endif()
        endforeach()
        set_target_properties(EIR::base_sdk PROPERTIES
            IMPORTED_LOCATION "${EIR_EQR_SDK_ROOT}/lib/libeir_base_sdk_eqr.so.1"
            INTERFACE_INCLUDE_DIRECTORIES "${EIR_EQR_SDK_ROOT}/include"
            INTERFACE_LINK_LIBRARIES EIR::lowlevel_sdk)
    else()
        eir_require_sdk_file("include/eir_base_sdk/AiqLowLevelApi.h")
        eir_require_sdk_file("lib/libEirLowLevelApi.so")
        set_target_properties(EIR::base_sdk PROPERTIES
            IMPORTED_LOCATION "${EIR_SDK_ROOT}/lib/libEirLowLevelApi.so"
            INTERFACE_LINK_LIBRARIES EIR::lowlevel_sdk)
    endif()
endif()

if(EIR_WITH_MUJOCO)
    foreach(header IN ITEMS mj_bridge.h finger_coupling.h depth_camera.h sensor_processing.h)
        eir_require_sdk_file("include/eir_mujoco/${header}")
    endforeach()
    eir_require_sdk_file("lib/libeir_mujoco.so")
    find_package(Threads REQUIRED)
    find_package(Eigen3 3.3 REQUIRED)
    # Always use the bundled engine; never fall back to system or cached 3.3.7 paths.
    eir_require_sdk_file("include/mujoco/mujoco.h")
    eir_require_sdk_file("lib/libmujoco.so.3.14.0")
    file(STRINGS "${EIR_SDK_ROOT}/include/mujoco/mujoco.h" _eir_mujoco_version
        REGEX "^#define mjVERSION_HEADER 3014000$")
    if(NOT _eir_mujoco_version)
        message(FATAL_ERROR "SDK must contain MuJoCo 3.14.0 headers")
    endif()
    set(EIR_MUJOCO_INCLUDE_DIR "${EIR_SDK_ROOT}/include")
    set(EIR_MUJOCO_LIBRARY "${EIR_SDK_ROOT}/lib/libmujoco.so.3.14.0")
    add_library(EIR::mujoco_sdk SHARED IMPORTED GLOBAL)
    set_target_properties(EIR::mujoco_sdk PROPERTIES
        IMPORTED_LOCATION "${EIR_SDK_ROOT}/lib/libeir_mujoco.so"
        INTERFACE_INCLUDE_DIRECTORIES "${EIR_MUJOCO_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "EIR::lowlevel_sdk;Eigen3::Eigen;Threads::Threads;${EIR_MUJOCO_LIBRARY}"
        INTERFACE_COMPILE_FEATURES cxx_std_20
    )
endif()
