cmake_minimum_required(VERSION 3.18)
project(eir_remote_gui LANGUAGES NONE)
include(CTest)
option(EIR_GUI_TESTS "Run local Python/Qt/ROS remote GUI tests" OFF)
option(EIR_GUI_TEST_RQT "Run real rqt depth rendering and process lifecycle test" OFF)
if(BUILD_TESTING AND EIR_GUI_TESTS)
    add_test(NAME remote_gui_test COMMAND /usr/bin/python3
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/remote_gui_test.py -v)
    set_tests_properties(remote_gui_test PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=188;QT_QPA_PLATFORM=offscreen")
endif()
if(BUILD_TESTING AND EIR_GUI_TEST_RQT)
    add_test(NAME remote_depth_viewer_test COMMAND /usr/bin/python3
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/depth_viewer_test.py)
    set_tests_properties(remote_depth_viewer_test PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE
        ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=188;QT_QPA_PLATFORM=offscreen")
endif()
install(PROGRAMS remote_gui.py depth_viewer.py xbox.py file_playback.py DESTINATION share/eir_remote_gui)
install(FILES config/remote_gui.yaml DESTINATION share/eir_remote_gui/config)
install(FILES LICENSE DESTINATION share/eir_remote_gui)
