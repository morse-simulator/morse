if (BUILD_CORE_SUPPORT)
    add_morse_test(jointstate_sockets)
    add_morse_test(torso_sockets)
endif()

if (BUILD_ROS_SUPPORT)
    add_morse_test(jointstate_ros)
    add_morse_test(torso_ros)
endif()

