if (BUILD_CORE_SUPPORT)
    add_morse_test(jointstate-sockets)
    add_morse_test(torso-sockets)
endif()

if (BUILD_ROS_SUPPORT)
    add_morse_test(jointstate-ros)
    add_morse_test(torso-ros)
endif()

