macro(add_morse_test name)
	set(full_name ${CMAKE_CURRENT_SOURCE_DIR}/${name})
	string(REPLACE "${CMAKE_SOURCE_DIR}/testing/" "" test_name ${full_name})
	add_test(NAME ${test_name} COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.py)
endmacro()

if (BUILD_CORE_SUPPORT)
	add_subdirectory(base)
	add_subdirectory(middlewares/sockets)
endif()

if (BUILD_ROS_SUPPORT)
	add_subdirectory(middlewares/ros)
endif()
