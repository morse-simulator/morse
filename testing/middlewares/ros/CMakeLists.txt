add_morse_test(topics)
add_morse_test(sick)
add_morse_test(video_camera)
add_morse_test(depth_camera)

# action test used actionlib which only work for the moment with python2, so
# search for python2 and use it to run the test
find_package(PythonInterp)
if (PYTHONINTERP_FOUND)
	add_test(NAME "middlewares/ros/actions.py" COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/actions.py)
else(PYTHONINTERP_FOUND)
	message(STATUS "Python2 interpreter not found, won't launch middlewares/ros/actions.py test")
endif(PYTHONINTERP_FOUND)
