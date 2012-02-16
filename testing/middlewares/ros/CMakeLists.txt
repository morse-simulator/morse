add_morse_test(services)
add_morse_test(data_stream)

# action test used actionlib which only work for the moment with python2, so
# search for python2 and use it to run the test
find_package(PythonInterp)
if (PYTHONINTERP_FOUND)
	add_test(NAME "middleware/ros/actions.py" COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/actions.py)
else(PYTHONINTERP_FOUND)
	message(STATUS "Python2 interpreter not found, won't launch middleware/ros/actions.py test")
endif(PYTHONINTERP_FOUND)
