cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME morse)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/)
find_package(PkgConfig REQUIRED)

find_package(PythonInterp REQUIRED)

option(BUILD_YARP_SUPPORT "Build yarp framework support" ON)
if (BUILD_YARP_SUPPORT)
	find_package(YARP2 REQUIRED)

	EXECUTE_PROCESS(COMMAND 
					${PYTHON_EXECUTABLE} -c "import yarp;"
					RESULT_VARIABLE YARP_PY_ERR
				   )
	if (YARP_PY_ERR)
	endif (YARP_PY_ERR)
endif(BUILD_YARP_SUPPORT)

option(BUILD_POCOLIBS_SUPPORT "Build pocolibs framework support" ON)
if (BUILD_POCOLIBS_SUPPORT)
	pkg_check_modules(POCOLIBS REQUIRED pocolibs)
endif(BUILD_POCOLIBS_SUPPORT)



