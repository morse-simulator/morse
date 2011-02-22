cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME morse)
set(PACKAGE_VERSION 0.2.1)

option(BUILD_YARP2_SUPPORT "Build yarp framework support" OFF)
option(BUILD_POCOLIBS_SUPPORT "Build pocolibs framework support" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/)
find_package(PkgConfig REQUIRED)

find_package(Python31Interp REQUIRED)
find_package(Python31Libs REQUIRED)

EXECUTE_PROCESS(COMMAND
				${PYTHON3_EXECUTABLE} -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_python_lib(0,0,\"\"))"
					OUTPUT_VARIABLE PYTHON_INSTALL
					ERROR_VARIABLE PYTHON_STDERR
					RESULT_VARIABLE PYTHON_ERR
				)

SET(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL}) 
MESSAGE(STATUS "will install python files in ${PYTHON_INSTDIR}")

if (NOT PYTHON3INTERP_FOUND)
	MESSAGE(FATAL_ERROR "Can't find python 3.1 on your system")
endif(NOT PYTHON3INTERP_FOUND)
if (BUILD_YARP2_SUPPORT)
	find_package(YARP2 REQUIRED)

	EXECUTE_PROCESS(COMMAND 
					${PYTHON3_EXECUTABLE} -c "import yarp;"
					RESULT_VARIABLE YARP_PY_ERR
				   )
	if (YARP_PY_ERR)
		MESSAGE(FATAL_ERROR "BUILD_YARP_SUPPORT is required, but '${PYTHON3_EXECUTABLE}' can't find yarp python binding")
	endif (YARP_PY_ERR)
endif(BUILD_YARP2_SUPPORT)

if (BUILD_POCOLIBS_SUPPORT)
	pkg_check_modules(POCOLIBS REQUIRED pocolibs)
	find_package(SWIG REQUIRED)
endif(BUILD_POCOLIBS_SUPPORT)


ADD_SUBDIRECTORY(src)
INSTALL (DIRECTORY data examples
	     DESTINATION ${CMAKE_INSTALL_PREFIX}/share
         REGEX .*blend.$ EXCLUDE
         REGEX .*py.$ EXCLUDE
		 REGEX .*clients.* EXCLUDE
		)

CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/bin/morse.in
	${CMAKE_CURRENT_BINARY_DIR}/bin/morse
	)
INSTALL(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/bin/morse
		DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	    )


### Doc target ###
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/doc/conf.py.in
	${CMAKE_CURRENT_BINARY_DIR}/doc/conf.py
	)


FIND_PROGRAM(SPHINX_BUILD NAMES sphinx-build)
IF (NOT SPHINX_BUILD)
	MESSAGE(STATUS "Can't find sphinx-build : will not build the documentation")
ELSE()
	FIND_PACKAGE(PythonInterp)
	IF (NOT PYTHONINTERP_FOUND) 
		MESSAGE(STATUS "Can't find python2 executable : sphinx-build will not run")
		MESSAGE(STATUS "Will not build the documentation")
	ELSE()
		SET (SPHINX_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc/html)
		SET (SPHINX_LATEX_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc/tex)
		ADD_CUSTOM_COMMAND(
			OUTPUT ${SPHINX_OUTPUT}
			COMMAND ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html
								    -c ${CMAKE_CURRENT_BINARY_DIR}/doc
									   ${CMAKE_CURRENT_SOURCE_DIR}/doc/morse
									   ${SPHINX_OUTPUT}
		)
	
		ADD_CUSTOM_TARGET(doc ALL DEPENDS ${SPHINX_OUTPUT})
	
		INSTALL(DIRECTORY ${SPHINX_OUTPUT}
			    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/morse
				)
	
		ADD_CUSTOM_TARGET(
			force_doc
			COMMAND ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b html 
								 -c ${CMAKE_CURRENT_BINARY_DIR}/doc/
								 ${CMAKE_CURRENT_SOURCE_DIR}/doc/morse
								 ${SPHINX_OUTPUT}
		)
	
		ADD_CUSTOM_TARGET(
			latex_doc
			COMMAND ${PYTHON_EXECUTABLE} ${SPHINX_BUILD} -b latex
								 -c ${CMAKE_CURRENT_BINARY_DIR}/doc/
								 ${CMAKE_CURRENT_SOURCE_DIR}/doc/morse
								 ${SPHINX_LATEX_OUTPUT}
		)
	ENDIF()
ENDIF()


### Uninstall ###
configure_file("${CMAKE_HOME_DIRECTORY}/config/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")

