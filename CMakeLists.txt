cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME morse)
set(PACKAGE_VERSION "1.0.99")

cmake_policy(SET CMP0017 OLD)



option(BUILD_CORE_SUPPORT "Build and install morse core" ON)
option(BUILD_DOC_SUPPORT "Build and install morse documentation (Requiere Sphinx)" OFF)

option(BUILD_POCOLIBS_SUPPORT "Build pocolibs framework support" OFF)
option(BUILD_YARP2_SUPPORT "Build yarp framework support" OFF)
option(BUILD_ROS_SUPPORT "Build ROS middleware support" OFF)
option(BUILD_MOOS_SUPPORT "Build MOOS middleware support" OFF)
option(BUILD_HLA_SUPPORT "Build HLA middleware support. Also used for distributed HLA simulation." OFF)

option(PYMORSE_SUPPORT "Build and install the Python bindings for MORSE." OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/)
find_package(PkgConfig REQUIRED)

set(PythonInterp_FIND_VERSION 3.2)
find_package(PythonInterp REQUIRED)

set(PythonLibs_FIND_VERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
set(PythonLibs_FIND_VERSION_EXACT "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs REQUIRED)

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

EXECUTE_PROCESS(COMMAND
					${PYTHON_EXECUTABLE} -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_python_lib(1,0,\"\"))"
					OUTPUT_VARIABLE PYTHON_INSTALL
					ERROR_VARIABLE PYTHON_STDERR
					RESULT_VARIABLE PYTHON_ERR
				)
if(NOT PYTHON_INSTDIR)
	SET(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL}) 
endif(NOT PYTHON_INSTDIR)
if(WIN32)
    STRING(REGEX REPLACE "\\\\" "/" PYTHON_INSTDIR ${PYTHON_INSTDIR})
endif(WIN32)
MESSAGE(STATUS "will install python files in ${PYTHON_INSTDIR}")

# Compute revision
ADD_CUSTOM_TARGET(revisiontag ALL)
ADD_CUSTOM_COMMAND(TARGET revisiontag COMMAND ${CMAKE_COMMAND}
	   -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
	   -P ${CMAKE_CURRENT_SOURCE_DIR}/config/version.cmake)

if (BUILD_CORE_SUPPORT)
	INSTALL(FILES ${CMAKE_BINARY_DIR}/version.py DESTINATION ${PYTHON_INSTDIR}/morse/)
endif(BUILD_CORE_SUPPORT)

if (BUILD_YARP2_SUPPORT)
	EXECUTE_PROCESS(COMMAND 
					${PYTHON_EXECUTABLE} -c "import yarp;"
					RESULT_VARIABLE YARP_PY_ERR
				   )
	if (YARP_PY_ERR)
		MESSAGE(FATAL_ERROR "BUILD_YARP_SUPPORT is required, but '${PYTHON_EXECUTABLE}' can't find yarp python binding")
	endif (YARP_PY_ERR)
endif(BUILD_YARP2_SUPPORT)


# check to make sure pymoos is installed
if (BUILD_MOOS_SUPPORT)
	EXECUTE_PROCESS(COMMAND 
					${PYTHON_EXECUTABLE} -c "import pymoos.MOOSCommClient;"
					RESULT_VARIABLE MOOS_PY_ERR
				   )
	if (MOOS_PY_ERR)
		#message(${MOOS_PY_ERR})
		#MESSAGE(FATAL_ERROR "BUILD_MOOS_SUPPORT is required, but '${PYTHON_EXECUTABLE}' can't find MOOS python binding")
	endif (MOOS_PY_ERR)
endif(BUILD_MOOS_SUPPORT)

if (BUILD_HLA_SUPPORT)
	EXECUTE_PROCESS(COMMAND 
					${PYTHON_EXECUTABLE} -c "import hla;"
					RESULT_VARIABLE HLA_PY_ERR
				   )
	if (HLA_PY_ERR)
		MESSAGE(FATAL_ERROR "BUILD_HLA_SUPPORT is required, but '${PYTHON_EXECUTABLE}' can't find 'hla' python binding")
	endif (HLA_PY_ERR)
endif(BUILD_HLA_SUPPORT)

ADD_SUBDIRECTORY(src)

IF(BUILD_CORE_SUPPORT)
INSTALL (DIRECTORY data examples addons
	     DESTINATION ${CMAKE_INSTALL_PREFIX}/share/morse
         REGEX .*blend.$ EXCLUDE
         REGEX .*py.$ EXCLUDE
		 REGEX .*clients.* EXCLUDE
		)

IF(WIN32)
CONFIGURE_FILE(
   ${CMAKE_CURRENT_SOURCE_DIR}/bin/morse.in
   ${CMAKE_CURRENT_BINARY_DIR}/bin/morserun.py
   )
CONFIGURE_FILE(
   ${CMAKE_CURRENT_SOURCE_DIR}/bin/multinode_server.in
   ${CMAKE_CURRENT_BINARY_DIR}/bin/multinode_server.py
   )
INSTALL(PROGRAMS
       ${CMAKE_CURRENT_BINARY_DIR}/bin/morserun.py
       ${CMAKE_CURRENT_BINARY_DIR}/bin/multinode_server.py
       DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       )
ELSE(WIN32)
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/bin/morse.in
	${CMAKE_CURRENT_BINARY_DIR}/bin/morse
	)
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/bin/morseexec.in
	${CMAKE_CURRENT_BINARY_DIR}/bin/morseexec
	)
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/bin/multinode_server.in
	${CMAKE_CURRENT_BINARY_DIR}/bin/multinode_server
	)
INSTALL(PROGRAMS
		${CMAKE_CURRENT_BINARY_DIR}/bin/morse
		${CMAKE_CURRENT_BINARY_DIR}/bin/morseexec
		${CMAKE_CURRENT_BINARY_DIR}/bin/multinode_server
		DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	    )
ENDIF(WIN32)
ENDIF()

ADD_SUBDIRECTORY(bindings)

###### Testing ######
enable_testing()
ADD_SUBDIRECTORY(testing)

IF (BUILD_DOC_SUPPORT)
### Doc target ###
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/doc/conf.py.in
	${CMAKE_CURRENT_BINARY_DIR}/doc/conf.py
	)

FIND_PROGRAM(SPHINX_BUILD NAMES sphinx-build)
IF (NOT SPHINX_BUILD)
	MESSAGE(FATAL_ERROR "Can't find sphinx-build : will not build the documentation")
ELSE()
	SET (CMAKE_DOC_OUT ${CMAKE_CURRENT_BINARY_DIR}/doc)
	SET (SPHINX_OUTPUT ${CMAKE_DOC_OUT}/html)
	SET (SPHINX_SRC ${CMAKE_DOC_OUT}/src)
	SET (SPHINX_LATEX_OUTPUT ${CMAKE_DOC_OUT}/tex)
	SET (SPHINX_MAN_OUTPUT ${CMAKE_DOC_OUT}/man)
	SET (SPHINX_FALSE_PYTHON_ENV ${CMAKE_CURRENT_BINARY_DIR}/fakeenv)
	SET (PYTHON_DOC_ENV env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/src:${SPHINX_FALSE_PYTHON_ENV}:${CMAKE_CURRENT_SOURCE_DIR}/src:${CMAKE_CURRENT_SOURCE_DIR}/testing:${CMAKE_CURRENT_SOURCE_DIR}/bindings/pymorse/src/:$$PYTHONPATH PYTHONDONTWRITEBYTECODE="morse" LANG="en_US.UTF-8" LC_CTYPE="en_US.UTF-8")
	SET (SPHINX_BUILD_ENV ${PYTHON_DOC_ENV} ${SPHINX_BUILD})

	# Copy the documentation in builddir, so we can include the generated part too
	ADD_CUSTOM_TARGET(
		copy_doc ALL
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/doc/morse ${SPHINX_SRC}/morse
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/doc/media ${SPHINX_SRC}/media
	)

	# Generate the documentation from the different components
	ADD_CUSTOM_TARGET(
		generate_doc ALL
		COMMAND ${PYTHON_DOC_ENV} ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doc/tools/generate_doc
		WORKING_DIRECTORY ${SPHINX_SRC}/morse/user/
	)

	ADD_DEPENDENCIES(generate_doc copy_doc ${ALL_C_PYTHON_MODULES})

	# Populate the false python environment
	ADD_CUSTOM_TARGET(
		populate_false_env ALL
		COMMAND ${CMAKE_COMMAND} -E make_directory ${SPHINX_FALSE_PYTHON_ENV}
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/mathutils.py
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/bge.py
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/PhysicsConstraints.py

		COMMAND ${CMAKE_COMMAND} -E make_directory ${SPHINX_FALSE_PYTHON_ENV}/hla
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/hla/__init__.py

		COMMAND ${CMAKE_COMMAND} -E make_directory ${SPHINX_FALSE_PYTHON_ENV}/pymoos
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/pymoos/__init__.py
		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/pymoos/MOOSCommClient.py

		COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/yarp.py
	)

	FILE(WRITE ${SPHINX_FALSE_PYTHON_ENV}/hla/omt.py "HLAfloat32LE=0\ndef HLAfixedArray(x,y,z):\n    pass")
	FILE(WRITE ${SPHINX_FALSE_PYTHON_ENV}/hla/rti.py "class FederateAmbassador:\n    pass")

	# GeNoM fake stuff
	FOREACH(lib genPos lwr niut pom platine spark stereopixel viam viman)
		ADD_CUSTOM_TARGET(
			populate_false_env_${lib} ALL
			COMMAND ${CMAKE_COMMAND} -E make_directory ${SPHINX_FALSE_PYTHON_ENV}/${lib}
			COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/${lib}/struct.py
			COMMAND ${CMAKE_COMMAND} -E touch ${SPHINX_FALSE_PYTHON_ENV}/${lib}/__init__.py
		)
		ADD_DEPENDENCIES(generate_doc populate_false_env_${lib})
	ENDFOREACH()


	ADD_DEPENDENCIES(copy_doc revisiontag ${SPHINX_FALSE_PYTHON_ENV}/hla/omt.py)


	FIND_PROGRAM(SPHINX_APIDOC NAMES sphinx-apidoc)
	IF (SPHINX_APIDOC)
		ADD_CUSTOM_TARGET(
			generate_api_doc ALL
			COMMAND ${PYTHON_DOC_ENV} ${SPHINX_APIDOC} -f -o ${SPHINX_SRC}/morse/user/code ${CMAKE_CURRENT_SOURCE_DIR}/src
			COMMAND ${PYTHON_DOC_ENV} ${SPHINX_APIDOC} -f -o ${SPHINX_SRC}/morse/user/code ${CMAKE_CURRENT_SOURCE_DIR}/testing)
	ELSE()
		ADD_CUSTOM_TARGET(
			generate_api_doc ALL
			COMMAND ${CMAKE_COMMAND} -E echo "sphinx-apidoc not found : skipping api documentation"
		)
	ENDIF()
	ADD_DEPENDENCIES(generate_api_doc copy_doc populate_false_env)

	# Generate API doc if we have the sphinx-api tool

	ADD_CUSTOM_TARGET(
		doc ALL
		COMMAND ${SPHINX_BUILD_ENV} -b html
								    -c ${CMAKE_DOC_OUT}
									${SPHINX_SRC}/morse
								    ${SPHINX_OUTPUT}
	)
	ADD_DEPENDENCIES(doc generate_doc generate_api_doc)

	FIND_PROGRAM(GZIP NAMES gzip)
	IF (NOT GZIP)
		MESSAGE(STATUS "Can't find gzip : will not build man pages")
	ELSE()
		ADD_CUSTOM_TARGET(man ALL COMMAND ${SPHINX_BUILD_ENV} -b man
									-c ${CMAKE_CURRENT_BINARY_DIR}/doc
									   ${CMAKE_CURRENT_SOURCE_DIR}/doc/man
        								   ${SPHINX_MAN_OUTPUT} &&
						  ${GZIP} -f ${SPHINX_MAN_OUTPUT}/*.1)
		INSTALL(FILES ${SPHINX_MAN_OUTPUT}/morse.1.gz 
					  ${SPHINX_MAN_OUTPUT}/morseexec.1.gz 
					  ${SPHINX_MAN_OUTPUT}/multinode_server.1.gz
			DESTINATION ${CMAKE_INSTALL_PREFIX}/share/man/man1
		)
	ENDIF()

	INSTALL(DIRECTORY ${SPHINX_OUTPUT}
			DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/morse
			REGEX .doctrees.* EXCLUDE
			REGEX _sources.* EXCLUDE
			REGEX .*.inv EXCLUDE
			REGEX .buildinfo EXCLUDE
			)

	ADD_CUSTOM_TARGET(
		latex_doc 
		COMMAND ${SPHINX_BUILD_ENV} -b latex
									-c ${CMAKE_DOC_OUT}
									${SPHINX_SRC}/morse
									${SPHINX_LATEX_OUTPUT}
	)
ENDIF()
ENDIF()

### CPack ###
EXECUTE_PROCESS(COMMAND
				which dpkg
					OUTPUT_VARIABLE SYSTEST_DPKG)
EXECUTE_PROCESS(COMMAND
				which rpm
					OUTPUT_VARIABLE SYSTEST_RPM)
IF (SYSTEST_DPKG)
	SET(CPACK_GENERATOR "DEB")
ELSEIF (SYSTEST_RPM)
	SET(CPACK_GENERATOR "RPM")
ELSE ()
	SET(CPACK_GENERATOR "TGZ")
ENDIF()
# TODO find a better way to check debian or redhat ?

SET(CPACK_PACKAGE_CONTACT "morse-users <morse-users@laas.fr>")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MORSE the Open Robots Simulator")
SET(CPACK_PACKAGE_NAME "openrobots-morse")
SET(CPACK_PACKAGE_VENDOR "CNRS/LAAS")
SET(CPACK_PACKAGE_VERSION ${PACKAGE_VERSION})
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README")

SET(CPACK_DEBIAN_PACKAGE_DEPENDS "python3.2-dev")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://morse.openrobots.org")

INCLUDE(CPack)

### Uninstall ###
configure_file("${CMAKE_HOME_DIRECTORY}/config/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")

