# - Find python libraries
# This module finds if Python is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  PYTHONLIBS3_FOUND       - have the Python libs been found
#  PYTHON3_LIBRARIES       - path to the python library
#  PYTHON3_INCLUDE_DIRS    - path to where Python.h is found
#  PYTHON3_DEBUG_LIBRARIES - path to the debug library
#

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distributed this file outside of CMake, substitute the full
#  License text for the above reference.)

INCLUDE(CMakeFindFrameworks)
# Is there a python3 framework? How do we search for it?
# Search for the python framework on Apple.
# CMAKE_FIND_FRAMEWORKS(Python)

EXECUTE_PROCESS(COMMAND
					${PYTHON3_EXECUTABLE} -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_python_inc())"
					OUTPUT_VARIABLE PYTHON3_INCLUDE_DIR
					ERROR_VARIABLE PYTHON_STDERR
					RESULT_VARIABLE PYTHON_ERR
					)

if (WIN32)
EXECUTE_PROCESS(COMMAND
   ${PYTHON3_EXECUTABLE} -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_config_vars()['BINDIR'] + '\\libs\\python32.lib')"
                   OUTPUT_VARIABLE PYTHON3_LIBRARY
                   ERROR_VARIABLE PYTHON_STDERR
                   RESULT_VARIABLE PYTHON_ERR
                   )
else (WIN32)
EXECUTE_PROCESS(COMMAND
	${PYTHON3_EXECUTABLE} -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_config_vars()['LIBDIR'] + '/' + distutils.sysconfig.get_config_vars()['LDLIBRARY'])"
					OUTPUT_VARIABLE PYTHON3_LIBRARY
					ERROR_VARIABLE PYTHON_STDERR
					RESULT_VARIABLE PYTHON_ERR
					)
endif (WIN32)

if (NOT EXISTS ${PYTHON3_LIBRARY})
	message(FATAL_ERROR "PYTHON3_LIBRARY detected at ${PYTHON3_LIBRARY}, but does not seem to exist")
endif()

if (NOT EXISTS ${PYTHON3_INCLUDE_DIR}/Python.h)
	message(FATAL_ERROR "PYTHON3_INCLUDE_DIR detected at ${PYTHON3_INCLUDE_DIR}, "
						"but ${PYTHON3_INCLUDE_DIR}/Python.h does not seem to exist")
endif()

MARK_AS_ADVANCED(
  PYTHON3_DEBUG_LIBRARY
  PYTHON3_LIBRARY
  PYTHON3_INCLUDE_DIR
)

# We use PYTHON3_INCLUDE_DIR, PYTHON3_LIBRARY and PYTHON3_DEBUG_LIBRARY for the
# cache entries because they are meant to specify the location of a single
# library. We now set the variables listed by the documentation for this
# module.
SET(PYTHON3_INCLUDE_DIRS "${PYTHON3_INCLUDE_DIR}")
SET(PYTHON3_LIBRARIES "${PYTHON3_LIBRARY}")
SET(PYTHON3_DEBUG_LIBRARIES "${PYTHON3_DEBUG_LIBRARY}")


INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Python3Libs DEFAULT_MSG PYTHON3_LIBRARIES PYTHON3_INCLUDE_DIRS)


# PYTHON_ADD_MODULE(<name> src1 src2 ... srcN) is used to build modules for python.
# PYTHON_WRITE_MODULES_HEADER(<filename>) writes a header file you can include 
# in your sources to initialize the static python modules

GET_PROPERTY(_TARGET_SUPPORTS_SHARED_LIBS
  GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS)

FUNCTION(PYTHON3_ADD_MODULE _NAME )
  OPTION(PYTHON3_ENABLE_MODULE_${_NAME} "Add module ${_NAME}" TRUE)
  OPTION(PYTHON3_MODULE_${_NAME}_BUILD_SHARED
    "Add module ${_NAME} shared" ${_TARGET_SUPPORTS_SHARED_LIBS})

  # Mark these options as advanced
  MARK_AS_ADVANCED(PYTHON3_ENABLE_MODULE_${_NAME}
    PYTHON3_MODULE_${_NAME}_BUILD_SHARED)

  IF(PYTHON3_ENABLE_MODULE_${_NAME})
    IF(PYTHON3_MODULE_${_NAME}_BUILD_SHARED)
      SET(PY_MODULE_TYPE MODULE)
    ELSE(PYTHON3_MODULE_${_NAME}_BUILD_SHARED)
      SET(PY_MODULE_TYPE STATIC)
      SET_PROPERTY(GLOBAL  APPEND  PROPERTY  PY_STATIC_MODULES_LIST ${_NAME})
    ENDIF(PYTHON3_MODULE_${_NAME}_BUILD_SHARED)

    SET_PROPERTY(GLOBAL  APPEND  PROPERTY  PY_MODULES_LIST ${_NAME})
    ADD_LIBRARY(${_NAME} ${PY_MODULE_TYPE} ${ARGN})
#    TARGET_LINK_LIBRARIES(${_NAME} ${PYTHON_LIBRARIES})

  ENDIF(PYTHON3_ENABLE_MODULE_${_NAME})
ENDFUNCTION(PYTHON3_ADD_MODULE)

FUNCTION(PYTHON3_WRITE_MODULES_HEADER _filename)

  GET_PROPERTY(PY_STATIC_MODULES_LIST  GLOBAL  PROPERTY PY_STATIC_MODULES_LIST)

  GET_FILENAME_COMPONENT(_name "${_filename}" NAME)
  STRING(REPLACE "." "_" _name "${_name}")
  STRING(TOUPPER ${_name} _nameUpper)

  SET(_filenameTmp "${_filename}.in")
  FILE(WRITE ${_filenameTmp} "/*Created by cmake, do not edit, changes will be lost*/\n")
  FILE(APPEND ${_filenameTmp} 
"#ifndef ${_nameUpper}
#define ${_nameUpper}

#include <Python.h>

#ifdef __cplusplus
extern \"C\" {
#endif /* __cplusplus */

")

  FOREACH(_currentModule ${PY_STATIC_MODULES_LIST})
    FILE(APPEND ${_filenameTmp} "extern void init${PYTHON_MODULE_PREFIX}${_currentModule}(void);\n\n")
  ENDFOREACH(_currentModule ${PY_STATIC_MODULES_LIST})

  FILE(APPEND ${_filenameTmp} 
"#ifdef __cplusplus
}
#endif /* __cplusplus */

")


  FOREACH(_currentModule ${PY_STATIC_MODULES_LIST})
    FILE(APPEND ${_filenameTmp} "int ${_name}_${_currentModule}(void) \n{\n  static char name[]=\"${PYTHON_MODULE_PREFIX}${_currentModule}\"; return PyImport_AppendInittab(name, init${PYTHON_MODULE_PREFIX}${_currentModule});\n}\n\n")
  ENDFOREACH(_currentModule ${PY_STATIC_MODULES_LIST})

  FILE(APPEND ${_filenameTmp} "void ${_name}_LoadAllPythonModules(void)\n{\n")
  FOREACH(_currentModule ${PY_STATIC_MODULES_LIST})
    FILE(APPEND ${_filenameTmp} "  ${_name}_${_currentModule}();\n")
  ENDFOREACH(_currentModule ${PY_STATIC_MODULES_LIST})
  FILE(APPEND ${_filenameTmp} "}\n\n")
  FILE(APPEND ${_filenameTmp} "#ifndef EXCLUDE_LOAD_ALL_FUNCTION\nvoid CMakeLoadAllPythonModules(void)\n{\n  ${_name}_LoadAllPythonModules();\n}\n#endif\n\n#endif\n")
  
# with CONFIGURE_FILE() cmake complains that you may not use a file created using FILE(WRITE) as input file for CONFIGURE_FILE()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_filenameTmp}" "${_filename}" OUTPUT_QUIET ERROR_QUIET)

ENDFUNCTION(PYTHON3_WRITE_MODULES_HEADER)
