
if(BUILD_CORE_SUPPORT)
INSTALL(FILES __init__.py environments.py
        DESTINATION ${PYTHON_INSTDIR}/morse
        )

INSTALL(DIRECTORY core actuators geolandloader helpers blender services robots testing
        DESTINATION ${PYTHON_INSTDIR}/morse
        REGEX .*py.$ EXCLUDE
        )

ADD_SUBDIRECTORY(builder)
ADD_SUBDIRECTORY(modifiers)
ADD_SUBDIRECTORY(sensors)
endif()

ADD_SUBDIRECTORY(multinode)
ADD_SUBDIRECTORY(middleware)
