
if(BUILD_CORE_SUPPORT)
INSTALL(FILES __init__.py #init_middleware.py
        DESTINATION ${PYTHON_INSTDIR}/morse
        )

INSTALL(DIRECTORY core actuators geolandloader helpers blender sensors services robots testing
        DESTINATION ${PYTHON_INSTDIR}/morse
        REGEX .*py.$ EXCLUDE
        )

ADD_SUBDIRECTORY(builder)
ADD_SUBDIRECTORY(modifiers)
ADD_SUBDIRECTORY(sensors)
endif()

ADD_SUBDIRECTORY(multinode)
ADD_SUBDIRECTORY(middleware)
