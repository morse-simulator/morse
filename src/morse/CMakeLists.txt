
INSTALL(FILES __init__.py #init_middleware.py
        DESTINATION ${PYTHON_INSTDIR}/morse
        )

INSTALL(DIRECTORY actuators geolandloader helpers blender sensors robots modifiers
        DESTINATION ${PYTHON_INSTDIR}/morse
        REGEX .*py. EXCLUDE
        )

ADD_SUBDIRECTORY(middleware)
