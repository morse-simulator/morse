
INSTALL(FILES __init__.py #init_middleware.py
        DESTINATION ${PYTHON_INSTDIR}/morse
        )

INSTALL(DIRECTORY actuators helpers blender sensors robots 
        DESTINATION ${PYTHON_INSTDIR}/morse
        REGEX .*py. EXCLUDE
        )

ADD_SUBDIRECTORY(middleware)
ADD_SUBDIRECTORY(modifiers)
