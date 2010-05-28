
INSTALL(FILES __init__.py init_middleware.py
        DESTINATION ${PYTHON_INSTDIR}/morse
        )

INSTALL(DIRECTORY actuators geolandloader helpers robots sensors robots setup
        DESTINATION ${PYTHON_INSTDIR}/morse
        REGEX .*py. EXCLUDE
        )

ADD_SUBDIRECTORY(middleware)
