
INSTALL(FILES __init__.py init_middleware.py
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL}/morse
        )

INSTALL(DIRECTORY actuators geolandloader helpers robots sensors robots setup
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL}/morse
        REGEX .*py. EXCLUDE
        )

ADD_SUBDIRECTORY(middleware)
