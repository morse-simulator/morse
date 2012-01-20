
if (PYMORSE_SUPPORT)
	install(DIRECTORY pymorse/
        DESTINATION ${PYTHON_INSTDIR}
        REGEX .*py.$ EXCLUDE
        REGEX setup.py EXCLUDE
        )
endif()

