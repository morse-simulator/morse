
if (PYMORSE_SUPPORT)
	install(DIRECTORY pymorse/src/pymorse
        DESTINATION ${PYTHON_INSTDIR}
        REGEX .*py.$ EXCLUDE
        )
endif()

