
if (PYMORSE_SUPPORT)
	install(DIRECTORY pymorse/src/pymorse
        DESTINATION ${PYTHON_INSTDIR}/pymorse
        REGEX .*py.$ EXCLUDE
        )
endif()

