PYTHON_ADD_MODULE(gaussian gaussian.c)
INSTALL(TARGETS gaussian
		DESTINATION ${PYTHON_INSTDIR}/morse/modifiers)

# install python files
FILE(GLOB PYTHON_FILES *.py)
INSTALL(FILES ${PYTHON_FILES}
        DESTINATION ${PYTHON_INSTDIR}/morse/modifiers)

