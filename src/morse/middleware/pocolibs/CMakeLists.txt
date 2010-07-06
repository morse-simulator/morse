
ADD_SUBDIRECTORY(controllers)
ADD_SUBDIRECTORY(sensors)

INSTALL(FILES __init__.py
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
		)
