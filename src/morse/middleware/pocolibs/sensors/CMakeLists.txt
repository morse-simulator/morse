ADD_SUBDIRECTORY(Camera_Poster)
ADD_SUBDIRECTORY(Gyro_Poster)
ADD_SUBDIRECTORY(Sick_Poster)
INSTALL(FILES __init__.py
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/sensors
		)

