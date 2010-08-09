ADD_SUBDIRECTORY(General_Poster)
ADD_SUBDIRECTORY(Camera_Poster)
ADD_SUBDIRECTORY(Gyro_Poster)
ADD_SUBDIRECTORY(Sick_Poster)
ADD_SUBDIRECTORY(Viman_Poster)
INSTALL(FILES __init__.py viam.py viman.py pom.py
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/sensors
		)

