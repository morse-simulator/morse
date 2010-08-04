ADD_SUBDIRECTORY(Control_Poster)
INSTALL(FILES __init__.py genpos.py
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/controllers
		)

