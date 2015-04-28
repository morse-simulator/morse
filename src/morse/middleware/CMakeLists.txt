macro(install_mw name)
	string(TOUPPER ${name} NAME)
	if(BUILD_${NAME}_SUPPORT)
		install(DIRECTORY ${name}
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware
				REGEX .*.py.$ EXCLUDE)
		
		file(GLOB ${NAME}_SRCS ${name}_*.py)
		install(FILES ${${NAME}_SRCS}
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware)
	endif()
endmacro()

if (BUILD_CORE_SUPPORT)
INSTALL(FILES __init__.py abstract_datastream.py socket_datastream.py text_datastream.py socket_request_manager.py 
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware
		)

INSTALL(DIRECTORY sockets
		 DESTINATION ${PYTHON_INSTDIR}/morse/middleware
		 REGEX .*.py.$ EXCLUDE)
endif()


IF(BUILD_POCOLIBS_SUPPORT)
	ADD_SUBDIRECTORY(pocolibs)
	INSTALL(FILES pocolibs_datastream.py pocolibs_request_manager.py
			DESTINATION ${PYTHON_INSTDIR}/morse/middleware)
ENDIF(BUILD_POCOLIBS_SUPPORT)

install_mw(yarp)
install_mw(ros)
install_mw(moos)
install_mw(hla)
