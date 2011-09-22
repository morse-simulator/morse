if (BUILD_CORE_SUPPORT)
INSTALL(FILES __init__.py socket.py
	    DESTINATION ${PYTHON_INSTDIR}/morse/multinode
		)
endif(BUILD_CORE_SUPPORT)

IF(BUILD_HLA_SUPPORT)
    INSTALL(FILES hla.py
        DESTINATION ${PYTHON_INSTDIR}/morse/multinode)
    INSTALL(FILES morse.fed
        DESTINATION share/federations)
ENDIF(BUILD_HLA_SUPPORT)
