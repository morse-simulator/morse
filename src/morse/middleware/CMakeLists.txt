
if (BUILD_CORE_SUPPORT)
INSTALL(FILES __init__.py abstract_datastream.py socket_datastream.py text_datastream.py socket_request_manager.py 
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware
		)

INSTALL(DIRECTORY sockets
		 DESTINATION ${PYTHON_INSTDIR}/morse/middleware)
endif()

IF(BUILD_YARP2_SUPPORT)
	INSTALL(DIRECTORY yarp
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware
        REGEX .*py.$ EXCLUDE
        )
	INSTALL(FILES yarp_datastream.py yarp_request_manager.py yarp_json_request_manager.py
		DESTINATION ${PYTHON_INSTDIR}/morse/middleware
		)
ENDIF(BUILD_YARP2_SUPPORT)

IF(BUILD_POCOLIBS_SUPPORT)
	ADD_SUBDIRECTORY(pocolibs)
	INSTALL(FILES pocolibs_datastream.py pocolibs_request_manager.py
			DESTINATION ${PYTHON_INSTDIR}/morse/middleware)
ENDIF(BUILD_POCOLIBS_SUPPORT)

IF(BUILD_ROS_SUPPORT)
  INSTALL(DIRECTORY ros
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware
        REGEX .*py.$ EXCLUDE
        )
  INSTALL(FILES ros_datastream.py ros_request_manager.py
			DESTINATION ${PYTHON_INSTDIR}/morse/middleware
      )
ENDIF(BUILD_ROS_SUPPORT)

IF(BUILD_MOOS_SUPPORT)
  INSTALL(DIRECTORY moos
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware
        REGEX .*py.$ EXCLUDE
        )
  INSTALL(FILES moos_datastream.py 
	    DESTINATION ${PYTHON_INSTDIR}/morse/middleware
		)
ENDIF(BUILD_MOOS_SUPPORT)
