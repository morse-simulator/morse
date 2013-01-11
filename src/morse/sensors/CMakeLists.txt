#PYTHON_ADD_MODULE(sickc sickcmodule.c)
#INSTALL(TARGETS sickc DESTINATION ${PYTHON_INSTDIR}/morse/sensors)
		

PYTHON_ADD_MODULE(zbuffertodepth zbuffertodepth.c)
INSTALL(TARGETS zbuffertodepth DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

PYTHON_ADD_MODULE(zbufferto3d zbufferto3d.c)
INSTALL(TARGETS zbufferto3d DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

# install python files
FILE(GLOB PYTHON_FILES *.py)
INSTALL(FILES ${PYTHON_FILES} DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

