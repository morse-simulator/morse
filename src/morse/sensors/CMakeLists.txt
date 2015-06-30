#PYTHON_ADD_MODULE(sickc sickcmodule.c)
#INSTALL(TARGETS sickc DESTINATION ${PYTHON_INSTDIR}/morse/sensors)
		

PYTHON_ADD_MODULE(zbuffertodepth zbuffertodepth.c)
INSTALL(TARGETS zbuffertodepth DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

PYTHON_ADD_MODULE(zbufferto3d zbufferto3d.c)
INSTALL(TARGETS zbufferto3d DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

PYTHON_ADD_MODULE(_magnetometer magnetometer.c GeomagnetismLibrary.c)
INSTALL(TARGETS _magnetometer DESTINATION ${PYTHON_INSTDIR}/morse/sensors)
INSTALL(FILES WMM.COF DESTINATION ${CMAKE_INSTALL_PREFIX}/share/morse/data)

# install python files
FILE(GLOB PYTHON_FILES *.py)
INSTALL(FILES ${PYTHON_FILES} DESTINATION ${PYTHON_INSTDIR}/morse/sensors)

