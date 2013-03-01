##
## To add a sensor or an actuator to the build, just add it to the according
## list (actuator_list or sensor_list) and add the right entry in
## MODULES_MAP_{entry}
##
## It generates _ors_<module_name>_poster.so, ors_<module_name>_poster.py, and
## we wait for <module_name>.py in the (sensor|actuator) directory)

option(BUILD_POCOLIBS_VIAM_SUPPORT "build viam support" OFF)
option(BUILD_POCOLIBS_STEREOPIXEL_SUPPORT "build image3D support" OFF)

if (BUILD_POCOLIBS_VIAM_SUPPORT)
	pkg_check_modules(POCOLIBS REQUIRED pocolibs)
	include_directories(${POCOLIBS_INCLUDE_DIRS})
	link_directories(${POCOLIBS_LIBRARY_DIRS})

	pkg_check_modules(VIAM REQUIRED viam)
	include_directories(${VIAM_INCLUDE_DIRS})

	python_add_module(viam sensors/viam.c)
	target_link_libraries(viam ${POCOLIBS_LIBRARIES})
	install(TARGETS viam DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs)
endif()

if (BUILD_POCOLIBS_STEREOPIXEL_SUPPORT)
	pkg_check_modules(POCOLIBS REQUIRED pocolibs)
	include_directories(${POCOLIBS_INCLUDE_DIRS})
	link_directories(${POCOLIBS_LIBRARY_DIRS})

	pkg_check_modules(STEREOPIXELE REQUIRED stereopixel)
	include_directories(${VIAM_INCLUDE_DIRS})

	python_add_module(stereopixel sensors/stereopixel.c)
	target_link_libraries(stereopixel ${POCOLIBS_LIBRARIES})
	install(TARGETS stereopixel DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs)
endif()

install (FILES __init__.py
		DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
		)

install(DIRECTORY overlays sensors actuators
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
		FILES_MATCHING PATTERN "*.py"
        )

