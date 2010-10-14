##
## To add a sensor or an actuator to the build, just add it to the according
## list (actuator_list or sensor_list) and add the right entry in
## MODULES_MAP_{entry}
##
## It generates _ors_<module_name>_poster.so, ors_<module_name>_poster.py, and
## we wait for <module_name>.py in the (sensor|actuator) directory)

include_directories(${PYTHON_INCLUDE_PATH})
include_directories(${POCOLIBS_INCLUDE_DIRS})

link_directories(${POCOLIBS_LIBDIR})
#macro that upcases the first letter of a given string all_lower
macro(upcase_first all_lower first_upper)
	string(SUBSTRING ${all_lower} 0 1 first_letter)
	string(TOUPPER "${first_letter}" first_letter)
	string(LENGTH ${all_lower} length)
	math(EXPR length "${length} - 1")
	string(SUBSTRING ${all_lower} 1 ${length} ${first_upper})
	set(${first_upper} "${first_letter}${${first_upper}}")
endmacro(upcase_first)

set(directories_list "sensors;controllers")
set(sensors_list "general;camera;gyro;sick;viman;human")
set(controllers_list "control")

set(MODULES_MAP_CAMERA "viam")
set(MODULES_MAP_GENERAL "")
set(MODULES_MAP_GYRO "pom")
set(MODULES_MAP_SICK "sick")
set(MODULES_MAP_VIMAN "viman")
set(MODULES_MAP_HUMAN "mhp")
set(MODULES_MAP_CONTROL "genPos")

foreach(directory ${directories_list})
	set(items_list "${directory}_list")
	foreach(item ${${items_list}})
		string(TOUPPER "${item}" ITEM)
		upcase_first(${item} Item)
		set(MODULE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${directory}/${Item}_Poster")
		set(module_to_find "MODULES_MAP_${ITEM}")
		set(module_to_find ${${module_to_find}})
		set(target_name "")
		if("${module_to_find}" STREQUAL "")
			set(target_name "ors_poster")
		else("${module_to_find}" STREQUAL "")
			string(TOUPPER "${module_to_find}" MODULE_TO_FIND)
			set(BUILD_MODULE_TO_FIND_SUPPORT "BUILD_POCOLIBS_${MODULE_TO_FIND}_SUPPORT")
			option(${BUILD_MODULE_TO_FIND_SUPPORT} "build ${module_to_find} support" OFF)
			if (${BUILD_MODULE_TO_FIND_SUPPORT})
				set(MODULE_INCLUDE_DIRS "${MODULE_TO_FIND}_INCLUDE_DIRS")
				pkg_check_modules(${MODULE_TO_FIND} REQUIRED ${module_to_find})
				include_directories(${${MODULE_INCLUDE_DIRS}})
				string(TOLOWER "${module_to_find}" module_to_find)
				set(target_name "ors_${module_to_find}_poster")
			endif (${BUILD_MODULE_TO_FIND_SUPPORT})
		endif("${module_to_find}" STREQUAL "")
		if(("${module_to_find}" STREQUAL "") OR (${BUILD_MODULE_TO_FIND_SUPPORT}))
			include_directories(${MODULE_SOURCE_DIR})
			file(GLOB interface ${directory}/${Item}_Poster/*.i)
			file(GLOB source ${directory}/${Item}_Poster/*.c)
			
			set(ALL_INCLUDES)
			foreach (inc ${${MODULE_INCLUDE_DIRS}})
				set(ALL_INCLUDES ${ALL_INCLUDES} -I${inc})
			endforeach (inc ${${MODULE_INCLUDE_DIRS}})

			add_custom_command(
				OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/${target_name}PYTHON_wrap.c 
				OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.py
				COMMAND ${SWIG_EXECUTABLE} 
				ARGS -python -outdir ${CMAKE_CURRENT_BINARY_DIR} 
									 -I${MODULE_SOURCE_DIR} 
									 ${ALL_INCLUDES} 
									 -o ${target_name}PYTHON_wrap.c ${interface}
				MAIN_DEPENDENCY ${interface}
				WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
				)

			add_library(${target_name} SHARED ${CMAKE_CURRENT_BINARY_DIR}/${target_name}PYTHON_wrap.c)
			set_target_properties(${target_name} PROPERTIES PREFIX "_")
			target_link_libraries(${target_name} ${POCOLIBS_LIBRARIES} ${PYTHON_LIBRARIES})
			install (FILES ${MODULE_SOURCE_DIR}/__init__.py 
				${CMAKE_CURRENT_BINARY_DIR}/_${target_name}.so 
				${CMAKE_CURRENT_BINARY_DIR}/${target_name}.py
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}/${Item}_Poster/
				)
		if(NOT "${module_to_find}" STREQUAL "")
		install(FILES ${directory}/${module_to_find}.py
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}
				)
		endif(NOT "${module_to_find}" STREQUAL "")
		endif(("${module_to_find}" STREQUAL "") OR (${BUILD_MODULE_TO_FIND_SUPPORT}))
	endforeach(item)
	install (FILES ${directory}/__init__.py
			 DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}
			 )
			
endforeach(directory)
install (FILES __init__.py
		DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
		)
