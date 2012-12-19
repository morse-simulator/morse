##
## To add a sensor or an actuator to the build, just add it to the according
## list (actuator_list or sensor_list) and add the right entry in
## MODULES_MAP_{entry}
##
## It generates _ors_<module_name>_poster.so, ors_<module_name>_poster.py, and
## we wait for <module_name>.py in the (sensor|actuator) directory)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${POCOLIBS_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")

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

set(directories_list "sensors;actuators")
set(sensors_list "viam;pom;velodyne;viman;human_posture;target;platine_posture")
set(actuators_list "genpos;platine;lwr;niut")

## Set here the dependencies of your modules
## over Genom modules
set(MODULES_MAP_VIAM "viam;pom")
set(MODULES_MAP_POM "")
set(MODULES_MAP_VELODYNE "velodyne;pom")
set(MODULES_MAP_VIMAN "viman")
set(MODULES_MAP_HUMAN_POSTURE "")
set(MODULES_MAP_GENPOS "")
set(MODULES_MAP_TARGET "")
set(MODULES_MAP_PLATINE "")
set(MODULES_MAP_LWR "")
set(MODULES_MAP_NIUT "")
set(MODULES_MAP_PLATINE_POSTURE "")

foreach(directory ${directories_list})
	set(items_list "${directory}_list")
	foreach(item ${${items_list}})
		string(TOUPPER "${item}" ITEM)
		upcase_first(${item} Item)
		set(MODULE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${directory}/${Item}_Poster")
		set(module_to_find "MODULES_MAP_${ITEM}")
		set(modules_additional_to_find "MODULES_MAP_${ITEM}_ADDITIONAL")
		set(module_to_find ${${module_to_find}})
		set(target_name "")
		set(BUILD_MODULE_TO_FIND_SUPPORT "BUILD_POCOLIBS_${ITEM}_SUPPORT")
		option(${BUILD_MODULE_TO_FIND_SUPPORT} "build ${item} support" OFF)
		if (${BUILD_MODULE_TO_FIND_SUPPORT})

			foreach(dependency ${module_to_find})
				string(TOUPPER "${dependency}" DEPENDENCY)
				set(MODULE_INCLUDE_DIRS "${DEPENDENCY}_INCLUDE_DIRS")
				#MESSAGE("Adding dependency ${dependency}")
				pkg_check_modules(${DEPENDENCY} REQUIRED ${dependency})
				include_directories(${${MODULE_INCLUDE_DIRS}})
			endforeach(dependency)

			set(target_name "ors_${item}_poster")

		endif (${BUILD_MODULE_TO_FIND_SUPPORT})

		if((NOT "${module_to_find}" STREQUAL "") AND (${BUILD_MODULE_TO_FIND_SUPPORT}))
			include_directories(${MODULE_SOURCE_DIR})
			file(GLOB interface ${directory}/${Item}_Poster/*.i)
			file(GLOB source ${directory}/${Item}_Poster/*.c)
			
			set(ALL_INCLUDES)
			foreach(dependency ${module_to_find})
		        string(TOUPPER "${dependency}" DEPENDENCY)
				set(MODULE_INCLUDE_DIRS "${DEPENDENCY}_INCLUDE_DIRS")
				foreach (inc ${${MODULE_INCLUDE_DIRS}})
					set(ALL_INCLUDES ${ALL_INCLUDES} -I${inc})
				endforeach (inc ${${MODULE_INCLUDE_DIRS}})
			endforeach(dependency ${module_to_find})


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

			add_library(${target_name} SHARED ${source} ${CMAKE_CURRENT_BINARY_DIR}/${target_name}PYTHON_wrap.c)
			set_target_properties(${target_name} PROPERTIES PREFIX "_")
			set_target_properties(${target_name} PROPERTIES SUFFIX ".so")
			target_link_libraries(${target_name} ${POCOLIBS_LIBRARIES} ${PYTHON_LIBRARIES})
			install (FILES ${MODULE_SOURCE_DIR}/__init__.py 
				${CMAKE_CURRENT_BINARY_DIR}/_${target_name}.so 
				${CMAKE_CURRENT_BINARY_DIR}/${target_name}.py
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}/${Item}_Poster/
				)
		endif((NOT "${module_to_find}" STREQUAL "") AND (${BUILD_MODULE_TO_FIND_SUPPORT}))
		install(FILES ${directory}/${item}.py
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory})
	endforeach(item)
	install (FILES ${directory}/__init__.py
			 DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}
			 )
			
endforeach(directory)
install (FILES __init__.py
		DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
		)

install(DIRECTORY overlays
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
        REGEX .*py.$ EXCLUDE
        )

