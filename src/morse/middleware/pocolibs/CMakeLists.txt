##
## To add a sensor or an actuator to the build, just add it to the according
## list (actuator_list or sensor_list) and add the right entry in
## MODULES_MAP_{entry}
##
## It generates _ors_<module_name>_poster.so, ors_<module_name>_poster.py, and
## we wait for <module_name>.py in the (sensor|actuator) directory)

include_directories(${PYTHON3_INCLUDE_DIR})
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
set(sensors_list "general;viam;pom;velodyne;viman;human_posture;target;platine_posture")
set(actuators_list "genpos;platine;lwr;niut")

## Set here the dependencies of your modules
## over Genom modules
set(MODULES_MAP_VIAM "viam;pom")
set(MODULES_MAP_GENERAL "")
set(MODULES_MAP_POM "pom")
set(MODULES_MAP_VELODYNE "velodyne;pom")
set(MODULES_MAP_VIMAN "viman")
set(MODULES_MAP_HUMAN_POSTURE "spark")
set(MODULES_MAP_GENPOS "genPos")
set(MODULES_MAP_TARGET "genPos")
set(MODULES_MAP_PLATINE "pom;platine")
set(MODULES_MAP_LWR "lwr;gbM;genBasic;genManip")
set(MODULES_MAP_NIUT "niut;genBasic")
set(MODULES_MAP_PLATINE_POSTURE "platine")

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
		if("${item}" STREQUAL "general")
			set(target_name "ors_poster")
		else("${module_to_find}" STREQUAL "")
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

		endif("${item}" STREQUAL "general")

		if(("${module_to_find}" STREQUAL "") OR (${BUILD_MODULE_TO_FIND_SUPPORT}))
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
			target_link_libraries(${target_name} ${POCOLIBS_LIBRARIES} ${PYTHON3_LIBRARIES})
			install (FILES ${MODULE_SOURCE_DIR}/__init__.py 
				${CMAKE_CURRENT_BINARY_DIR}/_${target_name}.so 
				${CMAKE_CURRENT_BINARY_DIR}/${target_name}.py
				DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs/${directory}/${Item}_Poster/
				)
		if(NOT "${module_to_find}" STREQUAL "")
		install(FILES ${directory}/${item}.py
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

install(DIRECTORY overlays
        DESTINATION ${PYTHON_INSTDIR}/morse/middleware/pocolibs
        REGEX .*py.$ EXCLUDE
        )

