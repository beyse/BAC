cmake_minimum_required(VERSION 2.8)


set(ROBVIS_CMAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/../../robvis/cmake CACHE PATH "Directory that contains the robvis cmake directory.")
set(ROBVIS_SUBDIRECTORIES "" CACHE STRING "list of robvis subdirectories to add ('lib_a;lib_b;app_c'), use '*' for all libs and apps")


#no tests and wrappers for robvis libs
set(ROBVIS_GENERATE_TESTS OFF CACHE BOOL "When enabled generates unit test projects")
set(ROBVIS_GENERATE_TEST_RUNNERS OFF CACHE BOOL "When enabled generates auxilary projects that run compiled unit tests")
set(ROBVIS_GENERATE_EXAMPLES OFF CACHE BOOL "When enabled generates example projects")
set(ROBVIS_GENERATE_EXAMPLE_RUNNERS OFF CACHE BOOL "When enabled generates auxilary projects that run compiled examples")
set(ROBVIS_GENERATE_MATLAB_WRAPPERS OFF CACHE BOOL "When enabled generates 'mex' wrapper projects for matlab")
set(ROBVIS_ENABLE_WARNINGS_AS_ERRORS OFF CACHE BOOL "When enabled compiler warnings will be treated as errors")

#always copy (use "unload projects" to disable)
set(ROBVIS_GENERATE_COPY_EXTERNAL_PROJECTS_DEPENDENCY ON CACHE BOOL "When enabled automatically adds a dependency from the copy external binaries project to targets using it.")
set(ROBVIS_ENABLE_COPY_PROTECTION OFF CACHE BOOL "When enabled the default copy protection system will be installed to prevent unauthorized usage.")

#adding custom cmake functions
include(${ROBVIS_CMAKE_DIRECTORY}/functions/project_creators.cmake)
include(${ROBVIS_CMAKE_DIRECTORY}/functions/grouping.cmake)

#creating solution
robvis_create_toplevel_project(BAC)

add_subdirectory("${ROBVIS_CMAKE_DIRECTORY}/.." robvis)


#set( LIBCONCURRENCY ${CMAKE_SOURCE_DIR}/../ShowMe_ext/robvis/lib_concurrency/)
#add_subdirectory(${LIBCONCURRENCY} concurrency)

#adding group common

robvis_begin_targets_group("SimulationController")
add_subdirectory(SimulationController)