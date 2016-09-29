set(target SimulationController)

robvis_create_executable_project(${target})          
include(dependencies.cmake)                             
robvis_target_link_libraries(${target} ${dependencies}) 
