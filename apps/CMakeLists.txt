add_executable(mobile_robot_2d_crossing
    mobile_robot_2d_crossing.cpp)
target_link_libraries(mobile_robot_2d_crossing PRIVATE comotion)

add_executable(planar_manipulator_cross
    planar_manipulator_cross.cpp)
target_link_libraries(planar_manipulator_cross PRIVATE comotion)

add_executable(panda_cage
    panda_cage.cpp)
target_link_libraries(panda_cage PRIVATE comotion)

add_executable(panda_flat
    panda_flat.cpp)
target_link_libraries(panda_flat PRIVATE comotion)

add_executable(heterogeneous_corridor
    heterogeneous_corridor.cpp)
target_link_libraries(heterogeneous_corridor PRIVATE comotion)

install(TARGETS
    mobile_robot_2d_crossing
    planar_manipulator_cross
    panda_cage
    panda_flat
    heterogeneous_corridor
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
