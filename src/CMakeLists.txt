find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost CONFIG REQUIRED COMPONENTS serialization program_options)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)

file(GLOB_RECURSE COMOTION_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/comotion/*.cpp")

add_library(comotion STATIC ${COMOTION_SOURCES})
add_library(comotion::comotion ALIAS comotion)

target_include_directories(comotion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${FCL_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_directories(comotion PUBLIC
  "$<BUILD_INTERFACE:${FCL_LIBRARY_DIRS}>")

target_link_libraries(comotion PUBLIC
  ompl::ompl
  Eigen3::Eigen
  Boost::serialization
  Boost::program_options
  "$<BUILD_INTERFACE:nlohmann_json::nlohmann_json>"
  "$<INSTALL_INTERFACE:nlohmann_json::nlohmann_json>"
  "$<BUILD_INTERFACE:${FCL_LIBRARIES}>"
)

target_compile_definitions(comotion PUBLIC EIGEN_MAX_STATIC_ALIGN_BYTES=0)

function(comotion_apply_private_interface TARGET_NAME DEPENDENCY_NAME)
  if(NOT TARGET "${DEPENDENCY_NAME}")
    return()
  endif()

  get_target_property(_comotion_include_dirs "${DEPENDENCY_NAME}"
    INTERFACE_INCLUDE_DIRECTORIES)
  if(_comotion_include_dirs)
    target_include_directories("${TARGET_NAME}" PRIVATE ${_comotion_include_dirs})
  endif()

  get_target_property(_comotion_system_include_dirs "${DEPENDENCY_NAME}"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
  if(_comotion_system_include_dirs)
    target_include_directories("${TARGET_NAME}" SYSTEM PRIVATE
      ${_comotion_system_include_dirs})
  endif()

  get_target_property(_comotion_compile_options "${DEPENDENCY_NAME}"
    INTERFACE_COMPILE_OPTIONS)
  if(_comotion_compile_options)
    target_compile_options("${TARGET_NAME}" PRIVATE ${_comotion_compile_options})
  endif()

  get_target_property(_comotion_compile_definitions "${DEPENDENCY_NAME}"
    INTERFACE_COMPILE_DEFINITIONS)
  if(_comotion_compile_definitions)
    target_compile_definitions("${TARGET_NAME}" PRIVATE
      ${_comotion_compile_definitions})
  endif()

  get_target_property(_comotion_compile_features "${DEPENDENCY_NAME}"
    INTERFACE_COMPILE_FEATURES)
  if(_comotion_compile_features)
    target_compile_features("${TARGET_NAME}" PRIVATE ${_comotion_compile_features})
  endif()

  get_target_property(_comotion_link_libraries "${DEPENDENCY_NAME}"
    INTERFACE_LINK_LIBRARIES)
  if(_comotion_link_libraries)
    foreach(_comotion_link_library IN LISTS _comotion_link_libraries)
      if(TARGET "${_comotion_link_library}")
        comotion_apply_private_interface("${TARGET_NAME}" "${_comotion_link_library}")
      endif()
    endforeach()
  endif()
endfunction()

if(TARGET vamp_cpp)
  comotion_apply_private_interface(comotion vamp_cpp)
  target_compile_options(comotion PRIVATE
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wno-ignored-attributes>")
  target_compile_definitions(comotion PRIVATE COMOTION_HAVE_VAMP=1)
else()
  target_compile_definitions(comotion PRIVATE COMOTION_HAVE_VAMP=0)
endif()

include(CMakePackageConfigHelpers)

set(COMOTION_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/comotion"
  CACHE STRING "Install path for CoMotion CMake package files")

install(TARGETS comotion
  EXPORT comotionTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

set(COMOTION_PUBLIC_HEADERS
  comotion/collision/CollisionChecker.h
  comotion/collision/ConflictChecker.h
  comotion/collision/ObstacleShapes.h
  comotion/collision/ValidationTypes.h
  comotion/planning/AOARC.h
  comotion/planning/AORRTCUtils.h
  comotion/planning/ARC.h
  comotion/planning/CompositeAORRTC.h
  comotion/planning/CompositePRMStar.h
  comotion/planning/CompositeRRT.h
  comotion/planning/CompositeRRTStar.h
  comotion/planning/MRdRRT.h
  comotion/planning/MultiRobotPlanner.h
  comotion/planning/MultiRobotProblem.h
  comotion/planning/ParallelARC.h
  comotion/planning/Path.h
  comotion/planning/PathSimplification.h
  comotion/planning/PlanningRng.h
  comotion/planning/PlanningSeed.h
  comotion/planning/PrioritizedSTRRT.h
  comotion/planning/STCBS.h
  comotion/planning/USTRRTstar.h
  comotion/robot/FlyingSphere.h
  comotion/robot/IKSolver.h
  comotion/robot/RobotModel.h)

foreach(_comotion_public_header IN LISTS COMOTION_PUBLIC_HEADERS)
  get_filename_component(_comotion_header_dir
    "${_comotion_public_header}" DIRECTORY)
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/${_comotion_public_header}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${_comotion_header_dir}")
endforeach()

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/comotionConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/comotionConfig.cmake"
  INSTALL_DESTINATION "${COMOTION_INSTALL_CMAKEDIR}")

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/comotionConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

install(FILES
    "${PROJECT_BINARY_DIR}/comotionConfig.cmake"
    "${PROJECT_BINARY_DIR}/comotionConfigVersion.cmake"
  DESTINATION "${COMOTION_INSTALL_CMAKEDIR}")

install(EXPORT comotionTargets
  NAMESPACE comotion::
  DESTINATION "${COMOTION_INSTALL_CMAKEDIR}")
