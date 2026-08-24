cmake_minimum_required(VERSION 3.14)
project(comotion
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "CoMotion: Coordinated Motion Planning for Multi-Robot Systems"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(COMOTION_STANDALONE_DEFAULT ON)
else()
  set(COMOTION_STANDALONE_DEFAULT OFF)
endif()

option(COMOTION_BUILD_APPS "Build standalone CoMotion workload applications"
  ${COMOTION_STANDALONE_DEFAULT})
option(COMOTION_BUILD_EXAMPLES "Build small downstream library-use examples"
  ${COMOTION_STANDALONE_DEFAULT})
option(COMOTION_BUILD_TESTS "Build CoMotion regression tests"
  ${COMOTION_STANDALONE_DEFAULT})
option(COMOTION_FETCH_NLOHMANN_JSON
  "Fetch nlohmann/json if it is not available as a CMake package" ON)

# ---------------------------------------------------------------------------
# Third-party: nlohmann/json
# ---------------------------------------------------------------------------
find_package(nlohmann_json 3.11.3 QUIET)
if(NOT nlohmann_json_FOUND)
  if(NOT COMOTION_FETCH_NLOHMANN_JSON)
    message(FATAL_ERROR
      "nlohmann_json was not found. Install it or enable COMOTION_FETCH_NLOHMANN_JSON.")
  endif()
  include(FetchContent)
  set(JSON_Install ON CACHE BOOL "Install nlohmann/json package files" FORCE)
  FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(json)
endif()

# ---------------------------------------------------------------------------
# OMPL (fork): build and use as in-tree dependency
# ---------------------------------------------------------------------------
set(COMOTION_OMPL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/como-ompl")
set(COMOTION_VAMP_SOURCE_DIR "${COMOTION_OMPL_SOURCE_DIR}/external/vamp")

set(OMPL_BUILD_VAMP ON CACHE BOOL "Build OMPL with its bundled VAMP submodule enabled" FORCE)
set(OMPL_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Disable Python bindings")
set(OMPL_BUILD_DEMOS OFF CACHE BOOL "We build our own demos; skip OMPL demos")
set(OMPL_BUILD_TESTS OFF CACHE BOOL "Skip OMPL tests in this project")
set(OMPL_INSTALL_BENCHMARK_TOOLS OFF CACHE BOOL "Skip OMPL benchmark helper scripts in this project")
set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Skip VAMP Python bindings in this project" FORCE)

add_subdirectory("${COMOTION_OMPL_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/external/como-ompl")

# ---------------------------------------------------------------------------
# CoMotion library
# ---------------------------------------------------------------------------
add_subdirectory(src)

# ---------------------------------------------------------------------------
# Applications / examples
# ---------------------------------------------------------------------------
if(COMOTION_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(COMOTION_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(COMOTION_BUILD_TESTS)
    enable_testing()

    add_executable(public_header_smoke
        tests/public_header_smoke.cpp)
    target_link_libraries(public_header_smoke PRIVATE comotion)
    add_test(NAME public_header_smoke COMMAND public_header_smoke)

    add_executable(fcl_urdf_geometry_regression
        tests/fcl_urdf_geometry_regression.cpp)
    target_link_libraries(fcl_urdf_geometry_regression PRIVATE comotion)
    add_test(NAME fcl_urdf_geometry_regression
             COMMAND fcl_urdf_geometry_regression)
    set_tests_properties(fcl_urdf_geometry_regression PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

    add_executable(path_timestep_semantics
        tests/path_timestep_semantics.cpp)
    target_link_libraries(path_timestep_semantics PRIVATE comotion)
    add_test(NAME path_timestep_semantics COMMAND path_timestep_semantics)

    add_executable(mobile_robot_2d_crossing_geometry
        benchmarks/tests/mobile_robot_2d_crossing_geometry.cpp)
    target_include_directories(mobile_robot_2d_crossing_geometry PRIVATE
        ${PROJECT_SOURCE_DIR}/apps)
    target_link_libraries(mobile_robot_2d_crossing_geometry PRIVATE comotion)
    add_test(NAME mobile_robot_2d_crossing_geometry
             COMMAND mobile_robot_2d_crossing_geometry)

    add_executable(planar_manipulator_cross_geometry
        benchmarks/tests/planar_manipulator_cross_geometry.cpp)
    target_include_directories(planar_manipulator_cross_geometry PRIVATE
        ${PROJECT_SOURCE_DIR}/apps)
    target_link_libraries(planar_manipulator_cross_geometry PRIVATE comotion)
    add_test(NAME planar_manipulator_cross_geometry
             COMMAND planar_manipulator_cross_geometry)
    set_tests_properties(planar_manipulator_cross_geometry PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

    add_executable(panda_flat_geometry
        benchmarks/tests/panda_flat_geometry.cpp)
    target_include_directories(panda_flat_geometry PRIVATE
        ${PROJECT_SOURCE_DIR}/apps)
    target_link_libraries(panda_flat_geometry PRIVATE comotion)
    add_test(NAME panda_flat_geometry COMMAND panda_flat_geometry)
    set_tests_properties(panda_flat_geometry PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

    add_executable(prioritized_strrt_goal_hold_regression
        tests/prioritized_strrt_goal_hold_regression.cpp)
    target_link_libraries(prioritized_strrt_goal_hold_regression PRIVATE comotion)
    add_test(NAME prioritized_strrt_goal_hold_regression
             COMMAND prioritized_strrt_goal_hold_regression)

    add_executable(composite_rrt_endpoint_regression
        tests/composite_rrt_endpoint_regression.cpp)
    target_link_libraries(composite_rrt_endpoint_regression PRIVATE comotion)
    add_test(NAME composite_rrt_endpoint_regression
             COMMAND composite_rrt_endpoint_regression)

    add_executable(composite_rrt_seed_regression
        tests/composite_rrt_seed_regression.cpp)
    target_link_libraries(composite_rrt_seed_regression PRIVATE comotion)
    add_test(NAME composite_rrt_seed_regression
             COMMAND composite_rrt_seed_regression)

    add_executable(composite_rrtstar_regression
        tests/composite_rrtstar_regression.cpp)
    target_link_libraries(composite_rrtstar_regression PRIVATE comotion)
    add_test(NAME composite_rrtstar_regression
             COMMAND composite_rrtstar_regression)

    add_executable(composite_prmstar_regression
        tests/composite_prmstar_regression.cpp)
    target_link_libraries(composite_prmstar_regression PRIVATE comotion)
    add_test(NAME composite_prmstar_regression
             COMMAND composite_prmstar_regression)

    add_executable(composite_rrt_approximate_rejection
        tests/composite_rrt_approximate_rejection.cpp)
    target_link_libraries(composite_rrt_approximate_rejection PRIVATE comotion)
    add_test(NAME composite_rrt_approximate_rejection
             COMMAND composite_rrt_approximate_rejection)

    add_executable(cooperative_composite_rrt_regression
        tests/cooperative_composite_rrt_regression.cpp)
    target_link_libraries(cooperative_composite_rrt_regression PRIVATE comotion)
    add_test(NAME cooperative_composite_rrt_regression
             COMMAND cooperative_composite_rrt_regression)

    add_executable(arc_exact_only_regression
        tests/arc_exact_only_regression.cpp)
    target_link_libraries(arc_exact_only_regression PRIVATE comotion)
    add_test(NAME arc_exact_only_regression
             COMMAND arc_exact_only_regression)

    add_executable(aorrtc_makespan_regression
        tests/aorrtc_makespan_regression.cpp)
    target_link_libraries(aorrtc_makespan_regression PRIVATE comotion)
    add_test(NAME aorrtc_makespan_regression
             COMMAND aorrtc_makespan_regression)

    add_executable(find_conflict_pairs_regression
        tests/find_conflict_pairs_regression.cpp)
    target_link_libraries(find_conflict_pairs_regression PRIVATE comotion)
    if(TARGET vamp_cpp)
        target_compile_definitions(find_conflict_pairs_regression PRIVATE
            COMOTION_HAVE_VAMP=1)
    else()
        target_compile_definitions(find_conflict_pairs_regression PRIVATE
            COMOTION_HAVE_VAMP=0)
    endif()
    add_test(NAME find_conflict_pairs_regression
             COMMAND find_conflict_pairs_regression)

    add_executable(parallel_arc_regression
        tests/parallel_arc_regression.cpp)
    target_link_libraries(parallel_arc_regression PRIVATE comotion)
    if(TARGET vamp_cpp)
        target_compile_definitions(parallel_arc_regression PRIVATE
            COMOTION_HAVE_VAMP=1)
    else()
        target_compile_definitions(parallel_arc_regression PRIVATE
            COMOTION_HAVE_VAMP=0)
    endif()
    add_test(NAME parallel_arc_regression COMMAND parallel_arc_regression)

    add_executable(or_parallel_planner_regression
        tests/or_parallel_planner_regression.cpp)
    target_link_libraries(or_parallel_planner_regression PRIVATE comotion)
    add_test(NAME or_parallel_planner_regression
             COMMAND or_parallel_planner_regression)

    add_executable(stcbs_regression
        tests/stcbs_regression.cpp)
    target_link_libraries(stcbs_regression PRIVATE comotion)
    add_test(NAME stcbs_regression COMMAND stcbs_regression)

    add_executable(mr_drrt_metrics_regression
        tests/mr_drrt_metrics_regression.cpp)
    target_link_libraries(mr_drrt_metrics_regression PRIVATE comotion)
    add_test(NAME mr_drrt_metrics_regression COMMAND mr_drrt_metrics_regression)

    add_executable(stcbs_post_arrival_repro
        tests/stcbs_post_arrival_repro.cpp)
    target_link_libraries(stcbs_post_arrival_repro PRIVATE comotion)
    add_test(NAME stcbs_post_arrival_repro
             COMMAND stcbs_post_arrival_repro)

    if(TARGET vamp_cpp)
        add_executable(vamp_cylinder_regression
            tests/vamp_cylinder_regression.cpp)
        target_link_libraries(vamp_cylinder_regression PRIVATE comotion vamp_cpp)
        target_include_directories(vamp_cylinder_regression PRIVATE
            ${COMOTION_VAMP_SOURCE_DIR}/src/impl)
        add_test(NAME vamp_cylinder_regression COMMAND vamp_cylinder_regression)

        add_executable(vamp_panda_cage_regression
            tests/vamp_panda_cage_regression.cpp)
        target_link_libraries(vamp_panda_cage_regression PRIVATE comotion)
        add_test(NAME vamp_panda_cage_regression COMMAND vamp_panda_cage_regression)
        set_tests_properties(vamp_panda_cage_regression PROPERTIES
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

        add_executable(vamp_planar3_regression
            tests/vamp_planar3_regression.cpp)
        target_link_libraries(vamp_planar3_regression PRIVATE comotion vamp_cpp)
        target_include_directories(vamp_planar3_regression PRIVATE
            ${COMOTION_VAMP_SOURCE_DIR}/src/impl)
        add_test(NAME vamp_planar3_regression COMMAND vamp_planar3_regression)
        set_tests_properties(vamp_planar3_regression PROPERTIES
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

        add_executable(vamp_goal_hold_simd_regression
            tests/vamp_goal_hold_simd_regression.cpp)
        target_link_libraries(vamp_goal_hold_simd_regression PRIVATE comotion)
        add_test(NAME vamp_goal_hold_simd_regression
                 COMMAND vamp_goal_hold_simd_regression)

        add_executable(vamp_validation_variants_regression
            tests/vamp_validation_variants_regression.cpp)
        target_link_libraries(vamp_validation_variants_regression PRIVATE comotion)
        target_include_directories(vamp_validation_variants_regression PRIVATE
            ${COMOTION_VAMP_SOURCE_DIR}/src/impl)
        add_test(NAME vamp_validation_variants_regression
                 COMMAND vamp_validation_variants_regression)

        add_executable(vamp_conflict_scan_regression
            tests/vamp_conflict_scan_regression.cpp)
        target_link_libraries(vamp_conflict_scan_regression PRIVATE comotion)
        target_include_directories(vamp_conflict_scan_regression PRIVATE
            ${COMOTION_VAMP_SOURCE_DIR}/src/impl)
        add_test(NAME vamp_conflict_scan_regression
                 COMMAND vamp_conflict_scan_regression)
    endif()
endif()

install(FILES
    README.md
    API.md
    BENCHMARKS.md
    EXAMPLES.md
    RELEASE_CHECKLIST.md
    THIRD_PARTY_NOTICES.md
    LICENSE
    CITATION.cff
  DESTINATION "${CMAKE_INSTALL_DOCDIR}")

install(DIRECTORY LICENSES/
  DESTINATION "${CMAKE_INSTALL_DOCDIR}/licenses")

install(PROGRAMS
    benchmarks/scripts/benchmark_runner_common.py
    benchmarks/scripts/run_feasibility.py
    benchmarks/scripts/run_anytime.py
    benchmarks/scripts/run_multicore.py
    benchmarks/scripts/run_parallel_arc_2d.py
    benchmarks/scripts/run_parallel_arc_panda.py
    benchmarks/scripts/run_parallel_arc_conflict_ablation.py
    benchmarks/scripts/run_parallel_arc_optimistic_ablation.py
    benchmarks/scripts/plot_results.py
    benchmarks/scripts/run_planner_trials.py
    benchmarks/scripts/run_param_sweep.py
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/benchmarks/scripts")

install(FILES
    benchmarks/configs/planner_trial_params.json
    benchmarks/configs/heterogenous_param_sweep.json
    benchmarks/configs/scenario_planner_param_sweep_8seed.json
    benchmarks/configs/scenario_planner_params_8seed.json
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/benchmarks/configs")

install(FILES resources/README.md
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/resources")

install(FILES
    resources/panda/panda.urdf
    resources/panda/panda_spherized.urdf
    resources/panda/panda.srdf
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/resources/panda")

install(DIRECTORY resources/panda/meshes
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/resources/panda")

install(DIRECTORY resources/benchmarks
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/resources")

install(FILES
    resources/planar3/planar3.urdf
    resources/planar3/planar3_spherized.urdf
    resources/planar3/planar3.srdf
  DESTINATION "${CMAKE_INSTALL_DATADIR}/comotion/resources/planar3")

set(CPACK_PACKAGE_NAME "CoMotion")
set(CPACK_PACKAGE_VENDOR "Parasol Lab at the University of Illinois Urbana-Champaign")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_IGNORE_FILES
  "/\\\\.git/"
  "/\\\\.git$"
  "/\\\\.agents/"
  "/\\\\.codex/"
  "/build[^/]*/"
  "/_CPack_Packages/"
  "/CoMotion-.*-Source\\\\.tar\\\\.gz$"
  "/src/comotion/benchmark/"
  "/results/"
  "/benchmarks/results/"
  "/papers/"
  "/video/"
  "/viewer/data/"
  "/[^/]*\\\\.csv$"
  "/benchmarks/configs/panda_cage_n8_arc_(custom|expansion|exponential|initial|linear|log|tuning).*\\\\.json$"
  "/benchmarks/scripts/.*_07_.*\\\\.py$"
  "/benchmarks/scripts/run_.*_campaign_.*\\\\.py$"
  "/benchmarks/scripts/(analyze_parallel_arc_repair_or_trace|package_new_parc_results|recover_parc_final_8_20|run_2d_va_profile_parallel_arc|run_arc_panda_n8_parallel_defaults_15jobs|run_parallel_arc_refresh_30seed|run_parallel_arc_reproduction|verify_final_results_8_19_2026|verify_parallel_arc_refresh_30seed)\\\\.py$"
  "/benchmarks/scripts/analyze_arc_conflict_timings\\\\.py$"
  "/benchmarks/scripts/export_validation_chunk_csv\\\\.py$"
  "/benchmarks/scripts/plot_arc_runtime_.*\\\\.py$"
  "/external/como-ompl/scripts/benchmark\\\\.db$"
  "/external/como-ompl/scripts/docker/"
  "/external/como-ompl/scripts/ompl_benchmark_statistics\\\\.py$"
  "/external/como-ompl/external/vamp/scripts/"
  "/external/como-ompl/external/vamp/resources/"
  "/comotion-results/"
  "__pycache__/"
  "\\\\.pyc$"
  "\\\\.zip$"
  "\\\\.diff$"
  "\\\\.patch$"
  "~$"
)
include(CPack)
