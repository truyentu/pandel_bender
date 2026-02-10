#==============================================================================
# Phase 4 Unit Tests
#==============================================================================

# Test for types (AABB, OBB, ValidationResult)
add_executable(test_phase4_types test_types.cpp)

target_link_libraries(test_phase4_types PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_phase4_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase4_types)

# Test for swept volume, bent state, and collision detector
add_executable(test_collision test_collision.cpp)

target_link_libraries(test_collision PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_collision PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_collision)

# Test for grasp physics engine
add_executable(test_grasp_physics test_grasp_physics.cpp)

target_link_libraries(test_grasp_physics PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_grasp_physics PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_grasp_physics)

# Test for extraction checker and springback compensator
add_executable(test_extraction test_extraction.cpp)

target_link_libraries(test_extraction PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_extraction PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_extraction)

# Test for sequence validator and feedback generator
add_executable(test_sequence_validator test_sequence_validator.cpp)

target_link_libraries(test_sequence_validator PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_sequence_validator PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_sequence_validator)

# Edge case tests
add_executable(test_phase4_edge_cases test_phase4_edge_cases.cpp)

target_link_libraries(test_phase4_edge_cases PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_phase4_edge_cases PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase4_edge_cases)

# Performance benchmarks
add_executable(test_phase4_performance test_phase4_performance.cpp)

target_link_libraries(test_phase4_performance PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_phase4_performance PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase4_performance)

# Integration tests (Phase 2 -> 3 -> 4)
add_executable(test_phase4_integration test_integration.cpp)

target_link_libraries(test_phase4_integration PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_phase4_integration PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase4_integration)

# Test for AABB tree hierarchy
add_executable(test_aabb_tree test_aabb_tree.cpp)

target_link_libraries(test_aabb_tree PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_aabb_tree PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_aabb_tree)

# Test for negative sequence analysis
add_executable(test_negative_sequence test_negative_sequence.cpp)

target_link_libraries(test_negative_sequence PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_negative_sequence PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_negative_sequence)

# Test for angular interval swept volume
add_executable(test_angular_intervals test_angular_intervals.cpp)

target_link_libraries(test_angular_intervals PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase4
)

target_include_directories(test_angular_intervals PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_angular_intervals)
