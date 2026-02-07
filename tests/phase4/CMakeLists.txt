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
