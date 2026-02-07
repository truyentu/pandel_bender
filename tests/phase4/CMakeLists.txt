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
