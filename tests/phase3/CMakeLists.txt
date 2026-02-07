#==============================================================================
# Phase 3 Unit Tests
#==============================================================================

# Test for types
add_executable(test_phase3_types test_types.cpp)

target_link_libraries(test_phase3_types PRIVATE
    Catch2::Catch2WithMain
)

target_include_directories(test_phase3_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase3_types)
