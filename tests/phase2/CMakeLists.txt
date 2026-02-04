#==============================================================================
# Phase 2 Unit Tests
#==============================================================================

add_executable(test_types
    test_types.cpp
)

target_link_libraries(test_types PRIVATE
    Catch2::Catch2WithMain
)

target_include_directories(test_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_types)
