#==============================================================================
# Phase 5 Unit Tests
#==============================================================================

# Test for types
add_executable(test_phase5_types test_types.cpp)

target_link_libraries(test_phase5_types PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_phase5_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase5_types)

# Test for instruction builder
add_executable(test_instruction_builder test_instruction_builder.cpp)

target_link_libraries(test_instruction_builder PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_instruction_builder PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_instruction_builder)
