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

# Test for animation builder
add_executable(test_animation_builder test_animation_builder.cpp)

target_link_libraries(test_animation_builder PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_animation_builder PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_animation_builder)

# Test for XML generator
add_executable(test_xml_generator test_xml_generator.cpp)

target_link_libraries(test_xml_generator PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_xml_generator PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_xml_generator)

# Test for JSON generator
add_executable(test_json_generator test_json_generator.cpp)

target_link_libraries(test_json_generator PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_json_generator PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_json_generator)

# Test for output validator
add_executable(test_output_validator test_output_validator.cpp)

target_link_libraries(test_output_validator PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_output_validator PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_output_validator)

# Test for post processor pipeline
add_executable(test_post_processor test_post_processor.cpp)

target_link_libraries(test_post_processor PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_post_processor PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_post_processor)

# Test for edge cases
add_executable(test_phase5_edge_cases test_phase5_edge_cases.cpp)

target_link_libraries(test_phase5_edge_cases PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_phase5_edge_cases PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase5_edge_cases)

# Test for performance
add_executable(test_phase5_performance test_phase5_performance.cpp)

target_link_libraries(test_phase5_performance PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_phase5_performance PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase5_performance)

# Integration tests (Phase 2 -> 3 -> 4 -> 5)
add_executable(test_phase5_integration test_integration.cpp)

target_link_libraries(test_phase5_integration PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase5
)

target_include_directories(test_phase5_integration PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase5_integration)
