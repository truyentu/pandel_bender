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

# PrecedenceDAG tests
add_executable(test_precedence_dag
    test_precedence_dag.cpp
)

target_link_libraries(test_precedence_dag PRIVATE
    openpanelcam_phase2
    Catch2::Catch2WithMain
)

target_include_directories(test_precedence_dag PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_precedence_dag)

# GeometricPrecedenceAnalyzer tests
add_executable(test_geometric_precedence_analyzer
    test_geometric_precedence_analyzer.cpp
)

target_link_libraries(test_geometric_precedence_analyzer PRIVATE
    openpanelcam_phase2
    Catch2::Catch2WithMain
)

target_include_directories(test_geometric_precedence_analyzer PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_geometric_precedence_analyzer)
