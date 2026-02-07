#==============================================================================
# Phase 3 Unit Tests
#==============================================================================

# Test for types (header-only, no library needed)
add_executable(test_phase3_types test_types.cpp)

target_link_libraries(test_phase3_types PRIVATE
    Catch2::Catch2WithMain
)

target_include_directories(test_phase3_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_phase3_types)

# Test for Zobrist hashing
add_executable(test_zobrist test_zobrist.cpp)

target_link_libraries(test_zobrist PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase3
)

target_include_directories(test_zobrist PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_zobrist)

# Test for cost function (MaskedTimeCost + BendTimeEstimator)
add_executable(test_cost_function test_cost_function.cpp)

target_link_libraries(test_cost_function PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase3
)

target_include_directories(test_cost_function PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

catch_discover_tests(test_cost_function)
