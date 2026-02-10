#==============================================================================
# Unfold Module Tests
#==============================================================================

add_executable(test_unfold
    test_k_factor_table.cpp
    test_bend_allowance.cpp
    test_unfold_tree.cpp
    test_unfolder.cpp
)

target_link_libraries(test_unfold PRIVATE
    openpanelcam_unfold
    Catch2::Catch2WithMain
)

catch_discover_tests(test_unfold)
