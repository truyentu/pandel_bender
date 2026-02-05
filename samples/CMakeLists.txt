#==============================================================================
# Sample Applications
#==============================================================================

# Sample 0: Test core utilities
add_executable(sample_test_core
    test_core.cpp
)

target_link_libraries(sample_test_core PRIVATE
    openpanelcam_core
)

# Sample 1: Parse STEP file (full pipeline)
add_executable(sample_parse_step
    parse_step.cpp
)

target_link_libraries(sample_parse_step PRIVATE
    openpanelcam_phase1
)

# Sample 2: Demo individual Phase 1 modules
add_executable(sample_phase1_modules
    demo_phase1_modules.cpp
)

target_link_libraries(sample_phase1_modules PRIVATE
    openpanelcam_phase1
)

# Sample 3: Visualize FAG
add_executable(sample_visualize_fag
    visualize_fag.cpp
)

target_link_libraries(sample_visualize_fag PRIVATE
    openpanelcam_phase1
    nlohmann_json::nlohmann_json
)

# Install samples
install(TARGETS
    sample_test_core
    sample_parse_step
    sample_phase1_modules
    sample_visualize_fag
    RUNTIME DESTINATION bin/samples
)
