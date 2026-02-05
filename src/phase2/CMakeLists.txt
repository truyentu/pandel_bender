#==============================================================================
# Phase 2 Library
#==============================================================================

add_library(openpanelcam_phase2
    precedence_dag.cpp
    geometric_precedence_analyzer.cpp
    grasp_constraint_generator.cpp
    aba_constraint_analyzer.cpp
    constraint_solver.cpp
)

target_include_directories(openpanelcam_phase2 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase2 PUBLIC
    Boost::graph
)

# TODO: Re-enable when Phase 1 is fixed
# target_link_libraries(openpanelcam_phase2 PUBLIC
#     openpanelcam_phase1
# )

set_target_properties(openpanelcam_phase2 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
