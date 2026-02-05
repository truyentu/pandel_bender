#==============================================================================
# Phase 2 Library
#==============================================================================

add_library(openpanelcam_phase2
    precedence_dag.cpp
)

target_include_directories(openpanelcam_phase2 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase2 PUBLIC
    Boost::graph
)

set_target_properties(openpanelcam_phase2 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
