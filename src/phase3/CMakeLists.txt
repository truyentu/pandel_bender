#==============================================================================
# OpenPanelCAM Phase 3: Sequencer
#==============================================================================

add_library(openpanelcam_phase3 STATIC
    zobrist_table.cpp
    cost_function.cpp
    heuristic.cpp
    successor_generator.cpp
    astar_search.cpp
)

target_include_directories(openpanelcam_phase3 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase3 PUBLIC
    openpanelcam_phase2
)

set_target_properties(openpanelcam_phase3 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
