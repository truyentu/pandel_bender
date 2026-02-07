#==============================================================================
# OpenPanelCAM Phase 4: Physics Validator
#==============================================================================

add_library(openpanelcam_phase4 STATIC
    collision_primitives.cpp
    swept_volume.cpp
    bent_state.cpp
    collision_detector.cpp
    grasp_physics.cpp
    extraction_checker.cpp
    sequence_validator.cpp
)

target_include_directories(openpanelcam_phase4 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase4 PUBLIC
    openpanelcam_phase3
)

set_target_properties(openpanelcam_phase4 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
