#==============================================================================
# OpenPanelCAM Phase 5: Post-Processor
#==============================================================================

add_library(openpanelcam_phase5 STATIC
    instruction_builder.cpp
)

target_include_directories(openpanelcam_phase5 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase5 PUBLIC
    openpanelcam_phase4
)

# Optional dependencies
if(TARGET pugixml::pugixml)
    target_link_libraries(openpanelcam_phase5 PUBLIC pugixml::pugixml)
    target_compile_definitions(openpanelcam_phase5 PUBLIC HAS_PUGIXML=1)
elseif(TARGET pugixml)
    target_link_libraries(openpanelcam_phase5 PUBLIC pugixml)
    target_compile_definitions(openpanelcam_phase5 PUBLIC HAS_PUGIXML=1)
endif()

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(openpanelcam_phase5 PUBLIC nlohmann_json::nlohmann_json)
    target_compile_definitions(openpanelcam_phase5 PUBLIC HAS_NLOHMANN_JSON=1)
endif()

set_target_properties(openpanelcam_phase5 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
