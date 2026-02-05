#==============================================================================
# OpenPanelCAM Phase 1: Geometric Parser
#==============================================================================

set(PHASE1_HEADERS
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/step_reader.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/geometry_healer.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/fag.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/fag_builder.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/base_face_identifier.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/bend_classifier.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/phase1/phase1.h
)

set(PHASE1_SOURCES
    step_reader.cpp
    geometry_healer.cpp
    fag.cpp
    fag_builder.cpp
    base_face_identifier.cpp
    bend_classifier.cpp
    phase1.cpp
)

add_library(openpanelcam_phase1 STATIC
    ${PHASE1_HEADERS}
    ${PHASE1_SOURCES}
)

target_include_directories(openpanelcam_phase1 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase1 PUBLIC
    openpanelcam_core
    ${OpenCASCADE_LIBRARIES}
    Eigen3::Eigen
    spdlog::spdlog
)

target_compile_features(openpanelcam_phase1 PUBLIC cxx_std_17)
