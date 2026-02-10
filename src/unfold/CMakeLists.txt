#==============================================================================
# OpenPanelCAM Unfold Module
#==============================================================================

set(UNFOLD_HEADERS
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/unfold/types.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/unfold/k_factor_table.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/unfold/bend_allowance.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/unfold/unfold_tree.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/unfold/unfolder.h
)

set(UNFOLD_SOURCES
    k_factor_table.cpp
    bend_allowance.cpp
    unfold_tree.cpp
    unfolder.cpp
)

add_library(openpanelcam_unfold STATIC
    ${UNFOLD_HEADERS}
    ${UNFOLD_SOURCES}
)

target_include_directories(openpanelcam_unfold PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_unfold PUBLIC
    openpanelcam_phase1
    openpanelcam_core
    ${OpenCASCADE_LIBRARIES}
    Eigen3::Eigen
    spdlog::spdlog
)

target_compile_features(openpanelcam_unfold PUBLIC cxx_std_17)
