#==============================================================================
# OpenPanelCAM Core Library
#==============================================================================

set(CORE_HEADERS
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/core/types.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/core/constants.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/core/geometry_utils.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/core/logger.h
    ${CMAKE_SOURCE_DIR}/include/openpanelcam/core/error.h
)

set(CORE_SOURCES
    geometry_utils.cpp
    logger.cpp
    error.cpp
)

add_library(openpanelcam_core STATIC
    ${CORE_HEADERS}
    ${CORE_SOURCES}
)

target_include_directories(openpanelcam_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_core PUBLIC
    ${OpenCASCADE_LIBRARIES}
    Eigen3::Eigen
    spdlog::spdlog
    fmt::fmt
)

# Set compile features
target_compile_features(openpanelcam_core PUBLIC cxx_std_17)

# Export symbols
if(BUILD_SHARED_LIBS)
    target_compile_definitions(openpanelcam_core PRIVATE OPENPANELCAM_EXPORTS)
endif()
