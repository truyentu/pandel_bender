#==============================================================================
# OpenPanelCAM UI Module
#==============================================================================

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)

set(UI_SOURCES
    main.cpp
    app/MainWindow.h
    app/MainWindow.cpp
    app/AppController.h
    app/AppController.cpp
    viewport/OcctViewport.h
    viewport/OcctViewport.cpp
    viewport/ViewportToolbar.h
    viewport/ViewportToolbar.cpp
    panels/PartTreePanel.h
    panels/PartTreePanel.cpp
    panels/PropertiesPanel.h
    panels/PropertiesPanel.cpp
    panels/DiagnosticsPanel.h
    panels/DiagnosticsPanel.cpp
    timeline/SequenceTimeline.h
    timeline/SequenceTimeline.cpp
    simulation/SimulationControls.h
    simulation/SimulationControls.cpp
)

add_executable(OpenPanelCAM_UI WIN32 ${UI_SOURCES})

target_include_directories(OpenPanelCAM_UI PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(OpenPanelCAM_UI PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    openpanelcam_phase1
    openpanelcam_core
    ${OpenCASCADE_LIBRARIES}
)

target_compile_features(OpenPanelCAM_UI PRIVATE cxx_std_17)

# Copy Qt DLLs to output directory
if(WIN32)
    add_custom_command(TARGET OpenPanelCAM_UI POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:OpenPanelCAM_UI>
            $<TARGET_FILE_DIR:OpenPanelCAM_UI>
        COMMAND_EXPAND_LISTS
    )
endif()
