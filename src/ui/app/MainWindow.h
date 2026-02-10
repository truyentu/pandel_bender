#pragma once

#include <QMainWindow>

class QDockWidget;
class QToolBar;
class QStatusBar;
class QLabel;

namespace openpanelcam::ui {

class OcctViewport;
class AppController;
class PartTreePanel;
class PropertiesPanel;
class DiagnosticsPanel;
class SequenceTimeline;
class SimulationControls;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    void setupMenuBar();
    void setupToolBar();
    void setupDockWidgets();
    void setupStatusBar();
    void setupConnections();
    void saveWindowState();
    void restoreWindowState();

    // Central widget
    OcctViewport* m_viewport = nullptr;

    // Dock panels
    PartTreePanel* m_partTree = nullptr;
    PropertiesPanel* m_properties = nullptr;
    DiagnosticsPanel* m_diagnostics = nullptr;
    SequenceTimeline* m_timeline = nullptr;
    SimulationControls* m_simControls = nullptr;

    // Dock widgets (for show/hide)
    QDockWidget* m_partTreeDock = nullptr;
    QDockWidget* m_propertiesDock = nullptr;
    QDockWidget* m_diagnosticsDock = nullptr;
    QDockWidget* m_timelineDock = nullptr;
    QDockWidget* m_simControlsDock = nullptr;

    // Toolbar
    QToolBar* m_mainToolBar = nullptr;

    // Status bar labels
    QLabel* m_statusLabel = nullptr;
    QLabel* m_partInfoLabel = nullptr;

    // Controller
    AppController* m_controller = nullptr;
};

} // namespace openpanelcam::ui
