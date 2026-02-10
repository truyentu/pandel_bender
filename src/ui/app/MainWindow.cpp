#include "MainWindow.h"
#include "AppController.h"
#include "../viewport/OcctViewport.h"
#include "../viewport/ViewportToolbar.h"
#include "../panels/PartTreePanel.h"
#include "../panels/PropertiesPanel.h"
#include "../panels/DiagnosticsPanel.h"
#include "../timeline/SequenceTimeline.h"
#include "../simulation/SimulationControls.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QLabel>
#include <QCloseEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QSettings>
#include <QFileDialog>
#include <QMessageBox>

namespace openpanelcam::ui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setAcceptDrops(true);
    setDockNestingEnabled(true);

    // Create controller first
    m_controller = new AppController(this);

    // Create central viewport
    m_viewport = new OcctViewport(this);
    setCentralWidget(m_viewport);

    setupMenuBar();
    setupToolBar();
    setupDockWidgets();
    setupStatusBar();
    setupConnections();
    restoreWindowState();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupMenuBar()
{
    // File menu
    auto* fileMenu = menuBar()->addMenu(tr("&File"));

    auto* importAction = fileMenu->addAction(tr("&Import STEP..."), this, [this]() {
        m_controller->importSTEP();
    });
    importAction->setShortcut(QKeySequence("Ctrl+O"));

    fileMenu->addSeparator();

    auto* saveAction = fileMenu->addAction(tr("&Save Project"), this, [this]() {
        m_controller->saveProject();
    });
    saveAction->setShortcut(QKeySequence("Ctrl+S"));

    fileMenu->addSeparator();

    auto* exportAction = fileMenu->addAction(tr("Export PB-&XML..."), this, [this]() {
        m_controller->exportPBXML();
    });
    exportAction->setShortcut(QKeySequence("Ctrl+Shift+S"));

    fileMenu->addSeparator();
    fileMenu->addAction(tr("E&xit"), this, &QWidget::close, QKeySequence("Alt+F4"));

    // Edit menu
    auto* editMenu = menuBar()->addMenu(tr("&Edit"));
    auto* undoAction = editMenu->addAction(tr("&Undo"));
    undoAction->setShortcut(QKeySequence::Undo);
    auto* redoAction = editMenu->addAction(tr("&Redo"));
    redoAction->setShortcut(QKeySequence::Redo);

    // View menu
    auto* viewMenu = menuBar()->addMenu(tr("&View"));
    // Dock visibility toggles will be added after dock creation

    // Sequence menu
    auto* seqMenu = menuBar()->addMenu(tr("&Sequence"));
    auto* autoSeqAction = seqMenu->addAction(tr("&Auto Generate"), this, [this]() {
        m_controller->autoSequence();
    });
    autoSeqAction->setShortcut(QKeySequence("Ctrl+G"));

    // Simulate menu
    auto* simMenu = menuBar()->addMenu(tr("Si&mulate"));
    simMenu->addAction(tr("&Play/Pause"), this, [this]() {
        m_controller->toggleSimulation();
    }, QKeySequence("Space"));
    simMenu->addAction(tr("&Stop"), this, [this]() {
        m_controller->stopSimulation();
    });

    // Help menu
    auto* helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(tr("&About"), this, [this]() {
        QMessageBox::about(this, tr("About OpenPanelCAM"),
            tr("OpenPanelCAM v0.1.0\n\n"
               "Desktop CAM for Salvagnini P4 Panel Bender\n"
               "STEP -> Bend Sequence -> PB-XML"));
    });

    // Add dock toggles to View menu
    // (done after setupDockWidgets)
}

void MainWindow::setupToolBar()
{
    m_mainToolBar = addToolBar(tr("Main"));
    m_mainToolBar->setMovable(false);
    m_mainToolBar->setIconSize(QSize(24, 24));

    m_mainToolBar->addAction(tr("Import"), this, [this]() {
        m_controller->importSTEP();
    });
    m_mainToolBar->addAction(tr("Save"), this, [this]() {
        m_controller->saveProject();
    });
    m_mainToolBar->addSeparator();
    m_mainToolBar->addAction(tr("Auto Sequence"), this, [this]() {
        m_controller->autoSequence();
    });
    m_mainToolBar->addSeparator();
    m_mainToolBar->addAction(tr("Simulate"), this, [this]() {
        m_controller->toggleSimulation();
    });
    m_mainToolBar->addSeparator();
    m_mainToolBar->addAction(tr("Export"), this, [this]() {
        m_controller->exportPBXML();
    });
}

void MainWindow::setupDockWidgets()
{
    // Part Tree - Left
    m_partTree = new PartTreePanel(this);
    m_partTreeDock = new QDockWidget(tr("Part Tree"), this);
    m_partTreeDock->setWidget(m_partTree);
    m_partTreeDock->setObjectName("PartTreeDock");
    addDockWidget(Qt::LeftDockWidgetArea, m_partTreeDock);

    // Properties - Right
    m_properties = new PropertiesPanel(this);
    m_propertiesDock = new QDockWidget(tr("Properties"), this);
    m_propertiesDock->setWidget(m_properties);
    m_propertiesDock->setObjectName("PropertiesDock");
    addDockWidget(Qt::RightDockWidgetArea, m_propertiesDock);

    // Diagnostics - Bottom Left
    m_diagnostics = new DiagnosticsPanel(this);
    m_diagnosticsDock = new QDockWidget(tr("Diagnostics"), this);
    m_diagnosticsDock->setWidget(m_diagnostics);
    m_diagnosticsDock->setObjectName("DiagnosticsDock");
    addDockWidget(Qt::BottomDockWidgetArea, m_diagnosticsDock);

    // Timeline - Bottom Center
    m_timeline = new SequenceTimeline(this);
    m_timelineDock = new QDockWidget(tr("Bend Sequence"), this);
    m_timelineDock->setWidget(m_timeline);
    m_timelineDock->setObjectName("TimelineDock");
    addDockWidget(Qt::BottomDockWidgetArea, m_timelineDock);

    // Simulation Controls - Bottom Right
    m_simControls = new SimulationControls(this);
    m_simControlsDock = new QDockWidget(tr("Simulation"), this);
    m_simControlsDock->setWidget(m_simControls);
    m_simControlsDock->setObjectName("SimControlsDock");
    addDockWidget(Qt::BottomDockWidgetArea, m_simControlsDock);

    // Tab bottom docks together
    tabifyDockWidget(m_diagnosticsDock, m_timelineDock);
    m_timelineDock->raise(); // Timeline visible by default

    // Add toggle actions to View menu
    auto* viewMenu = menuBar()->findChild<QMenu*>(QString(), Qt::FindDirectChildrenOnly);
    // Find View menu (3rd menu)
    auto menus = menuBar()->findChildren<QMenu*>(QString(), Qt::FindDirectChildrenOnly);
    for (auto* menu : menus) {
        if (menu->title().contains("View")) {
            menu->addAction(m_partTreeDock->toggleViewAction());
            menu->addAction(m_propertiesDock->toggleViewAction());
            menu->addAction(m_diagnosticsDock->toggleViewAction());
            menu->addAction(m_timelineDock->toggleViewAction());
            menu->addAction(m_simControlsDock->toggleViewAction());
            break;
        }
    }
}

void MainWindow::setupStatusBar()
{
    m_statusLabel = new QLabel(tr("Ready"));
    statusBar()->addWidget(m_statusLabel, 1);

    m_partInfoLabel = new QLabel();
    statusBar()->addPermanentWidget(m_partInfoLabel);
}

void MainWindow::setupConnections()
{
    // Controller → Status bar
    connect(m_controller, &AppController::statusMessage,
            m_statusLabel, &QLabel::setText);

    connect(m_controller, &AppController::partInfoChanged,
            m_partInfoLabel, &QLabel::setText);

    // Controller → Viewport
    connect(m_controller, &AppController::shapesReady,
            m_viewport, &OcctViewport::displayShapes);

    // Controller → Part Tree
    connect(m_controller, &AppController::bendsReady,
            m_partTree, &PartTreePanel::populateBends);

    // Part Tree → Controller (selection sync)
    connect(m_partTree, &PartTreePanel::bendSelected,
            m_controller, &AppController::selectBend);

    // Controller → Properties
    connect(m_controller, &AppController::bendSelectionChanged,
            m_properties, &PropertiesPanel::showBendInfo);

    // Controller → Viewport highlight
    connect(m_controller, &AppController::highlightBend,
            m_viewport, &OcctViewport::highlightShape);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    saveWindowState();
    event->accept();
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
    if (event->mimeData()->hasUrls()) {
        for (const auto& url : event->mimeData()->urls()) {
            QString path = url.toLocalFile().toLower();
            if (path.endsWith(".step") || path.endsWith(".stp")) {
                event->acceptProposedAction();
                return;
            }
        }
    }
}

void MainWindow::dropEvent(QDropEvent* event)
{
    for (const auto& url : event->mimeData()->urls()) {
        QString path = url.toLocalFile();
        if (path.toLower().endsWith(".step") || path.toLower().endsWith(".stp")) {
            m_controller->importSTEP(path);
            return;
        }
    }
}

void MainWindow::saveWindowState()
{
    QSettings settings;
    settings.setValue("MainWindow/geometry", saveGeometry());
    settings.setValue("MainWindow/state", saveState());
}

void MainWindow::restoreWindowState()
{
    QSettings settings;
    restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
    restoreState(settings.value("MainWindow/state").toByteArray());
}

} // namespace openpanelcam::ui
