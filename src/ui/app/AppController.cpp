#include "AppController.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>

namespace openpanelcam::ui {

AppController::AppController(QObject* parent)
    : QObject(parent)
{
}

AppController::~AppController() = default;

void AppController::importSTEP(const QString& path)
{
    QString filePath = path;
    if (filePath.isEmpty()) {
        filePath = QFileDialog::getOpenFileName(
            qobject_cast<QWidget*>(parent()),
            tr("Import STEP File"),
            QString(),
            tr("STEP Files (*.step *.stp);;All Files (*)")
        );
    }

    if (filePath.isEmpty()) return;

    m_currentFile = filePath;
    emit statusMessage(tr("Importing: %1").arg(filePath));

    runPhase1(filePath);
}

void AppController::runPhase1(const QString& stepPath)
{
    // TODO: Call Phase1::parseSTEPFile() in worker thread
    // For now, emit placeholder data to verify UI layout

    emit statusMessage(tr("Phase 1 complete (placeholder)"));

    // Extract filename for display
    QString filename = stepPath.section('/', -1).section('\\', -1);
    emit partInfoChanged(tr("Part: %1 | Bends: 0 | Faces: 0").arg(filename));

    // Placeholder: empty shapes and bends
    std::vector<ShapeDisplayData> shapes;
    emit shapesReady(shapes);

    m_bends.clear();
    emit bendsReady(m_bends);
}

void AppController::saveProject()
{
    emit statusMessage(tr("Save project - not yet implemented"));
}

void AppController::exportPBXML()
{
    emit statusMessage(tr("Export PB-XML - not yet implemented"));
}

void AppController::autoSequence()
{
    emit statusMessage(tr("Auto sequence - not yet implemented"));
}

void AppController::toggleSimulation()
{
    emit statusMessage(tr("Simulation toggle - not yet implemented"));
}

void AppController::stopSimulation()
{
    emit statusMessage(tr("Simulation stopped"));
}

void AppController::selectBend(int bendId)
{
    m_selectedBendId = bendId;
    emit highlightBend(bendId);

    for (const auto& bend : m_bends) {
        if (bend.id == bendId) {
            emit bendSelectionChanged(bend);
            return;
        }
    }
}

} // namespace openpanelcam::ui
