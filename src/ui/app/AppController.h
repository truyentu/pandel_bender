#pragma once

#include <QObject>
#include <QString>
#include <vector>

namespace openpanelcam::ui {

class MainWindow;

struct BendDisplayInfo {
    int id = -1;
    double angle = 0.0;
    double radius = 0.0;
    double length = 0.0;
    double force = 0.0;
    double springback = 0.0;
    QString direction;  // "UP", "DOWN", "HEM"
    QString material;
    double thickness = 0.0;
};

struct ShapeDisplayData {
    int id = -1;
    int colorR = 120, colorG = 144, colorB = 156; // default gray-blue
    // TopoDS_Shape will be stored in the controller, not passed through signals
};

class AppController : public QObject
{
    Q_OBJECT

public:
    explicit AppController(QObject* parent = nullptr);
    ~AppController() override;

    // Actions
    void importSTEP(const QString& path = QString());
    void saveProject();
    void exportPBXML();
    void autoSequence();
    void toggleSimulation();
    void stopSimulation();

    // Selection
    void selectBend(int bendId);

signals:
    void statusMessage(const QString& msg);
    void partInfoChanged(const QString& info);
    void shapesReady(const std::vector<ShapeDisplayData>& shapes);
    void bendsReady(const std::vector<BendDisplayInfo>& bends);
    void bendSelectionChanged(const BendDisplayInfo& info);
    void highlightBend(int bendId);

private:
    void runPhase1(const QString& stepPath);

    QString m_currentFile;
    std::vector<BendDisplayInfo> m_bends;
    int m_selectedBendId = -1;
};

} // namespace openpanelcam::ui
