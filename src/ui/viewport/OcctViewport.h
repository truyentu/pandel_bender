#pragma once

#include <QWidget>

#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>

#include <vector>

namespace openpanelcam::ui {

struct ShapeDisplayData;

class OcctViewport : public QWidget
{
    Q_OBJECT

public:
    explicit OcctViewport(QWidget* parent = nullptr);
    ~OcctViewport() override;

    // View operations
    void fitAll();
    void setViewPreset(int preset); // 0=Iso, 1=Front, 2=Back, 3=Top, 4=Bottom, 5=Left, 6=Right

public slots:
    void displayShapes(const std::vector<ShapeDisplayData>& shapes);
    void highlightShape(int shapeId);

protected:
    QPaintEngine* paintEngine() const override { return nullptr; }
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void initOcct();

    // OCCT handles
    Handle(Aspect_DisplayConnection) m_displayConnection;
    Handle(OpenGl_GraphicDriver) m_graphicDriver;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(AIS_InteractiveContext) m_context;

    // Mouse tracking
    QPoint m_lastMousePos;
    bool m_isRotating = false;
    bool m_isPanning = false;

    // Displayed shapes
    std::vector<Handle(AIS_Shape)> m_shapes;
    int m_highlightedId = -1;
};

} // namespace openpanelcam::ui
