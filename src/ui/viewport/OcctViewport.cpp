#include "OcctViewport.h"
#include "../app/AppController.h"

#include <QMouseEvent>
#include <QWheelEvent>

#ifdef _WIN32
#include <WNT_Window.hxx>
#else
#include <Xw_Window.hxx>
#endif

#include <Graphic3d_GraphicDriver.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Quantity_Color.hxx>
#include <V3d_AmbientLight.hxx>
#include <V3d_DirectionalLight.hxx>

namespace openpanelcam::ui {

OcctViewport::OcctViewport(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setMouseTracking(true);
    setMinimumSize(400, 300);

    // Deferred initialization - OCCT needs a valid window handle
    // initOcct() will be called on first paint
}

OcctViewport::~OcctViewport() = default;

void OcctViewport::initOcct()
{
    if (!m_viewer.IsNull()) return; // Already initialized

    // Create display connection
    m_displayConnection = new Aspect_DisplayConnection();

    // Create graphic driver
    m_graphicDriver = new OpenGl_GraphicDriver(m_displayConnection);

    // Create viewer
    m_viewer = new V3d_Viewer(m_graphicDriver);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();

    // Background gradient (dark theme)
    m_viewer->SetDefaultBgGradientColors(
        Quantity_Color(0.12, 0.12, 0.18, Quantity_TOC_RGB),  // top: dark
        Quantity_Color(0.08, 0.08, 0.12, Quantity_TOC_RGB),  // bottom: darker
        Aspect_GradientFillMethod_Vertical
    );

    // Create interactive context
    m_context = new AIS_InteractiveContext(m_viewer);
    m_context->SetDisplayMode(AIS_Shaded, Standard_True);

    // Create view
    m_view = m_viewer->CreateView();

    // Create platform-specific window
#ifdef _WIN32
    Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle)winId());
#else
    Handle(Xw_Window) wind = new Xw_Window(m_displayConnection, (Aspect_Handle)winId());
#endif

    m_view->SetWindow(wind);
    if (!wind->IsMapped()) {
        wind->Map();
    }

    // Setup view
    m_view->SetBackgroundColor(Quantity_Color(0.12, 0.12, 0.18, Quantity_TOC_RGB));
    m_view->MustBeResized();
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08);

    // Isometric view by default
    m_view->SetProj(V3d_XposYnegZpos);
    m_view->FitAll();
}

void OcctViewport::paintEvent(QPaintEvent*)
{
    initOcct();
    if (!m_view.IsNull()) {
        m_view->Redraw();
    }
}

void OcctViewport::resizeEvent(QResizeEvent*)
{
    if (!m_view.IsNull()) {
        m_view->MustBeResized();
    }
}

void OcctViewport::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->pos();

    if (event->button() == Qt::MiddleButton) {
        m_isRotating = true;
    } else if (event->button() == Qt::RightButton) {
        m_isPanning = true;
    } else if (event->button() == Qt::LeftButton) {
        // Selection
        if (!m_context.IsNull()) {
            m_context->MoveTo(event->pos().x(), event->pos().y(), m_view, Standard_True);
            m_context->Select(Standard_True);
        }
    }
}

void OcctViewport::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton) {
        m_isRotating = false;
    } else if (event->button() == Qt::RightButton) {
        m_isPanning = false;
    }
}

void OcctViewport::mouseMoveEvent(QMouseEvent* event)
{
    if (m_view.IsNull()) return;

    QPoint delta = event->pos() - m_lastMousePos;

    if (m_isRotating) {
        m_view->Rotation(event->pos().x(), event->pos().y());
    } else if (m_isPanning) {
        m_view->Pan(delta.x(), -delta.y());
    } else {
        // Hover detection
        if (!m_context.IsNull()) {
            m_context->MoveTo(event->pos().x(), event->pos().y(), m_view, Standard_True);
        }
    }

    if (m_isRotating && m_lastMousePos != event->pos()) {
        m_view->StartRotation(m_lastMousePos.x(), m_lastMousePos.y());
        m_view->Rotation(event->pos().x(), event->pos().y());
    }

    m_lastMousePos = event->pos();
}

void OcctViewport::wheelEvent(QWheelEvent* event)
{
    if (m_view.IsNull()) return;

    double delta = event->angleDelta().y();
    if (delta > 0) {
        m_view->SetScale(m_view->Scale() * 1.1);
    } else {
        m_view->SetScale(m_view->Scale() / 1.1);
    }
    m_view->Redraw();
}

void OcctViewport::fitAll()
{
    if (!m_view.IsNull()) {
        m_view->FitAll();
        m_view->Redraw();
    }
}

void OcctViewport::setViewPreset(int preset)
{
    if (m_view.IsNull()) return;

    switch (preset) {
        case 0: m_view->SetProj(V3d_XposYnegZpos); break; // Iso
        case 1: m_view->SetProj(V3d_Yneg); break;         // Front
        case 2: m_view->SetProj(V3d_Ypos); break;         // Back
        case 3: m_view->SetProj(V3d_Zpos); break;         // Top
        case 4: m_view->SetProj(V3d_Zneg); break;         // Bottom
        case 5: m_view->SetProj(V3d_Xneg); break;         // Left
        case 6: m_view->SetProj(V3d_Xpos); break;         // Right
    }
    m_view->FitAll();
    m_view->Redraw();
}

void OcctViewport::displayShapes(const std::vector<ShapeDisplayData>& shapes)
{
    if (m_context.IsNull()) return;

    // Clear existing
    m_context->RemoveAll(Standard_False);
    m_shapes.clear();

    // TODO: Display actual TopoDS_Shapes from Phase 1
    // For now this is a placeholder

    m_view->FitAll();
    m_view->Redraw();
}

void OcctViewport::highlightShape(int shapeId)
{
    if (m_context.IsNull()) return;

    // Unhighlight previous
    m_context->ClearSelected(Standard_False);

    // Highlight new
    if (shapeId >= 0 && shapeId < static_cast<int>(m_shapes.size())) {
        m_context->SetSelected(m_shapes[shapeId], Standard_True);
    }

    m_highlightedId = shapeId;
    m_view->Redraw();
}

} // namespace openpanelcam::ui
