#include "ViewportToolbar.h"
#include "OcctViewport.h"

namespace openpanelcam::ui {

ViewportToolbar::ViewportToolbar(OcctViewport* viewport, QWidget* parent)
    : QToolBar(tr("Viewport"), parent)
    , m_viewport(viewport)
{
    addAction(tr("Fit All (F)"), m_viewport, &OcctViewport::fitAll);
    addSeparator();
    addAction(tr("Iso (0)"), m_viewport, [this]() { m_viewport->setViewPreset(0); });
    addAction(tr("Front (1)"), m_viewport, [this]() { m_viewport->setViewPreset(1); });
    addAction(tr("Back (2)"), m_viewport, [this]() { m_viewport->setViewPreset(2); });
    addAction(tr("Top (3)"), m_viewport, [this]() { m_viewport->setViewPreset(3); });
    addAction(tr("Bottom (4)"), m_viewport, [this]() { m_viewport->setViewPreset(4); });
    addAction(tr("Left (5)"), m_viewport, [this]() { m_viewport->setViewPreset(5); });
    addAction(tr("Right (6)"), m_viewport, [this]() { m_viewport->setViewPreset(6); });
}

} // namespace openpanelcam::ui
