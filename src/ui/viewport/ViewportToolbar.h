#pragma once

#include <QToolBar>

namespace openpanelcam::ui {

class OcctViewport;

class ViewportToolbar : public QToolBar
{
    Q_OBJECT

public:
    explicit ViewportToolbar(OcctViewport* viewport, QWidget* parent = nullptr);

private:
    OcctViewport* m_viewport;
};

} // namespace openpanelcam::ui
