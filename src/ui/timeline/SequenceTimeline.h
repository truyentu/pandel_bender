#pragma once

#include <QWidget>

namespace openpanelcam::ui {

class SequenceTimeline : public QWidget
{
    Q_OBJECT

public:
    explicit SequenceTimeline(QWidget* parent = nullptr);

    void setMinimumHeight(int h) { QWidget::setMinimumHeight(h); }

protected:
    void paintEvent(QPaintEvent* event) override;

signals:
    void stepSelected(int stepIndex);

private:
    // Placeholder - full implementation in UI-Phase 3
};

} // namespace openpanelcam::ui
