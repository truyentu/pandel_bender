#include "SequenceTimeline.h"

#include <QPainter>
#include <QPaintEvent>

namespace openpanelcam::ui {

SequenceTimeline::SequenceTimeline(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(80);
}

void SequenceTimeline::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Background
    painter.fillRect(rect(), QColor(0x2d, 0x2d, 0x3f));

    // Placeholder text
    painter.setPen(QColor(0xa0, 0xa0, 0xb0));
    painter.setFont(QFont("Segoe UI", 11));
    painter.drawText(rect(), Qt::AlignCenter,
        tr("Bend Sequence Timeline\n(Import a STEP file and run Auto Sequence)"));
}

} // namespace openpanelcam::ui
