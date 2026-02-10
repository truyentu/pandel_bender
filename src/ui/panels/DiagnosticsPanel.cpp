#include "DiagnosticsPanel.h"

#include <QVBoxLayout>

namespace openpanelcam::ui {

DiagnosticsPanel::DiagnosticsPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_list = new QListWidget(this);
    layout->addWidget(m_list);

    connect(m_list, &QListWidget::itemDoubleClicked,
            this, [this](QListWidgetItem* item) {
        int step = item->data(Qt::UserRole).toInt();
        if (step >= 0) {
            emit navigateToStep(step);
        }
    });

    // Show placeholder
    addMessage(tr("No validation results yet"), 0);
}

void DiagnosticsPanel::addMessage(const QString& message, int severity)
{
    auto* item = new QListWidgetItem(m_list);

    QString prefix;
    QColor color;
    switch (severity) {
        case 0:  prefix = QString::fromUtf8("\xe2\x9c\x93 "); color = QColor(0x66, 0xbb, 0x6a); break; // green check
        case 1:  prefix = QString::fromUtf8("\xe2\x9a\xa0 "); color = QColor(0xff, 0xa7, 0x26); break; // warning
        default: prefix = QString::fromUtf8("\xe2\x9c\x97 "); color = QColor(0xef, 0x53, 0x50); break; // red X
    }

    item->setText(prefix + message);
    item->setForeground(color);
    item->setData(Qt::UserRole, -1);
}

void DiagnosticsPanel::clear()
{
    m_list->clear();
}

} // namespace openpanelcam::ui
