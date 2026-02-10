#include "PartTreePanel.h"
#include "../app/AppController.h"

#include <QVBoxLayout>
#include <QHeaderView>

namespace openpanelcam::ui {

PartTreePanel::PartTreePanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({tr("Part Structure")});
    m_tree->header()->setVisible(false);
    m_tree->setRootIsDecorated(true);
    m_tree->setAnimated(true);
    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::currentItemChanged,
            this, [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
        if (current) {
            int bendId = current->data(0, Qt::UserRole).toInt();
            if (bendId >= 0) {
                emit bendSelected(bendId);
            }
        }
    });
}

void PartTreePanel::populateBends(const std::vector<BendDisplayInfo>& bends)
{
    m_tree->clear();

    auto* partItem = new QTreeWidgetItem(m_tree, {tr("Part")});
    auto* bendsItem = new QTreeWidgetItem(partItem, {tr("Bends (%1)").arg(bends.size())});

    for (const auto& bend : bends) {
        QString label = tr("B%1: %2 %3")
            .arg(bend.id)
            .arg(bend.angle, 0, 'f', 1)
            .arg(bend.direction);

        auto* item = new QTreeWidgetItem(bendsItem, {label});
        item->setData(0, Qt::UserRole, bend.id);

        // Color icon based on direction
        if (bend.direction == "UP") {
            item->setForeground(0, QColor(0x42, 0xa5, 0xf5)); // blue
        } else if (bend.direction == "DOWN") {
            item->setForeground(0, QColor(0xef, 0x53, 0x50)); // red
        } else if (bend.direction == "HEM") {
            item->setForeground(0, QColor(0xff, 0xca, 0x28)); // yellow
        }
    }

    m_tree->expandAll();
}

} // namespace openpanelcam::ui
