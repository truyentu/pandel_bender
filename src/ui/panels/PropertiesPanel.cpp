#include "PropertiesPanel.h"
#include "../app/AppController.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QGroupBox>

namespace openpanelcam::ui {

PropertiesPanel::PropertiesPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* mainLayout = new QVBoxLayout(this);

    m_titleLabel = new QLabel(tr("No bend selected"), this);
    m_titleLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
    mainLayout->addWidget(m_titleLabel);

    // Bend properties group
    auto* bendGroup = new QGroupBox(tr("Bend Properties"), this);
    auto* bendLayout = new QFormLayout(bendGroup);

    m_angleValue = new QLabel("-");
    m_radiusValue = new QLabel("-");
    m_directionValue = new QLabel("-");
    m_lengthValue = new QLabel("-");
    m_forceValue = new QLabel("-");
    m_springbackValue = new QLabel("-");

    bendLayout->addRow(tr("Angle:"), m_angleValue);
    bendLayout->addRow(tr("Radius:"), m_radiusValue);
    bendLayout->addRow(tr("Direction:"), m_directionValue);
    bendLayout->addRow(tr("Length:"), m_lengthValue);
    bendLayout->addRow(tr("Force:"), m_forceValue);
    bendLayout->addRow(tr("Springback:"), m_springbackValue);
    mainLayout->addWidget(bendGroup);

    // Material group
    auto* matGroup = new QGroupBox(tr("Material"), this);
    auto* matLayout = new QFormLayout(matGroup);

    m_materialValue = new QLabel("-");
    m_thicknessValue = new QLabel("-");

    matLayout->addRow(tr("Type:"), m_materialValue);
    matLayout->addRow(tr("Thickness:"), m_thicknessValue);
    mainLayout->addWidget(matGroup);

    mainLayout->addStretch();
}

void PropertiesPanel::showBendInfo(const BendDisplayInfo& info)
{
    m_titleLabel->setText(tr("Bend #%1").arg(info.id));
    m_angleValue->setText(tr("%1\u00B0").arg(info.angle, 0, 'f', 1));
    m_radiusValue->setText(tr("%1 mm").arg(info.radius, 0, 'f', 2));
    m_directionValue->setText(info.direction);
    m_lengthValue->setText(tr("%1 mm").arg(info.length, 0, 'f', 1));
    m_forceValue->setText(tr("%1 kN").arg(info.force, 0, 'f', 2));
    m_springbackValue->setText(tr("+%1\u00B0").arg(info.springback, 0, 'f', 1));
    m_materialValue->setText(info.material);
    m_thicknessValue->setText(tr("%1 mm").arg(info.thickness, 0, 'f', 1));

    // Color the direction label
    if (info.direction == "UP") {
        m_directionValue->setStyleSheet("color: #42a5f5;");
    } else if (info.direction == "DOWN") {
        m_directionValue->setStyleSheet("color: #ef5350;");
    } else if (info.direction == "HEM") {
        m_directionValue->setStyleSheet("color: #ffca28;");
    }
}

void PropertiesPanel::clear()
{
    m_titleLabel->setText(tr("No bend selected"));
    m_angleValue->setText("-");
    m_radiusValue->setText("-");
    m_directionValue->setText("-");
    m_directionValue->setStyleSheet("");
    m_lengthValue->setText("-");
    m_forceValue->setText("-");
    m_springbackValue->setText("-");
    m_materialValue->setText("-");
    m_thicknessValue->setText("-");
}

} // namespace openpanelcam::ui
