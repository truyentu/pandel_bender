#pragma once

#include <QWidget>

class QFormLayout;
class QLabel;

namespace openpanelcam::ui {

struct BendDisplayInfo;

class PropertiesPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PropertiesPanel(QWidget* parent = nullptr);

public slots:
    void showBendInfo(const BendDisplayInfo& info);
    void clear();

private:
    QLabel* m_titleLabel;
    QLabel* m_angleValue;
    QLabel* m_radiusValue;
    QLabel* m_directionValue;
    QLabel* m_lengthValue;
    QLabel* m_forceValue;
    QLabel* m_springbackValue;
    QLabel* m_materialValue;
    QLabel* m_thicknessValue;
};

} // namespace openpanelcam::ui
