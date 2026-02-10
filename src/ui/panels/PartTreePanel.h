#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <vector>

namespace openpanelcam::ui {

struct BendDisplayInfo;

class PartTreePanel : public QWidget
{
    Q_OBJECT

public:
    explicit PartTreePanel(QWidget* parent = nullptr);

public slots:
    void populateBends(const std::vector<BendDisplayInfo>& bends);

signals:
    void bendSelected(int bendId);

private:
    QTreeWidget* m_tree;
};

} // namespace openpanelcam::ui
