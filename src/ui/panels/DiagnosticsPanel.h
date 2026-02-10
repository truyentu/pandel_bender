#pragma once

#include <QWidget>
#include <QListWidget>

namespace openpanelcam::ui {

class DiagnosticsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit DiagnosticsPanel(QWidget* parent = nullptr);

    void addMessage(const QString& message, int severity); // 0=OK, 1=Warning, 2=Error
    void clear();

signals:
    void navigateToStep(int stepIndex);

private:
    QListWidget* m_list;
};

} // namespace openpanelcam::ui
