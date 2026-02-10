#pragma once

#include <QWidget>

class QPushButton;
class QComboBox;
class QSlider;
class QLabel;

namespace openpanelcam::ui {

class SimulationControls : public QWidget
{
    Q_OBJECT

public:
    explicit SimulationControls(QWidget* parent = nullptr);

signals:
    void playPauseClicked();
    void stopClicked();
    void stepForward();
    void stepBackward();
    void speedChanged(double factor);

private:
    QPushButton* m_playBtn;
    QPushButton* m_stopBtn;
    QPushButton* m_stepBackBtn;
    QPushButton* m_stepFwdBtn;
    QComboBox* m_speedCombo;
    QSlider* m_progressSlider;
    QLabel* m_stepLabel;
};

} // namespace openpanelcam::ui
