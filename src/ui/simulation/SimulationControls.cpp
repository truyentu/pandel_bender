#include "SimulationControls.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QLabel>

namespace openpanelcam::ui {

SimulationControls::SimulationControls(QWidget* parent)
    : QWidget(parent)
{
    auto* mainLayout = new QVBoxLayout(this);

    // Transport controls
    auto* transportLayout = new QHBoxLayout();

    m_stepBackBtn = new QPushButton(tr("<|"), this);
    m_stepBackBtn->setFixedWidth(36);
    m_stepBackBtn->setToolTip(tr("Step Back (Left Arrow)"));
    transportLayout->addWidget(m_stepBackBtn);

    m_playBtn = new QPushButton(tr("Play"), this);
    m_playBtn->setFixedWidth(60);
    m_playBtn->setToolTip(tr("Play/Pause (Space)"));
    transportLayout->addWidget(m_playBtn);

    m_stopBtn = new QPushButton(tr("Stop"), this);
    m_stopBtn->setFixedWidth(48);
    transportLayout->addWidget(m_stopBtn);

    m_stepFwdBtn = new QPushButton(tr("|>"), this);
    m_stepFwdBtn->setFixedWidth(36);
    m_stepFwdBtn->setToolTip(tr("Step Forward (Right Arrow)"));
    transportLayout->addWidget(m_stepFwdBtn);

    transportLayout->addSpacing(16);

    // Speed
    transportLayout->addWidget(new QLabel(tr("Speed:"), this));
    m_speedCombo = new QComboBox(this);
    m_speedCombo->addItems({"0.25x", "0.5x", "1x", "2x", "4x"});
    m_speedCombo->setCurrentIndex(2); // 1x default
    m_speedCombo->setFixedWidth(70);
    transportLayout->addWidget(m_speedCombo);

    transportLayout->addStretch();

    // Step counter
    m_stepLabel = new QLabel(tr("Step: -/-"), this);
    transportLayout->addWidget(m_stepLabel);

    mainLayout->addLayout(transportLayout);

    // Progress slider
    m_progressSlider = new QSlider(Qt::Horizontal, this);
    m_progressSlider->setRange(0, 100);
    m_progressSlider->setValue(0);
    mainLayout->addWidget(m_progressSlider);

    // Connections
    connect(m_playBtn, &QPushButton::clicked, this, &SimulationControls::playPauseClicked);
    connect(m_stopBtn, &QPushButton::clicked, this, &SimulationControls::stopClicked);
    connect(m_stepBackBtn, &QPushButton::clicked, this, &SimulationControls::stepBackward);
    connect(m_stepFwdBtn, &QPushButton::clicked, this, &SimulationControls::stepForward);

    connect(m_speedCombo, &QComboBox::currentTextChanged,
            this, [this](const QString& text) {
        QString num = text;
        num.remove('x');
        emit speedChanged(num.toDouble());
    });
}

} // namespace openpanelcam::ui
