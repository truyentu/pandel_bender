/**
 * @file main.cpp
 * @brief OpenPanelCAM Desktop Application entry point
 */

#include "app/MainWindow.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

static void setupDarkTheme(QApplication& app)
{
    app.setStyle(QStyleFactory::create("Fusion"));

    QPalette palette;
    palette.setColor(QPalette::Window, QColor(0x1e, 0x1e, 0x2e));
    palette.setColor(QPalette::WindowText, QColor(0xe0, 0xe0, 0xe0));
    palette.setColor(QPalette::Base, QColor(0x2d, 0x2d, 0x3f));
    palette.setColor(QPalette::AlternateBase, QColor(0x36, 0x36, 0x50));
    palette.setColor(QPalette::ToolTipBase, QColor(0x2d, 0x2d, 0x3f));
    palette.setColor(QPalette::ToolTipText, QColor(0xe0, 0xe0, 0xe0));
    palette.setColor(QPalette::Text, QColor(0xe0, 0xe0, 0xe0));
    palette.setColor(QPalette::Button, QColor(0x2d, 0x2d, 0x3f));
    palette.setColor(QPalette::ButtonText, QColor(0xe0, 0xe0, 0xe0));
    palette.setColor(QPalette::BrightText, QColor(0xff, 0x17, 0x44));
    palette.setColor(QPalette::Link, QColor(0x4f, 0xc3, 0xf7));
    palette.setColor(QPalette::Highlight, QColor(0x4f, 0xc3, 0xf7));
    palette.setColor(QPalette::HighlightedText, QColor(0x1e, 0x1e, 0x2e));

    // Disabled colors
    palette.setColor(QPalette::Disabled, QPalette::Text, QColor(0x60, 0x60, 0x70));
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x60, 0x60, 0x70));
    palette.setColor(QPalette::Disabled, QPalette::WindowText, QColor(0x60, 0x60, 0x70));

    app.setPalette(palette);
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("OpenPanelCAM");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("OpenPanelCAM");

    setupDarkTheme(app);

    openpanelcam::ui::MainWindow mainWindow;
    mainWindow.setWindowTitle("OpenPanelCAM - Panel Bender CAM");
    mainWindow.resize(1600, 900);
    mainWindow.show();

    return app.exec();
}
