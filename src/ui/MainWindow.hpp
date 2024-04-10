#pragma once

#include <QMainWindow>

class QToolButton;

/**
 * @brief Main window displaying the main user interface
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private slots:
    void
    setStartWidget();

    void
    setBagToVideoWidget();

    void
    setProgressWidget(const QString bagDirectory,
                      const QString topicName,
                      const QString vidDirectory,
                      bool          useHardwareAcceleration);

private:
    void
    closeEvent(QCloseEvent *event) override;
};
