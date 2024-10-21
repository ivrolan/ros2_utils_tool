#pragma once

#include <QDialog>
#include <QPointer>

class QCheckBox;

// Dialog used to modify settings
class SettingsDialog : public QDialog {
    Q_OBJECT

public:
    SettingsDialog(QWidget* parent = 0);

private:
    void
    storeParametersCheckStateChanged();

    void
    readSettings();

    void
    writeSettings();

    void
    okClicked();

private:
    QPointer<QCheckBox> m_storeParametersCheckBox;
};
