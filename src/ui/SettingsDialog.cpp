#include "SettingsDialog.hpp"

#include "UtilsSettings.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QSettings>
#include <QVBoxLayout>

SettingsDialog::SettingsDialog(QWidget* parent) :
    QDialog(parent)
{
    setWindowTitle("Options");

    m_storeParametersCheckBox = new QCheckBox("Save Input Parameters");
    m_storeParametersCheckBox->setTristate(false);
    m_storeParametersCheckBox->setToolTip("If this is checked, all input parameters are saved\n"
                                          "and reused if this application is launched another time.");

    auto* const buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    connect(m_storeParametersCheckBox, &QCheckBox::stateChanged, this, &SettingsDialog::storeParametersCheckStateChanged);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SettingsDialog::okClicked);
    connect(buttonBox, &QDialogButtonBox::rejected, this, [this] {
        QDialog::reject();
    });

    // Set main layout
    auto* const mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(m_storeParametersCheckBox);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    readSettings();
}


void
SettingsDialog::storeParametersCheckStateChanged()
{
    if (m_storeParametersCheckBox->checkState() != Qt::Unchecked) {
        return;
    }

    auto* const msgBox = new QMessageBox();
    msgBox->setIcon(QMessageBox::Information);
    msgBox->setText("Changes will take effect after restarting the application.");
    msgBox->exec();
}


void
SettingsDialog::readSettings()
{
    const auto saveParametersState = Utils::Settings::readAreParametersSaved();
    m_storeParametersCheckBox->setCheckState(saveParametersState ? Qt::Checked : Qt::Unchecked);
}


void
SettingsDialog::writeSettings()
{
    QSettings settings;
    settings.setValue("save", m_storeParametersCheckBox->checkState() == Qt::Checked);
}


void
SettingsDialog::okClicked()
{
    writeSettings();
    QDialog::accept();
}
