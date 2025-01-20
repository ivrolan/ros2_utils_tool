#include "SettingsDialog.hpp"

#include "BasicSettings.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QSettings>
#include <QVBoxLayout>

SettingsDialog::SettingsDialog(Utils::UI::DialogParameters& dialogParameters, QWidget* parent) :
    QDialog(parent), m_dialogSettings(dialogParameters, "dialog"), m_dialogParameters(dialogParameters)
{
    setWindowTitle("Options");

    m_storeParametersCheckBox = new QCheckBox("Save Input Parameters");
    m_storeParametersCheckBox->setTristate(false);
    m_storeParametersCheckBox->setToolTip("If this is checked, all input parameters are saved\n"
                                          "and reused if this application is launched another time.");
    m_storeParametersCheckBox->setCheckState(m_dialogParameters.saveParameters ? Qt::Checked : Qt::Unchecked);

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
}


void
SettingsDialog::storeParametersCheckStateChanged()
{
    m_dialogParameters.saveParameters = m_storeParametersCheckBox->checkState() == Qt::Checked;

    auto* const msgBox = new QMessageBox();
    msgBox->setIcon(QMessageBox::Information);
    msgBox->setText("Changes will take effect after restarting the application.");
    msgBox->exec();
}


void
SettingsDialog::okClicked()
{
    m_dialogSettings.write();
    QDialog::accept();
}
