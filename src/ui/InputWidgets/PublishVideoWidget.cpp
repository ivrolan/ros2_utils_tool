#include "PublishVideoWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

#include <filesystem>

PublishVideoWidget::PublishVideoWidget(Utils::UI::PublishVideoParameters& parameters,
                                       bool checkROS2NameConform, QWidget *parent) :
    BasicInputWidget("Publish Video as ROS Message", ":/icons/publish_video", parent),
    m_parameters(parameters), m_settings(parameters, "publish_vid"),
    m_checkROS2NameConform(checkROS2NameConform)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the source video file.");

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("Name of the topic to be published.");

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Video File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", topicNameLineEdit);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const useHardwareAccCheckBox = Utils::UI::createCheckBox("Enable hardware acceleration for faster video decoding.",
                                                                   m_parameters.useHardwareAcceleration);
    auto* const switchRedBlueCheckBox = Utils::UI::createCheckBox("Switch the video's red and blue values.", m_parameters.switchRedBlueValues);
    auto* const loopCheckBox = Utils::UI::createCheckBox("Loop the video file if it has been played through.", m_parameters.loop);

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("Hardware Accleration:", useHardwareAccCheckBox);
    advancedOptionsFormLayout->addRow("Switch Red and Blue Values:", switchRedBlueCheckBox);
    advancedOptionsFormLayout->addRow("Loop Video:", loopCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(basicOptionsFormLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(advancedOptionsCheckBox);
    controlsLayout->addSpacing(10);
    controlsLayout->addWidget(advancedOptionsWidget);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(m_findSourceButton, &QPushButton::clicked, this, &PublishVideoWidget::searchButtonPressed);
    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeSettingsParameter(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_parameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.useHardwareAcceleration, state == Qt::Checked, m_settings);
    });
    connect(switchRedBlueCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.switchRedBlueValues, state == Qt::Checked, m_settings);
    });
    connect(loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.loop, state == Qt::Checked, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &PublishVideoWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &PublishVideoWidget::okButtonPressed);
}


void
PublishVideoWidget::searchButtonPressed()
{
    enableOkButton(false);

    const auto videoDir = QFileDialog::getOpenFileName(this, "Open Video");
    if (videoDir.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(videoDir);
    if (fileInfo.suffix().toLower() != "mp4" && fileInfo.suffix().toLower() != "mkv") {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Wrong format!", "The video must be in mp4 or mkv format!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    writeSettingsParameter(m_parameters.sourceDirectory, videoDir, m_settings);
    m_sourceLineEdit->setText(videoDir);

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


void
PublishVideoWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (m_checkROS2NameConform && !Utils::ROS::isNameROS2Conform(m_parameters.topicName)) {
        auto *const msgBox = Utils::UI::createInvalidROSNameMessageBox();

        if (const auto returnValue = msgBox->exec(); returnValue == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
