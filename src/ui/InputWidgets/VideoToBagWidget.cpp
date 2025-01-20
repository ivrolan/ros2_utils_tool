#include "VideoToBagWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
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
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

VideoToBagWidget::VideoToBagWidget(Utils::UI::BagInputParameters& parameters, QWidget *parent) :
    BasicInputWidget("Write Video to a ROSBag", ":/icons/video_to_bag", parent),
    m_parameters(parameters), m_settings(parameters, "vid_to_bag")
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the source video file.");

    m_bagNameLineEdit = new QLineEdit(m_parameters.targetDirectory);
    m_bagNameLineEdit->setToolTip("The directory where the ROSBag should be stored.");
    auto* const bagLocationButton = new QToolButton;
    auto* const storeBagLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, bagLocationButton);

    auto* const topicNameLineEdit = new QLineEdit(m_parameters.topicName);
    topicNameLineEdit->setToolTip("The video's topic name inside the ROSBag.");

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Video File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Bag Location:", storeBagLayout);
    basicOptionsFormLayout->addRow("Topic Name:", topicNameLineEdit);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const useCustomFPSCheckBox = new QCheckBox;
    useCustomFPSCheckBox->setToolTip("Use custom fps for the bag file. If this is unchecked, the video's fps will be used.");
    useCustomFPSCheckBox->setCheckState(m_parameters.useCustomFPS ? Qt::Checked : Qt::Unchecked);

    auto* const useHardwareAccCheckBox = new QCheckBox;
    useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video decoding and writing.");
    useHardwareAccCheckBox->setCheckState(m_parameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Use Custom FPS:", useCustomFPSCheckBox);
    m_advancedOptionsFormLayout->addRow("Hardware Accleration:", useHardwareAccCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(m_advancedOptionsFormLayout);
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
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());

    connect(m_findSourceButton, &QPushButton::clicked, this, &VideoToBagWidget::searchButtonPressed);
    connect(bagLocationButton, &QPushButton::clicked, this, &VideoToBagWidget::bagLocationButtonPressed);
    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        writeSettingsParameter(m_parameters.topicName, topicNameLineEdit->text(), m_settings);
        enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
    });
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_parameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(useCustomFPSCheckBox, &QCheckBox::stateChanged, this, &VideoToBagWidget::useCustomFPSCheckBoxPressed);
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_parameters.useHardwareAcceleration, state == Qt::Checked, m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &VideoToBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &VideoToBagWidget::okButtonPressed);

    useCustomFPSCheckBoxPressed(m_parameters.useCustomFPS);
}


void
VideoToBagWidget::searchButtonPressed()
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

    QDir videoDirectoryDir(videoDir);
    videoDirectoryDir.cdUp();
    if (const auto autoBagDirectory = videoDirectoryDir.path() + "/video_bag"; !std::filesystem::exists(autoBagDirectory.toStdString())) {
        m_bagNameLineEdit->setText(autoBagDirectory);
        writeSettingsParameter(m_parameters.targetDirectory, autoBagDirectory, m_settings);
    }

    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


void
VideoToBagWidget::bagLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save ROSBag");
    if (fileName.isEmpty()) {
        return;
    }

    writeSettingsParameter(m_parameters.targetDirectory, fileName, m_settings);
    m_bagNameLineEdit->setText(fileName);
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.targetDirectory.isEmpty() && !m_parameters.topicName.isEmpty());
}


void
VideoToBagWidget::useCustomFPSCheckBoxPressed(int state)
{
    // Partially checked value can still count for this case
    writeSettingsParameter(m_parameters.useCustomFPS, state != Qt::Unchecked, m_settings);

    if (state != Qt::Unchecked) {
        m_fpsSpinBox = new QSpinBox;
        m_fpsSpinBox->setRange(10, 60);
        m_fpsSpinBox->setValue(m_parameters.fps);
        m_fpsSpinBox->setToolTip("FPS of the video stored in the bag.");

        m_advancedOptionsFormLayout->insertRow(1, "", m_fpsSpinBox);

        connect(m_fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeSettingsParameter(m_parameters.fps, value, m_settings);
        });
    } else if (m_fpsSpinBox) {
        m_advancedOptionsFormLayout->removeRow(m_fpsSpinBox);
    }
}


void
VideoToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!Utils::ROS::doesTopicNameFollowROS2Convention(m_parameters.topicName)) {
        Utils::UI::createCriticalMessageBox("Wrong topic name format!",
                                            "The topic name does not follow the ROS2 naming convention! More information for naming ROS2 topics can be found here:<br>"
                                            "<a href='https://design.ros2.org/articles/topic_and_service_names.html'>https://design.ros2.org/articles/topic_and_service_names.html</a>");
        return;
    }
    if (std::filesystem::exists(m_parameters.targetDirectory.toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Bag file already exists!",
                                             "A bag file already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
