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

VideoToBagWidget::VideoToBagWidget(Utils::UI::BagParameters& bagParameters, QWidget *parent) :
    BasicInputWidget("Write Video to a ROSBag", ":/icons/video_to_bag_white.svg", ":/icons/video_to_bag_black.svg", parent),
    m_bagParameters(bagParameters), m_bagParamSettings(bagParameters, "vid_to_bag")
{
    m_videoNameLineEdit = new QLineEdit(m_bagParameters.sourceDirectory);
    m_videoNameLineEdit->setToolTip("The directory of the source video file.");
    auto* const searchVideoFileButton = new QToolButton;
    auto* const searchVideoFileLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, searchVideoFileButton);

    m_bagNameLineEdit = new QLineEdit(m_bagParameters.targetDirectory);
    m_bagNameLineEdit->setToolTip("The directory where the ROSBag should be stored.");
    auto* const bagLocationButton = new QToolButton;
    auto* const storeBagLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, bagLocationButton);

    auto* const topicNameLineEdit = new QLineEdit(m_bagParameters.topicName);
    topicNameLineEdit->setToolTip("The video's topic name inside the ROSBag.");

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Video File:", searchVideoFileLayout);
    basicOptionsFormLayout->addRow("Bag Location:", storeBagLayout);
    basicOptionsFormLayout->addRow("Topic Name:", topicNameLineEdit);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_bagParameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("sqlite3", 0);
    formatComboBox->addItem("cdr", 1);
    formatComboBox->setCurrentText(m_bagParameters.useCDRForSerialization ? "cdr" : "sqlite3");
    formatComboBox->setToolTip("The format used to serialize the single image messages in the bag.");

    auto* const spinBox = new QSpinBox;
    spinBox->setRange(10, 60);
    spinBox->setValue(m_bagParameters.fps);
    spinBox->setToolTip("FPS of the video stored in the bag.");

    auto* const useHardwareAccCheckBox = new QCheckBox;
    useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video decoding and writing.");
    useHardwareAccCheckBox->setCheckState(m_bagParameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("Serialization Format:", formatComboBox);
    advancedOptionsFormLayout->addRow("FPS:", spinBox);
    advancedOptionsFormLayout->addRow("Hardware Accleration:", useHardwareAccCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_bagParameters.showAdvancedOptions);

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
    enableOkButton(!m_bagParameters.sourceDirectory.isEmpty() && !m_bagParameters.targetDirectory.isEmpty() && !m_bagParameters.topicName.isEmpty());

    connect(searchVideoFileButton, &QPushButton::clicked, this, &VideoToBagWidget::searchButtonPressed);
    connect(bagLocationButton, &QPushButton::clicked, this, &VideoToBagWidget::bagLocationButtonPressed);
    connect(topicNameLineEdit, &QLineEdit::textChanged, this, [this, topicNameLineEdit] {
        m_bagParameters.topicName = topicNameLineEdit->text();
        m_bagParamSettings.write();
        enableOkButton(!m_bagParameters.sourceDirectory.isEmpty() && !m_bagParameters.targetDirectory.isEmpty() && !m_bagParameters.topicName.isEmpty());
    });
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_bagParameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(formatComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        m_bagParameters.useCDRForSerialization = text == "cdr";
        m_bagParamSettings.write();
    });
    connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        m_bagParameters.fps = value;
        m_bagParamSettings.write();
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_bagParameters.useHardwareAcceleration = state == Qt::Checked;
        m_bagParamSettings.write();
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &VideoToBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &VideoToBagWidget::okButtonPressed);
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

    m_bagParameters.sourceDirectory = videoDir;
    m_bagParamSettings.write();
    m_videoNameLineEdit->setText(videoDir);

    QDir videoDirectoryDir(videoDir);
    videoDirectoryDir.cdUp();
    if (const auto autoBagDirectory = videoDirectoryDir.path() + "/video_bag"; !std::filesystem::exists(autoBagDirectory.toStdString())) {
        m_bagNameLineEdit->setText(autoBagDirectory);
        m_bagParameters.targetDirectory = autoBagDirectory;
        m_bagParamSettings.write();
    }

    enableOkButton(!m_bagParameters.sourceDirectory.isEmpty() && !m_bagParameters.targetDirectory.isEmpty() && !m_bagParameters.topicName.isEmpty());
}


void
VideoToBagWidget::bagLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save ROSBag");
    if (fileName.isEmpty()) {
        return;
    }

    m_bagParameters.targetDirectory = fileName;
    m_bagParamSettings.write();
    m_bagNameLineEdit->setText(fileName);
    enableOkButton(!m_bagParameters.sourceDirectory.isEmpty() && !m_bagParameters.targetDirectory.isEmpty() && !m_bagParameters.topicName.isEmpty());
}


void
VideoToBagWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    if (!Utils::ROS::doesTopicNameFollowROS2Convention(m_bagParameters.topicName)) {
        Utils::UI::createCriticalMessageBox("Wrong topic name format!", "The topic name does not follow the ROS2 naming convention!");
        return;
    }
    if (std::filesystem::exists(m_bagParameters.targetDirectory.toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Bag file already exists!",
                                             "A bag file already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
