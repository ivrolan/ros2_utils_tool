#include "BagToVideoWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QEvent>
#include <QFileDialog>
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

BagToVideoWidget::BagToVideoWidget(Utils::UI::VideoParameters& videoParameters, QString& encodingFormat, QWidget *parent) :
    BasicInputWidget("Encode Video from ROSBag", ":/icons/bag_to_video_white.svg", ":/icons/bag_to_video_black.svg", parent),
    m_videoParameters(videoParameters), m_encodingFormat(encodingFormat)
{
    m_bagNameLineEdit = new QLineEdit(m_videoParameters.sourceDirectory);
    m_bagNameLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, searchBagButton);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!m_videoParameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_videoParameters.sourceDirectory);
    }

    m_videoNameLineEdit = new QLineEdit(m_videoParameters.targetDirectory);
    m_videoNameLineEdit->setToolTip("The directory where the video file should be stored.");

    auto* const videoLocationButton = new QToolButton;
    auto* const searchVideoPathLayout = Utils::UI::createLineEditButtonLayout(m_videoNameLineEdit, videoLocationButton);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("mp4", 0);
    m_formatComboBox->addItem("mkv", 1);
    m_formatComboBox->setToolTip("The video format file.");
    if (!m_videoParameters.targetDirectory.isEmpty()) {
        QFileInfo fileInfo(m_videoParameters.targetDirectory);
        m_formatComboBox->setCurrentText(fileInfo.suffix());
    } else {
        m_formatComboBox->setCurrentText(m_encodingFormat);
    }

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", searchBagFileLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("Video Location:", searchVideoPathLayout);
    basicOptionsFormLayout->addRow("Format:", m_formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_videoParameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    auto* const fpsSpinBox = new QSpinBox;
    fpsSpinBox->setRange(10, 60);
    fpsSpinBox->setValue(m_videoParameters.fps);
    fpsSpinBox->setToolTip("FPS of the encoded video.");

    auto* const useHardwareAccCheckBox = new QCheckBox;
    useHardwareAccCheckBox->setToolTip("Enable hardware acceleration for faster video encoding.");
    useHardwareAccCheckBox->setCheckState(m_videoParameters.useHardwareAcceleration ? Qt::Checked : Qt::Unchecked);

    auto* const useBWImagesCheckBox = new QCheckBox;
    useBWImagesCheckBox->setToolTip("Write a colorless video.");
    useBWImagesCheckBox->setCheckState(m_videoParameters.useBWImages ? Qt::Checked : Qt::Unchecked);

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("FPS:", fpsSpinBox);
    advancedOptionsFormLayout->addRow("HW Acceleration:", useHardwareAccCheckBox);
    advancedOptionsFormLayout->addRow("Use Colorless Images:", useBWImagesCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_videoParameters.showAdvancedOptions);

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
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() &&
                   !m_videoNameLineEdit->text().isEmpty());


    connect(searchBagButton, &QPushButton::clicked, this, &BagToVideoWidget::searchButtonPressed);
    connect(videoLocationButton, &QPushButton::clicked, this, &BagToVideoWidget::videoLocationButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        m_videoParameters.topicName = text;
    });
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToVideoWidget::formatComboBoxTextChanged);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_videoParameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(fpsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        m_videoParameters.fps = value;
    });
    connect(useHardwareAccCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_videoParameters.useHardwareAcceleration = state == Qt::Checked;
    });
    connect(useBWImagesCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_videoParameters.useBWImages = state == Qt::Checked;
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &BagToVideoWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToVideoWidget::okButtonPressed);
}


void
BagToVideoWidget::searchButtonPressed()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open ROSBag", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }

    if (const auto containsVideoTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, bagDirectory); !containsVideoTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any image/video topics!");
        return;
    }

    m_bagNameLineEdit->setText(bagDirectory);
    m_videoParameters.sourceDirectory = bagDirectory;

    QDir bagDirectoryDir(bagDirectory);
    bagDirectoryDir.cdUp();
    if (const auto autoVideoDirectory = bagDirectoryDir.path() + "/bag_video." + m_formatComboBox->currentText();
        !std::filesystem::exists(autoVideoDirectory.toStdString())) {
        m_videoNameLineEdit->setText(autoVideoDirectory);
        m_videoParameters.targetDirectory = autoVideoDirectory;
    }

    // Only enable if both line edits contain text
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_videoNameLineEdit->text().isEmpty());
}


void
BagToVideoWidget::videoLocationButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Video", "",
                                                       m_formatComboBox->currentText() + " files (*." + m_formatComboBox->currentText() + ")");
    if (fileName.isEmpty()) {
        return;
    }

    // Only enable if both line edits contain text
    m_fileDialogOpened = true;
    m_videoParameters.targetDirectory = fileName;
    m_videoNameLineEdit->setText(fileName);
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_videoNameLineEdit->text().isEmpty());
}


void
BagToVideoWidget::formatComboBoxTextChanged(const QString& text)
{
    m_encodingFormat = text;
    // If the combo box item changes, apply a different appendix to the text in the video line edit
    if (m_videoNameLineEdit->text().isEmpty()) {
        return;
    }

    auto newLineEditText = m_videoNameLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;

    m_videoParameters.targetDirectory = newLineEditText;
    m_videoNameLineEdit->setText(newLineEditText);
}


void
BagToVideoWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    // Only ask if exists and the file dialog has not been called
    if (std::filesystem::exists(m_videoNameLineEdit->text().toStdString()) && !m_fileDialogOpened) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Video already exists!",
                                             "A video already exists under the specified directory! Are you sure you want to continue? This will overwrite the existing file.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
