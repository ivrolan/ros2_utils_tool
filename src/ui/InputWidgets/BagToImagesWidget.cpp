#include "BagToImagesWidget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

BagToImagesWidget::BagToImagesWidget(Utils::UI::ImageParameters& imageParameters, QWidget *parent) :
    BasicInputWidget("Write Images from ROSBag", ":/icons/bag_to_images", parent),
    m_imageParameters(imageParameters), m_imageParamSettings(imageParameters, "bag_to_images")
{
    m_sourceLineEdit->setText(imageParameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The directory of the ROSBag source file.");

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!m_imageParameters.sourceDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_imageParameters.sourceDirectory);

        if (!m_imageParameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_imageParameters.topicName);
        }
    }

    m_imagesNameLineEdit = new QLineEdit(m_imageParameters.targetDirectory);
    m_imagesNameLineEdit->setToolTip("The directory where the images should be stored.");

    auto* const imagesLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_imagesNameLineEdit, imagesLocationButton);

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("jpg", 0);
    formatComboBox->addItem("png", 1);
    formatComboBox->addItem("bmp", 2);
    formatComboBox->setToolTip("The format of the written images.");
    formatComboBox->setCurrentText(m_imageParameters.format);

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("Images Location:", searchImagesPathLayout);
    basicOptionsFormLayout->addRow("Format:", formatComboBox);

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_imageParameters.showAdvancedOptions ? Qt::Checked : Qt::Unchecked);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    m_useBWCheckBox = new QCheckBox;
    m_useBWCheckBox->setChecked(m_imageParameters.useBWImages ? Qt::Checked : Qt::Unchecked);
    m_useBWCheckBox->setToolTip("If the images should be colorless or not.");

    m_advancedOptionsFormLayout = new QFormLayout;
    m_advancedOptionsFormLayout->addRow("Colorless Images:", m_useBWCheckBox);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(m_advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_imageParameters.showAdvancedOptions);

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

    adjustWidgetsToChangedFormat(m_imageParameters.format);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_imageParameters.sourceDirectory.isEmpty() &&
                   !m_imageParameters.topicName.isEmpty() && !m_imageParameters.targetDirectory.isEmpty());


    connect(m_findSourceButton, &QPushButton::clicked, this, &BagToImagesWidget::searchButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeSettingsParameter(m_imageParameters.topicName, text, m_imageParamSettings);
    });
    connect(imagesLocationButton, &QPushButton::clicked, this, &BagToImagesWidget::imagesLocationButtonPressed);
    connect(formatComboBox, &QComboBox::currentTextChanged, this, &BagToImagesWidget::adjustWidgetsToChangedFormat);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        m_imageParameters.showAdvancedOptions = state == Qt::Checked;
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(m_useBWCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeSettingsParameter(m_imageParameters.useBWImages, state == Qt::Checked, m_imageParamSettings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &BagToImagesWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToImagesWidget::okButtonPressed);
}


void
BagToImagesWidget::searchButtonPressed()
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

    m_sourceLineEdit->setText(bagDirectory);
    writeSettingsParameter(m_imageParameters.sourceDirectory, bagDirectory, m_imageParamSettings);

    QDir bagDirectoryDir(bagDirectory);
    bagDirectoryDir.cdUp();
    if (const auto autoImageDirectory = bagDirectoryDir.path() + "/bag_images"; !std::filesystem::exists(autoImageDirectory.toStdString())) {
        m_imagesNameLineEdit->setText(autoImageDirectory);
        writeSettingsParameter(m_imageParameters.targetDirectory, autoImageDirectory, m_imageParamSettings);
    }

    enableOkButton(!m_imageParameters.sourceDirectory.isEmpty() &&
                   !m_imageParameters.topicName.isEmpty() && !m_imageParameters.targetDirectory.isEmpty());
}


void
BagToImagesWidget::imagesLocationButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Save Images", "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (fileName.isEmpty()) {
        return;
    }

    // Only enable if both line edits contain text
    m_fileDialogOpened = true;
    writeSettingsParameter(m_imageParameters.targetDirectory, fileName, m_imageParamSettings);
    m_imagesNameLineEdit->setText(fileName);
    enableOkButton(!m_imageParameters.sourceDirectory.isEmpty() &&
                   !m_imageParameters.topicName.isEmpty() && !m_imageParameters.targetDirectory.isEmpty());
}


void
BagToImagesWidget::adjustWidgetsToChangedFormat(const QString& text)
{
    writeSettingsParameter(m_imageParameters.format, text, m_imageParamSettings);

    if (m_optimizeOrBilevelCheckBox && m_slider) {
        m_advancedOptionsFormLayout->removeRow(m_optimizeOrBilevelCheckBox);
        m_advancedOptionsFormLayout->removeRow(m_slider);
    }
    if (text == "bmp") {
        return;
    }

    m_slider = new QSlider(Qt::Horizontal);
    m_slider->setRange(0, 9);
    m_slider->setTickInterval(1);
    m_slider->setValue(m_imageParameters.quality);
    m_slider->setTickPosition(QSlider::TicksBelow);
    m_slider->setToolTip(text == "jpg" ? "Image quality. A higher quality will increase the image size."
                                       : "Higher compression will result in smaller size, but increase writing time.");

    m_optimizeOrBilevelCheckBox = new QCheckBox;
    auto& memberVal = text == "jpg" ? m_imageParameters.jpgOptimize : m_imageParameters.pngBilevel;
    m_optimizeOrBilevelCheckBox->setChecked(memberVal ? Qt::Checked : Qt::Unchecked);
    m_optimizeOrBilevelCheckBox->setToolTip(text == "jpg" ? "Optimize the stored file size." : "Save as an image containing only black and white pixels.");

    m_advancedOptionsFormLayout->insertRow(0, text == "jpg" ? "Quality:" : "Level of Compression:", m_slider);
    m_advancedOptionsFormLayout->insertRow(1, text == "jpg" ? "Optimize Size" : "Binary Image", m_optimizeOrBilevelCheckBox);

    connect(m_slider, &QSlider::valueChanged, this, [this] (int value) {
        writeSettingsParameter(m_imageParameters.quality, value, m_imageParamSettings);
    });
    connect(m_optimizeOrBilevelCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        m_imageParameters.format == "jpg" ?
        writeSettingsParameter(m_imageParameters.jpgOptimize, state == Qt::Checked, m_imageParamSettings) :
        writeSettingsParameter(m_imageParameters.pngBilevel, state == Qt::Checked, m_imageParamSettings);
    });
}


void
BagToImagesWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    // Only ask if exists and the file dialog has not been called
    if (std::filesystem::exists(m_imagesNameLineEdit->text().toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Directory already exists!",
                                             "The specified directory already exists! Are you sure you want to continue? "
                                             "This will overwrite all potentially existing images.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
