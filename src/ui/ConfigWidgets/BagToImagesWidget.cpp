#include "BagToImagesWidget.hpp"

#include "UtilsROS.hpp"

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
    BasicConfigWidget(":/icons/bag_to_images_white.svg", ":/icons/bag_to_images_black.svg", parent),
    m_imageParameters(imageParameters)
{
    auto* const headerTextLabel = new QLabel("Write Images from ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_bagNameLineEdit = new QLineEdit(m_imageParameters.bagDirectory);
    m_bagNameLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, searchBagButton);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!m_imageParameters.bagDirectory.isEmpty()) {
        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_imageParameters.bagDirectory);
    }

    m_imagesNameLineEdit = new QLineEdit(m_imageParameters.imagesDirectory);
    m_imagesNameLineEdit->setToolTip("The directory where the images should be stored.");

    auto* const imagesLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_imagesNameLineEdit, imagesLocationButton);

    auto* const formatComboBox = new QComboBox;
    formatComboBox->addItem("png", 0);
    formatComboBox->addItem("jpg", 1);
    formatComboBox->setToolTip("The format of the written iimages.");
    formatComboBox->setCurrentText(m_imageParameters.format);

    m_formLayoutSliderLabel = new QLabel;

    m_slider = new QSlider(Qt::Horizontal);
    m_slider->setRange(0, 9);
    m_slider->setTickInterval(1);
    m_slider->setValue(m_imageParameters.quality);
    m_slider->setTickPosition(QSlider::TicksBelow);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", searchBagFileLayout);
    formLayout->addRow("Topic Name:", m_topicNameComboBox);
    formLayout->addRow("Images Location:", searchImagesPathLayout);
    formLayout->addRow("Format:", formatComboBox);
    formLayout->addRow(m_formLayoutSliderLabel, m_slider);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(headerTextLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(formLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const backButton = new QPushButton("Back");

    auto* const buttonBox = new QDialogButtonBox;
    buttonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(backButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(buttonBox);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() && !m_topicNameComboBox->currentText().isEmpty() &&
                   !m_imagesNameLineEdit->text().isEmpty());

    adjustSliderToChangedFormat(m_imageParameters.format);

    connect(searchBagButton, &QPushButton::clicked, this, &BagToImagesWidget::searchButtonPressed);
    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        m_imageParameters.topicName = text;
    });
    connect(imagesLocationButton, &QPushButton::clicked, this, &BagToImagesWidget::imagesLocationButtonPressed);
    connect(formatComboBox, &QComboBox::currentTextChanged, this, &BagToImagesWidget::adjustSliderToChangedFormat);
    connect(m_slider, &QSlider::valueChanged, this, [this] (int value) {
        m_imageParameters.quality = value;
    });
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &BagToImagesWidget::okButtonPressed);
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

    m_imageParameters.bagDirectory = bagDirectory;
    m_bagNameLineEdit->setText(bagDirectory);
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_imagesNameLineEdit->text().isEmpty());
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
    m_imageParameters.imagesDirectory = fileName;
    m_imagesNameLineEdit->setText(fileName);
    enableOkButton(!m_bagNameLineEdit->text().isEmpty() &&
                   !m_topicNameComboBox->currentText().isEmpty() && !m_imagesNameLineEdit->text().isEmpty());
}


void
BagToImagesWidget::adjustSliderToChangedFormat(const QString& text)
{
    m_formLayoutSliderLabel->setText(text == "jpg" ? "Quality:" : "Level of Compression:");
    m_slider->setToolTip(text == "jpg" ? "Image quality. A higher quality will increase the image size."
                                       : "Higher compression will result in smaller size, but increase writing time.");
    m_imageParameters.format = text;
}


void
BagToImagesWidget::okButtonPressed()
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    // Only ask if exists and the file dialog has not been called
    if (!std::filesystem::is_empty(m_imagesNameLineEdit->text().toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, "Images already exist!",
                                             "Images already exist under the specified directory! Are you sure you want to continue? "
                                             "This will overwrite all existing files.",
                                             QMessageBox::Yes | QMessageBox::No);
        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return;
        }
    }

    emit okPressed();
}
