#include "BagToImagesWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

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
#include <QSlider>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

BagToImagesWidget::BagToImagesWidget(const Utils::UI::ImageParameters& imageParameters, QWidget *parent) :
    QWidget(parent)
{
    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    auto* const headerTextLabel = new QLabel("Write Images from ROSBag");
    Utils::UI::setWidgetHeaderFont(headerTextLabel);
    headerTextLabel->setAlignment(Qt::AlignHCenter);

    m_bagNameLineEdit = new QLineEdit(imageParameters.bagDirectory);
    m_bagNameLineEdit->setToolTip("The directory of the ROSBag source file.");

    auto* const searchBagButton = new QToolButton;
    auto* const searchBagFileLayout = Utils::UI::createLineEditButtonLayout(m_bagNameLineEdit, searchBagButton);

    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);
    m_topicNameComboBox->setToolTip("The ROSBag topic of the video file.\n"
                                    "If the Bag contains multiple video topics, you can choose one of them.");
    if (!imageParameters.topicName.isEmpty()) {
        m_topicNameComboBox->addItem(imageParameters.topicName);
    }

    m_imagesNameLineEdit = new QLineEdit(imageParameters.imagesDirectory);
    m_imagesNameLineEdit->setToolTip("The directory where the images should be stored.");

    auto* const imagesLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_imagesNameLineEdit, imagesLocationButton);

    m_formatComboBox = new QComboBox;
    m_formatComboBox->addItem("png", 0);
    m_formatComboBox->addItem("jpg", 1);
    m_formatComboBox->setToolTip("The format of the written iimages.");
    if (!imageParameters.format.isEmpty()) {
        m_formatComboBox->setCurrentText(imageParameters.format);
    }

    m_formLayoutSliderLabel = new QLabel;

    m_slider = new QSlider(Qt::Horizontal);
    m_slider->setRange(0, 9);
    m_slider->setTickInterval(1);
    m_slider->setValue(imageParameters.quality);
    m_slider->setTickPosition(QSlider::TicksBelow);

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", searchBagFileLayout);
    formLayout->addRow("Topic Name:", m_topicNameComboBox);
    formLayout->addRow("Images Location:", searchImagesPathLayout);
    formLayout->addRow("Format:", m_formatComboBox);
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
    m_okButton = new QPushButton("Ok");
    m_okButton->setEnabled(false);

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
    m_okButton->setEnabled(!m_bagNameLineEdit->text().isEmpty() &&
                           !m_topicNameComboBox->currentText().isEmpty() &&
                           !m_imagesNameLineEdit->text().isEmpty());

    adjustSliderToChangedFormat(imageParameters.format);

    connect(searchBagButton, &QPushButton::clicked, this, &BagToImagesWidget::searchButtonPressed);
    connect(imagesLocationButton, &QPushButton::clicked, this, &BagToImagesWidget::imagesLocationButtonPressed);
    connect(m_formatComboBox, &QComboBox::currentTextChanged, this, &BagToImagesWidget::adjustSliderToChangedFormat);
    connect(backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &BagToImagesWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BagToImagesWidget::okButtonPressed);
}


void
BagToImagesWidget::searchButtonPressed()
{
    m_bagNameLineEdit->setText(QFileDialog::getExistingDirectory(this, "Open Directory", "",
                                                                 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
    if (m_bagNameLineEdit->text().isEmpty()) {
        return;
    }

    m_topicNameComboBox->clear();
    const auto videoTopics = Utils::ROS::getBagVideoTopics(m_bagNameLineEdit->text().toStdString());
    // Only enable if both line edits contain text
    m_okButton->setEnabled(!videoTopics.empty() && !m_imagesNameLineEdit->text().isEmpty());

    if (videoTopics.empty()) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Topic not found!",
                                             "The bag file does not contain any image/video topics!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    for (const auto& videoTopic : videoTopics) {
        m_topicNameComboBox->addItem(QString::fromStdString(videoTopic));
    }
}


void
BagToImagesWidget::imagesLocationButtonPressed()
{
    const auto fileName = QFileDialog::getExistingDirectory(this, "Save Images", "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    m_imagesNameLineEdit->setText(fileName);
    // Only enable if both line edits contain text
    m_okButton->setEnabled(!m_topicNameComboBox->currentText().isEmpty() && !m_imagesNameLineEdit->text().isEmpty());
    m_fileDialogOpened = true;
}


void
BagToImagesWidget::adjustSliderToChangedFormat(const QString& text)
{
    m_formLayoutSliderLabel->setText(text == "jpg" ? "Quality:" : "Level of Compression:");
    m_slider->setToolTip(text == "jpg" ? "Image quality. A higher quality will increase the image size."
                                       : "Higher compression will result in smaller size, but increase writing time.");
}


void
BagToImagesWidget::formatComboBoxTextChanged(const QString& text)
{
    // If the combo box item changes, apply a different appendix to the text in the video line edit
    if (m_imagesNameLineEdit->text().isEmpty()) {
        return;
    }

    auto newLineEditText = m_imagesNameLineEdit->text();
    newLineEditText.truncate(newLineEditText.lastIndexOf(QChar('.')));
    newLineEditText += "." + text;
    m_imagesNameLineEdit->setText(newLineEditText);
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
                                             "Images already exist under the specified directory! Are you sure you want to continue? This will overwrite all existing files.",
                                             QMessageBox::Yes | QMessageBox::No);
        const auto ret = msgBox->exec();
        if (ret == QMessageBox::No) {
            return;
        }
    }

    emit parametersSet(m_bagNameLineEdit->text(), m_topicNameComboBox->currentText(), m_imagesNameLineEdit->text(),
                       m_formatComboBox->currentText(), m_slider->value());
}


void
BagToImagesWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg").pixmap(QSize(100, 45)));
}


bool
BagToImagesWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
