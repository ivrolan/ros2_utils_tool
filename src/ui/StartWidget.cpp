#include "StartWidget.hpp"

#include "UtilsUI.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

StartWidget::StartWidget(QWidget *parent) :
    QWidget(parent)
{
    auto* const headerLabel = new QLabel("ROS TOOLS");
    Utils::UI::setWidgetFontSize(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    m_bagToVideoPushButton = createToolButton("Encode Video\nfrom ROSBag");
    m_bagToImagesPushButton = createToolButton("Write Images\nfrom ROSBag");
    m_videoToBagPushButton = createToolButton("Write Video\nto ROSBag");
    m_dummyBagButton = createToolButton("Create Dummy\nROSBag");

    setButtonIcons();

    auto* const upperButtonLayout = new QHBoxLayout;
    upperButtonLayout->addStretch();
    upperButtonLayout->addWidget(m_bagToVideoPushButton);
    upperButtonLayout->addWidget(m_bagToImagesPushButton);
    upperButtonLayout->addStretch();

    auto* const lowerButtonLayout = new QHBoxLayout;
    lowerButtonLayout->addStretch();
    lowerButtonLayout->addWidget(m_videoToBagPushButton);
    lowerButtonLayout->addWidget(m_dummyBagButton);
    lowerButtonLayout->addStretch();

    auto* const versionLabel = new QLabel("v0.2.0");
    versionLabel->setToolTip("Bag to Images feature, saved parameters and bugfixes!");

    auto* const versionLayout = new QHBoxLayout;
    versionLayout->addStretch();
    versionLayout->addWidget(versionLabel);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addSpacing(40);
    mainLayout->addWidget(headerLabel);
    mainLayout->addStretch();
    mainLayout->addLayout(upperButtonLayout);
    mainLayout->addLayout(lowerButtonLayout);
    mainLayout->addStretch();
    mainLayout->addLayout(versionLayout);

    setLayout(mainLayout);

    connect(m_bagToVideoPushButton, &QPushButton::clicked, this, [this] {
        emit functionRequested(0);
    });
    connect(m_bagToImagesPushButton, &QPushButton::clicked, this, [this] {
        emit functionRequested(1);
    });
    connect(m_videoToBagPushButton, &QPushButton::clicked, this, [this] {
        emit functionRequested(2);
    });
    connect(m_dummyBagButton, &QPushButton::clicked, this, [this] {
        emit functionRequested(3);
    });
}


QPointer<QToolButton>
StartWidget::createToolButton(const QString& buttonText)
{
    auto* const toolButton = new QToolButton;
    toolButton->setText(buttonText);
    toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    toolButton->setIconSize(QSize(100, 45));
    toolButton->setFixedSize(QSize(150, 150));

    Utils::UI::setWidgetFontSize(toolButton, true);

    return toolButton;
}


void
StartWidget::setButtonIcons()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg"));
    m_bagToImagesPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg"));
    m_dummyBagButton->setIcon(QIcon(isDarkMode ? ":/icons/dummy_bag_white.svg" : ":/icons/dummy_bag_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
