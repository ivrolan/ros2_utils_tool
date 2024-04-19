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
    m_bagToVideoPushButton = createToolButton("Encode Video\nfrom ROSBag");
    m_videoToBagPushButton = createToolButton("Write Video\nto ROSBag");

    setButtonIcons();

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_bagToVideoPushButton);
    buttonLayout->addWidget(m_videoToBagPushButton);
    buttonLayout->addStretch();

    auto* const versionLabel = new QLabel("v0.1.0");
    versionLabel->setToolTip("Initial version.");

    auto* const versionLayout = new QHBoxLayout;
    versionLayout->addStretch();
    versionLayout->addWidget(versionLabel);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addStretch();
    mainLayout->addLayout(buttonLayout);
    mainLayout->addStretch();
    mainLayout->addLayout(versionLayout);

    setLayout(mainLayout);

    connect(m_bagToVideoPushButton, &QPushButton::clicked, this, [this] {
        emit bagToVideoRequested();
    });
    connect(m_videoToBagPushButton, &QPushButton::clicked, this, [this] {
        emit videoToBagRequested();
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

    Utils::UI::setWidgetHeaderFont(toolButton);

    return toolButton;
}


void
StartWidget::setButtonIcons()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
