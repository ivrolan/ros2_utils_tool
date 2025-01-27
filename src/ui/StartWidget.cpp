#include "StartWidget.hpp"

#include "SettingsDialog.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

StartWidget::StartWidget(Utils::UI::DialogParameters& dialogParameters, QWidget *parent) :
    QWidget(parent), m_dialogParameters(dialogParameters), m_isBagToolsWidgetSelected{true}
{
    auto* const headerLabel = new QLabel("ROS TOOLS");
    Utils::UI::setWidgetFontSize(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    m_settingsButton = new QPushButton;
    m_settingsButton->setFlat(true);

    auto* const settingsButtonLayout = new QHBoxLayout;
    settingsButtonLayout->addStretch();
    settingsButtonLayout->addWidget(m_settingsButton);

    // Overall widget
    m_bagToolsButton = createToolButton("Bag Tools");
    m_publishingToolsButton = createToolButton("Publishing\nTools");

    auto* const overallToolsButtonLayout = new QHBoxLayout;
    overallToolsButtonLayout->addStretch();
    overallToolsButtonLayout->addWidget(m_bagToolsButton);
    overallToolsButtonLayout->addWidget(m_publishingToolsButton);
    overallToolsButtonLayout->addStretch();

    auto* const overallToolsWidget = new QWidget;
    overallToolsWidget->setLayout(overallToolsButtonLayout);

    // Bag tools widget
    m_editROSBagButton = createToolButton("Edit\nROSBag");
    m_bagInfoButton = createToolButton("Get Infos\nfrom ROSBag");
    m_bagToVideoPushButton = createToolButton("Encode Video\nfrom ROSBag");
    m_videoToBagPushButton = createToolButton("Write Video\nto ROSBag");
    m_bagToImagesPushButton = createToolButton("Write Images\nfrom ROSBag");
    m_dummyBagButton = createToolButton("Create Dummy\nROSBag");

    auto* const upperBagToolsButtonLayout = new QHBoxLayout;
    upperBagToolsButtonLayout->addStretch();
    upperBagToolsButtonLayout->addWidget(m_editROSBagButton);
    upperBagToolsButtonLayout->addWidget(m_bagInfoButton);
    upperBagToolsButtonLayout->addStretch();

    auto* const centerBagToolsButtonLayout = new QHBoxLayout;
    centerBagToolsButtonLayout->addStretch();
    centerBagToolsButtonLayout->addWidget(m_bagToVideoPushButton);
    centerBagToolsButtonLayout->addWidget(m_videoToBagPushButton);
    centerBagToolsButtonLayout->addStretch();

    auto* const lowerBagToolsButtonLayout = new QHBoxLayout;
    lowerBagToolsButtonLayout->addStretch();
    lowerBagToolsButtonLayout->addWidget(m_bagToImagesPushButton);
    lowerBagToolsButtonLayout->addWidget(m_dummyBagButton);
    lowerBagToolsButtonLayout->addStretch();

    auto* const bagToolsLayout = new QVBoxLayout;
    bagToolsLayout->addStretch();
    bagToolsLayout->addLayout(upperBagToolsButtonLayout);
    bagToolsLayout->addLayout(centerBagToolsButtonLayout);
    bagToolsLayout->addLayout(lowerBagToolsButtonLayout);
    bagToolsLayout->addStretch();

    auto* const bagToolsWidget = new QWidget;
    bagToolsWidget->setLayout(bagToolsLayout);

    // Publishing tools widget
    m_publishVideoButton = createToolButton("Video as\nROS Messages");

    auto* const publishingToolsLayout = new QHBoxLayout;
    publishingToolsLayout->addStretch();
    publishingToolsLayout->addWidget(m_publishVideoButton);
    publishingToolsLayout->addStretch();

    auto* const publishingToolsWidget = new QWidget;
    publishingToolsWidget->setLayout(publishingToolsLayout);

    setButtonIcons();

    m_backButton = new QPushButton("Back");
    m_backButton->setVisible(false);

    auto* const backButtonLayout = new QHBoxLayout;
    backButtonLayout->addStretch();
    backButtonLayout->addWidget(m_backButton);

    m_versionLabel = new QLabel("v0.6.2");
    m_versionLabel->setToolTip("Bug fixes and stability improvements.");

    auto* const versionLayout = new QHBoxLayout;
    versionLayout->addStretch();
    versionLayout->addWidget(m_versionLabel);

    m_mainLayout = new QVBoxLayout;
    m_mainLayout->addLayout(settingsButtonLayout);
    m_mainLayout->addWidget(headerLabel);
    m_mainLayout->addStretch();
    m_mainLayout->addWidget(overallToolsWidget);
    m_mainLayout->addStretch();
    m_mainLayout->addLayout(versionLayout);
    m_mainLayout->addLayout(backButtonLayout);
    setLayout(m_mainLayout);

    connect(m_settingsButton, &QPushButton::clicked, this, &StartWidget::openSettingsDialog);

    connect(m_bagToolsButton, &QPushButton::clicked, this, [this, overallToolsWidget, bagToolsWidget] {
        m_isBagToolsWidgetSelected = true;
        replaceWidgets(overallToolsWidget, bagToolsWidget, false);
    });
    connect(m_publishingToolsButton, &QPushButton::clicked, this, [this, overallToolsWidget, publishingToolsWidget] {
        m_isBagToolsWidgetSelected = false;
        replaceWidgets(overallToolsWidget, publishingToolsWidget, false);
    });

    connect(m_editROSBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(0);
    });
    connect(m_bagInfoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(1);
    });
    connect(m_bagToVideoPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(2);
    });
    connect(m_videoToBagPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(3);
    });
    connect(m_bagToImagesPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(4);
    });
    connect(m_dummyBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(5);
    });
    connect(m_publishVideoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(6);
    });

    connect(m_backButton, &QPushButton::clicked, this, [this, bagToolsWidget, publishingToolsWidget, overallToolsWidget] {
        replaceWidgets(m_isBagToolsWidgetSelected ? bagToolsWidget : publishingToolsWidget, overallToolsWidget, true);
    });
}


void
StartWidget::openSettingsDialog()
{
    auto* const settingsDialog = new SettingsDialog(m_dialogParameters);
    settingsDialog->exec();
}


void
StartWidget::replaceWidgets(QWidget* fromWidget, QWidget* toWidget, bool otherItemVisibility)
{
    m_settingsButton->setVisible(otherItemVisibility);
    m_backButton->setVisible(!otherItemVisibility);
    m_versionLabel->setVisible(otherItemVisibility);

    m_mainLayout->replaceWidget(fromWidget, toWidget);
    fromWidget->setVisible(false);
    toWidget->setVisible(true);
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
    m_settingsButton->setIcon(QIcon(isDarkMode ? ":/icons/gear_white.svg" : ":/icons/gear_black.svg"));

    m_bagToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_tools_white.svg" : ":/icons/bag_tools_black.svg"));
    m_publishingToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/publishing_tools_white.svg" : ":/icons/publishing_tools_black.svg"));

    m_editROSBagButton->setIcon(QIcon(isDarkMode ? ":/icons/edit_bag_white.svg" : ":/icons/edit_bag_black.svg"));
    m_bagInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_info_white.svg" : ":/icons/bag_info_black.svg"));
    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg"));
    m_bagToImagesPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg"));
    m_dummyBagButton->setIcon(QIcon(isDarkMode ? ":/icons/dummy_bag_white.svg" : ":/icons/dummy_bag_black.svg"));

    m_publishVideoButton->setIcon(QIcon(isDarkMode ? ":/icons/publish_video_white.svg" : ":/icons/publish_video_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
