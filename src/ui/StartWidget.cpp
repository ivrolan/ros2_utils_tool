#include "StartWidget.hpp"

#include "SettingsDialog.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

StartWidget::StartWidget(Utils::UI::DialogParameters& dialogParameters, QWidget *parent) :
    QWidget(parent), m_dialogParameters(dialogParameters)
{
    m_headerLabel = new QLabel("ROS TOOLS");
    Utils::UI::setWidgetFontSize(m_headerLabel);
    m_headerLabel->setAlignment(Qt::AlignHCenter);

    m_settingsButton = new QPushButton;
    m_settingsButton->setFlat(true);

    auto settingsButtonSizePolicy = m_settingsButton->sizePolicy();
    settingsButtonSizePolicy.setRetainSizeWhenHidden(true);
    m_settingsButton->setSizePolicy(settingsButtonSizePolicy);

    auto* const settingsButtonLayout = new QHBoxLayout;
    settingsButtonLayout->addStretch();
    settingsButtonLayout->addWidget(m_settingsButton);

    // Create three widgets: One for providing the overview for bag and publishing tools,
    // one for bag and one for publishing tools only
    // Overview widget
    m_conversionToolsButton = createToolButton("Conversion\nTools");
    m_bagToolsButton = createToolButton("Bag Tools");
    m_publishingToolsButton = createToolButton("Publishing\nTools");

    auto* const overallUpperLayout = new QHBoxLayout;
    overallUpperLayout->addStretch();
    overallUpperLayout->addWidget(m_conversionToolsButton);
    overallUpperLayout->addWidget(m_bagToolsButton);
    overallUpperLayout->addStretch();

    auto* const overallLowerLayout = new QHBoxLayout;
    overallLowerLayout->addStretch();
    overallLowerLayout->addWidget(m_publishingToolsButton);
    overallLowerLayout->addStretch();

    auto* const overallToolsButtonLayout = new QVBoxLayout;
    overallToolsButtonLayout->addLayout(overallUpperLayout);
    overallToolsButtonLayout->addLayout(overallLowerLayout);

    auto* const overallToolsWidget = new QWidget;
    overallToolsWidget->setLayout(overallToolsButtonLayout);

    // Conversion tools widget
    m_bagToVideoPushButton = createToolButton("Encode Video\nfrom Bag");
    m_videoToBagPushButton = createToolButton("Write Video\nto Bag");
    m_bagToImagesPushButton = createToolButton("Write Images\nfrom Bag");

    auto* const conversionToolsUpperLayout = new QHBoxLayout;
    conversionToolsUpperLayout->addStretch();
    conversionToolsUpperLayout->addWidget(m_bagToVideoPushButton);
    conversionToolsUpperLayout->addWidget(m_videoToBagPushButton);
    conversionToolsUpperLayout->addStretch();

    auto* const conversionToolsLowerLayout = new QHBoxLayout;
    conversionToolsLowerLayout->addStretch();
    conversionToolsLowerLayout->addWidget(m_bagToImagesPushButton);
    conversionToolsLowerLayout->addStretch();

    auto* const conversionToolsLayout = new QVBoxLayout;
    conversionToolsLayout->addStretch();
    conversionToolsLayout->addLayout(conversionToolsUpperLayout);
    conversionToolsLayout->addLayout(conversionToolsLowerLayout);
    conversionToolsLayout->addStretch();

    auto* const conversionToolsWidget = new QWidget;
    conversionToolsWidget->setLayout(conversionToolsLayout);

    // Bag tools widget
    m_editBagButton = createToolButton("Edit Bag");
    m_mergeBagsButton = createToolButton("Merge Bags");
    m_dummyBagButton = createToolButton("Create\nDummy Bag");
    m_bagInfoButton = createToolButton("Get Infos\nfrom Bag");

    auto* const bagToolsUpperLayout = new QHBoxLayout;
    bagToolsUpperLayout->addStretch();
    bagToolsUpperLayout->addWidget(m_editBagButton);
    bagToolsUpperLayout->addWidget(m_mergeBagsButton);
    bagToolsUpperLayout->addStretch();

    auto* const bagToolsLowerLayout = new QHBoxLayout;
    bagToolsLowerLayout->addStretch();
    bagToolsLowerLayout->addWidget(m_dummyBagButton);
    bagToolsLowerLayout->addWidget(m_bagInfoButton);
    bagToolsLowerLayout->addStretch();

    auto* const bagToolsLayout = new QVBoxLayout;
    bagToolsLayout->addStretch();
    bagToolsLayout->addLayout(bagToolsUpperLayout);
    bagToolsLayout->addLayout(bagToolsLowerLayout);
    bagToolsLayout->addStretch();

    auto* const bagToolsWidget = new QWidget;
    bagToolsWidget->setLayout(bagToolsLayout);

    // Publishing tools widget
    m_publishVideoButton = createToolButton("Publish Video\nas ROS Topic");
    m_publishImagesButton = createToolButton("Publish Images\nas ROS Topic");

    auto* const publishingToolsLayout = new QHBoxLayout;
    publishingToolsLayout->addStretch();
    publishingToolsLayout->addWidget(m_publishVideoButton);
    publishingToolsLayout->addWidget(m_publishImagesButton);
    publishingToolsLayout->addStretch();

    auto* const publishingToolsWidget = new QWidget;
    publishingToolsWidget->setLayout(publishingToolsLayout);

    setButtonIcons();

    m_backButton = new QPushButton("Back");
    m_backButton->setVisible(false);

    auto* const backButtonLayout = new QHBoxLayout;
    backButtonLayout->addWidget(m_backButton);
    backButtonLayout->addStretch();

    m_versionLabel = new QLabel("v0.7.2");
    m_versionLabel->setToolTip("Bug fixes and stability improvements.");

    auto* const versionLayout = new QHBoxLayout;
    versionLayout->addStretch();
    versionLayout->addWidget(m_versionLabel);

    m_mainLayout = new QVBoxLayout;
    m_mainLayout->addLayout(settingsButtonLayout);
    m_mainLayout->addWidget(m_headerLabel);
    m_mainLayout->addStretch();
    m_mainLayout->addWidget(overallToolsWidget);
    m_mainLayout->addStretch();
    m_mainLayout->addLayout(versionLayout);
    m_mainLayout->addLayout(backButtonLayout);
    setLayout(m_mainLayout);

    const auto switchToOverallTools = [this, conversionToolsWidget, bagToolsWidget, publishingToolsWidget, overallToolsWidget] {
        switch (m_widgetOnInstantiation) {
        case 1:
            replaceWidgets(conversionToolsWidget, overallToolsWidget, 0, true);
            break;
        case 2:
            replaceWidgets(bagToolsWidget, overallToolsWidget, 0, true);
            break;
        case 3:
            replaceWidgets(publishingToolsWidget, overallToolsWidget, 0, true);
            break;
        default:
            break;
        }
    };
    const auto switchToConversionTools = [this, overallToolsWidget, conversionToolsWidget] {
        replaceWidgets(overallToolsWidget, conversionToolsWidget, 1, false);
    };
    const auto switchToBagTools = [this, overallToolsWidget, bagToolsWidget] {
        replaceWidgets(overallToolsWidget, bagToolsWidget, 2, false);
    };
    const auto switchToPublishingTools = [this, overallToolsWidget, publishingToolsWidget] {
        replaceWidgets(overallToolsWidget, publishingToolsWidget, 3, false);
    };

    connect(m_settingsButton, &QPushButton::clicked, this, &StartWidget::openSettingsDialog);

    connect(m_backButton, &QPushButton::clicked, this, switchToOverallTools);
    connect(m_conversionToolsButton, &QPushButton::clicked, this, switchToConversionTools);
    connect(m_bagToolsButton, &QPushButton::clicked, this, switchToBagTools);
    connect(m_publishingToolsButton, &QPushButton::clicked, this, switchToPublishingTools);

    connect(m_bagToVideoPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(0);
    });
    connect(m_videoToBagPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(1);
    });
    connect(m_bagToImagesPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(2);
    });
    connect(m_editBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(3);
    });
    connect(m_mergeBagsButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(4);
    });
    connect(m_dummyBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(5);
    });
    connect(m_bagInfoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(6);
    });
    connect(m_publishVideoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(7);
    });
    connect(m_publishImagesButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(8);
    });

    switch (m_widgetOnInstantiation) {
    case 1:
        switchToConversionTools();
        break;
    case 2:
        switchToBagTools();
        break;
    case 3:
        switchToPublishingTools();
        break;
    default:
        break;
    }
}


void
StartWidget::openSettingsDialog()
{
    auto* const settingsDialog = new SettingsDialog(m_dialogParameters);
    settingsDialog->exec();
}


// Used to switch between the four overall widgets
void
StartWidget::replaceWidgets(QWidget* fromWidget, QWidget* toWidget, int widgetIdentifier, bool otherItemVisibility)
{
    // If the back button is visible, the other elements should be hidden and vice versa
    m_backButton->setVisible(!otherItemVisibility);
    m_settingsButton->setVisible(otherItemVisibility);
    m_versionLabel->setVisible(otherItemVisibility);
    m_widgetOnInstantiation = widgetIdentifier;

    switch (m_widgetOnInstantiation) {
    case 0:
        m_headerLabel->setText("ROS TOOLS");
        break;
    case 1:
        m_headerLabel->setText("CONVERSION TOOLS");
        break;
    case 2:
        m_headerLabel->setText("BAG TOOLS");
        break;
    case 3:
        m_headerLabel->setText("PUBLISHING TOOLS");
        break;
    default:
        break;
    }

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

    m_conversionToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/conversion_tools_white.svg" : ":/icons/conversion_tools_black.svg"));
    m_bagToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_tools_white.svg" : ":/icons/bag_tools_black.svg"));
    m_publishingToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/publishing_tools_white.svg" : ":/icons/publishing_tools_black.svg"));

    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg"));
    m_bagToImagesPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg"));

    m_editBagButton->setIcon(QIcon(isDarkMode ? ":/icons/edit_bag_white.svg" : ":/icons/edit_bag_black.svg"));
    m_mergeBagsButton->setIcon(QIcon(isDarkMode ? ":/icons/merge_bags_white.svg" : ":/icons/merge_bags_black.svg"));
    m_dummyBagButton->setIcon(QIcon(isDarkMode ? ":/icons/dummy_bag_white.svg" : ":/icons/dummy_bag_black.svg"));
    m_bagInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_info_white.svg" : ":/icons/bag_info_black.svg"));

    m_publishVideoButton->setIcon(QIcon(isDarkMode ? ":/icons/publish_video_white.svg" : ":/icons/publish_video_black.svg"));
    m_publishImagesButton->setIcon(QIcon(isDarkMode ? ":/icons/publish_images_white.svg" : ":/icons/publish_images_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
