#include "ImagesProgressWidget.hpp"

#include "UtilsUI.hpp"
#include "WriteToImageThread.hpp"

ImagesProgressWidget::ImagesProgressWidget(const Utils::UI::ImageParameters& imageParameters, QWidget* parent) :
    BasicProgressWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    m_headerLabel->setText("Writing Images...");
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg").pixmap(QSize(100, 45)));
    m_thread = new WriteToImageThread(imageParameters, this);
    connectThread();
}
