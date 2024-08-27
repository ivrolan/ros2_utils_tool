#include "ImagesProgressWidget.hpp"

#include "UtilsUI.hpp"
#include "WriteToImageThread.hpp"

ImagesProgressWidget::ImagesProgressWidget(const QString& bagDirectory, const QString& topicName, const QString& vidDirectory,
                                           const QString& format, int compressionLevel, QWidget* parent) :
    BasicProgressWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg").pixmap(QSize(100, 45)));
    m_thread = new WriteToImageThread(bagDirectory, topicName, vidDirectory, format, compressionLevel, this);
    connectThread();
}
