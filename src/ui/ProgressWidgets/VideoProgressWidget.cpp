#include "VideoProgressWidget.hpp"

#include "EncodingThread.hpp"
#include "WriteToBagThread.hpp"

VideoProgressWidget::VideoProgressWidget(const Utils::UI::VideoParameters& videoParameters, bool isEncoding, QWidget* parent) :
    BasicProgressWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    m_headerLabel->setText(isEncoding ? "Encoding Video..." : "Writing to Bag...");
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg").pixmap(QSize(100, 45)));

    if (isEncoding) {
        m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg").pixmap(QSize(100, 45)));
        m_thread = new EncodingThread(videoParameters, this);
    } else {
        m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg").pixmap(QSize(100, 45)));
        m_thread = new WriteToBagThread(videoParameters, this);
    }
    connectThread();
}
