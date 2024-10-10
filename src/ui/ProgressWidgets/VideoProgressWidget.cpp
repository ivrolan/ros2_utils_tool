#include "VideoProgressWidget.hpp"

#include "EncodingThread.hpp"

VideoProgressWidget::VideoProgressWidget(const Utils::UI::VideoParameters& videoParameters, QWidget* parent) :
    BasicProgressWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    m_headerLabel->setText("Encoding Video...");
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg").pixmap(QSize(100, 45)));
    m_thread = new EncodingThread(videoParameters, this);
    connectThread();
}
