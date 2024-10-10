#include "BagProgressWidget.hpp"

#include "WriteToBagThread.hpp"

BagProgressWidget::BagProgressWidget(const Utils::UI::BagParameters& bagParameters, QWidget* parent) :
    BasicProgressWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    m_headerLabel->setText("Writing to Bag...");
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg").pixmap(QSize(100, 45)));
    m_thread = new WriteToBagThread(bagParameters, this);
    connectThread();
}
