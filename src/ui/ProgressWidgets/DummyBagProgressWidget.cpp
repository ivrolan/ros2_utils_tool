#include "DummyBagProgressWidget.hpp"

#include "DummyBagThread.hpp"

DummyBagProgressWidget::DummyBagProgressWidget(const Utils::UI::DummyBagParameters& dummyBagParameters, QWidget* parent) :
    BasicProgressWidget(parent)
{
    m_headerLabel->setText("Creating ROSBag...");

    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/dummy_bag_white.svg" : ":/icons/dummy_bag_black.svg").pixmap(QSize(100, 45)));

    m_thread = new DummyBagThread(dummyBagParameters, this);
    connectThread();
}
