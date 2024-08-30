#include "BasicConfigWidget.hpp"

#include "UtilsUI.hpp"

#include <QEvent>
#include <QLabel>
#include <QPushButton>

BasicConfigWidget::BasicConfigWidget(const QString& pathLogoDark, const QString& pathLogoLight, QWidget *parent) :
    QWidget(parent), m_pathLogoDark(pathLogoDark), m_pathLogoLight(pathLogoLight)
{
    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    m_okButton = new QPushButton("Ok", this);
    m_okButton->setEnabled(false);
}


void
BasicConfigWidget::enableOkButton(bool enable)
{
    m_okButton->setEnabled(enable);
}


void
BasicConfigWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_pathLogoDark : m_pathLogoLight).pixmap(QSize(100, 45)));
}


bool
BasicConfigWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
