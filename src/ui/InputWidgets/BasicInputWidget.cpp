#include "BasicInputWidget.hpp"

#include "UtilsUI.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>

BasicInputWidget::BasicInputWidget(const QString& headerText, const QString& logoPath, QWidget *parent) :
    QWidget(parent), m_logoPath(logoPath)
{
    m_headerLabel = new QLabel(headerText);
    Utils::UI::setWidgetFontSize(m_headerLabel);
    m_headerLabel->setAlignment(Qt::AlignHCenter);

    m_headerPixmapLabel = new QLabel;
    m_headerPixmapLabel->setAlignment(Qt::AlignHCenter);
    setPixmapLabelIcon();

    m_sourceLineEdit = new QLineEdit;
    m_findSourceButton = new QToolButton;
    m_backButton = new QPushButton("Back", this);

    m_okButton = new QPushButton("Ok", this);
    m_okButton->setEnabled(false);

    m_dialogButtonBox = new QDialogButtonBox;
    m_dialogButtonBox->addButton(m_okButton, QDialogButtonBox::AcceptRole);

    m_findSourceLayout = Utils::UI::createLineEditButtonLayout(m_sourceLineEdit, m_findSourceButton);

    m_buttonLayout = new QHBoxLayout;
    m_buttonLayout->addWidget(m_backButton);
    m_buttonLayout->addStretch();
    m_buttonLayout->addWidget(m_dialogButtonBox);

    connect(m_backButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
}


void
BasicInputWidget::enableOkButton(bool enable)
{
    m_okButton->setEnabled(enable);
}


void
BasicInputWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_logoPath + "_white.svg" : m_logoPath + "_black.svg").pixmap(QSize(100, 45)));
}


bool
BasicInputWidget::event(QEvent *event)
{
    if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
