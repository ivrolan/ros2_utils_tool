#pragma once

#include <QPointer>
#include <QWidget>

class QHBoxLayout;
class QDialogButtonBox;
class QLabel;
class QPushButton;

/**
 * @brief The basic input widget, which is used to input all sorts of information for the different functions
 */
class BasicInputWidget : public QWidget
{
    Q_OBJECT

public:
    BasicInputWidget(const QString& headerText,
                     const QString& pathLogoDark,
                     const QString& pathLogoLight,
                     QWidget*       parent = 0);

signals:
    void
    back();

    void
    okPressed();

protected:
    void
    enableOkButton(bool enable);

    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event) override;

protected:
    QPointer<QLabel> m_headerLabel;
    QPointer<QLabel> m_headerPixmapLabel;

    QPointer<QPushButton> m_backButton;
    QPointer<QPushButton> m_okButton;

    QPointer<QHBoxLayout> m_buttonLayout;
    QPointer<QDialogButtonBox> m_dialogButtonBox;

    QString m_pathLogoDark;
    QString m_pathLogoLight;
};
