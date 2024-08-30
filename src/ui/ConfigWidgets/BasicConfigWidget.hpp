#pragma once

#include <QPointer>
#include <QWidget>

class QLabel;
class QPushButton;

/**
 * @brief The basic config widget, providing a base for all other config widgets
 */
class BasicConfigWidget : public QWidget
{
    Q_OBJECT

public:
    BasicConfigWidget(const QString& pathLogoDark,
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
    QPointer<QLabel> m_headerPixmapLabel;
    QPointer<QPushButton> m_okButton;

    QString m_pathLogoDark;
    QString m_pathLogoLight;
};
