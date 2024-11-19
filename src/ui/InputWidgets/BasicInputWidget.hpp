#pragma once

#include "BasicParamSettings.hpp"

#include <QPointer>
#include <QWidget>

class QDialogButtonBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QToolButton;

template<typename T, typename U>
concept WriteSettingsParameter = (std::same_as<T, U> && (std::same_as<T, QString> || std::same_as<T, int> || std::same_as<T, bool>));

/**
 * @brief The basic input widget, which is used to input all sorts of information for the different functions
 */
class BasicInputWidget : public QWidget
{
    Q_OBJECT

public:
    BasicInputWidget(const QString& headerText,
                     const QString& logoPath,
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

    template<typename T, typename U>
    requires WriteSettingsParameter<T, U>
    void
    writeSettingsParameter(T&                  settingsParameter,
                           const U&            newValue,
                           BasicParamSettings& basicParamSettings)
    {
        settingsParameter = newValue;
        basicParamSettings.write();
    }

protected:
    QPointer<QLabel> m_headerLabel;
    QPointer<QLabel> m_headerPixmapLabel;

    QPointer<QLineEdit> m_sourceLineEdit;

    QPointer<QToolButton> m_findSourceButton;
    QPointer<QPushButton> m_backButton;
    QPointer<QPushButton> m_okButton;

    QPointer<QHBoxLayout> m_findSourceLayout;
    QPointer<QHBoxLayout> m_buttonLayout;
    QPointer<QDialogButtonBox> m_dialogButtonBox;

    QString m_logoPath;
};
