#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QToolButton;

/**
 * @brief The widget used to manage a video encoding out of a ros bag
 */
class BagToVideoWidget : public QWidget
{
    Q_OBJECT

public:
    BagToVideoWidget(const Utils::UI::WidgetParameters& widgetParameters,
                     QWidget*                           parent = 0);

signals:
    void
    back();

    void
    parametersSet(const QString& bagDirectory,
                  const QString& topicName,
                  const QString& vidDirectory,
                  bool           useHardwareAcceleration);

private slots:
    void
    searchButtonPressed();

    void
    videoLocationButtonPressed();

    void
    formatComboBoxTextChanged(const QString& text);

    void
    okButtonPressed();

private:
    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event) override;

private:
    QPointer<QLabel> m_headerPixmapLabel;
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_videoNameLineEdit;
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QCheckBox> m_useHardwareAccCheckBox;

    QPointer<QPushButton> m_okButton;
};
