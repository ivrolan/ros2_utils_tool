#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QLineEdit;

/**
 * @brief The widget used to manage a video encoding out of a ros bag
 */
class BagToVideoWidget : public BasicConfigWidget
{
    Q_OBJECT

public:
    BagToVideoWidget(Utils::UI::VideoParameters& videoParameters,
                     QString&                    encodingFormat,
                     QWidget*                    parent = 0);

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
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_videoNameLineEdit;
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QCheckBox> m_useHardwareAccCheckBox;

    Utils::UI::VideoParameters& m_videoParameters;
    QString& m_encodingFormat;

    bool m_fileDialogOpened = false;
};
