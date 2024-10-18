#pragma once

#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QComboBox;
class QLineEdit;

/**
 * @brief The widget used to manage a video encoding out of a ros bag
 */
class BagToVideoWidget : public BasicInputWidget
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

    Utils::UI::VideoParameters& m_videoParameters;

    bool m_fileDialogOpened = false;
};
