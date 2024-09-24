#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QTreeWidget;

// The widget showing bag info data
class BagInfoWidget : public BasicConfigWidget
{
    Q_OBJECT
public:
    explicit
    BagInfoWidget(Utils::UI::BasicParameters& bagInfoParameters,
                  QWidget*                    parent = 0);

private slots:
    void
    displayBagInfo();

private:
    QPointer<QTreeWidget> m_infoTreeWidget;

    Utils::UI::BasicParameters& m_bagInfoParameters;

    static constexpr int COL_DESCRIPTION = 0;
    static constexpr int COL_INFORMATION = 1;
};
