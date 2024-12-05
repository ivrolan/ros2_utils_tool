#pragma once

#include "BasicInputWidget.hpp"
#include "EditBagParamSettings.hpp"
#include "UtilsUI.hpp"

#include <QLineEdit>
#include <QPointer>
#include <QWidget>
#include <QTreeWidgetItem>

class QLabel;
class QTreeWidget;

// The widget showing bag info data
class EditBagWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    EditBagWidget(Utils::UI::EditBagParameters& editBagParameters,
                  QWidget*                      parent = 0);

private slots:
    void
    createTopicTree(bool newTreeRequested);

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

    void
    targetPushButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QTreeWidget> m_treeWidget;
    QPointer<QLabel> m_editLabel;
    QPointer<QLineEdit> m_targetLineEdit;
    QPointer<QWidget> m_targetBagNameWidget;

    Utils::UI::EditBagParameters& m_editBagParameters;

    EditBagParamSettings m_editBagParamSettings;

    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPICS = 1;
    static constexpr int COL_MESSAGE_COUNT = 2;
    static constexpr int COL_RENAMING = 3;
};
