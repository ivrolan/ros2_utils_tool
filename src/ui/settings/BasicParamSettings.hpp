#pragma once

#include "UtilsSettings.hpp"
#include "UtilsUI.hpp"

#include <QSettings>

template<typename T>
concept SettingsParameter = std::same_as<T, int> || std::same_as<T, bool> || std::same_as<T, QString>;

// Basic settings, from which all other settings derive
class BasicParamSettings {
public:
    BasicParamSettings(Utils::UI::BasicParameters& basicParameters,
                       const QString&              groupName);

    virtual bool
    write();

protected:
    virtual bool
    read();

    template<typename T>
    requires SettingsParameter<T>
    void
    setSettingsParameter(QSettings&     settings,
                         T              parameter,
                         const QString& identifier)
    {
        if (settings.value(identifier).value<T>() == parameter) {
            return;
        }

        settings.setValue(identifier, parameter);
    }

protected:
    const QString m_groupName;

private:
    Utils::UI::BasicParameters& m_basicParameters;
};
