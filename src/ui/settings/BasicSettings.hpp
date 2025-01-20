#pragma once

#include <QSettings>

template<typename T>
concept SettingsParameter = std::same_as<T, int> || std::same_as<T, size_t> ||
                            std::same_as<T, bool> || std::same_as<T, QString>;

// Basic settings, from which all other settings derive
class BasicSettings {
public:
    BasicSettings(const QString& groupName) : m_groupName(groupName)
    {
    }

    virtual bool
    write() = 0;

protected:
    virtual bool
    read() = 0;

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
        // Simple conversion between size_t and QVariant is not possible
        if constexpr (std::is_same_v<T, size_t>) {
            QVariant v;
            v.setValue(parameter);
            settings.setValue(identifier, v);
        } else {
            settings.setValue(identifier, parameter);
        }
    }

protected:
    const QString m_groupName;
};
