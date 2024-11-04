#include "catch_ros2/catch_ros2.hpp"

#include "UtilsSettings.hpp"

#include <QSettings>

TEST_CASE("Utils Settings Testing", "[utils]") {
    QSettings settings;

    SECTION("Unsaved test") {
        settings.clear();
        const auto parametersSaved = Utils::Settings::readAreParametersSaved();

        REQUIRE(parametersSaved == false);
        REQUIRE(!settings.value("save").isValid());
    }
    SECTION("Saving test") {
        settings.clear();
        settings.setValue("save", true);
        const auto parametersSaved = Utils::Settings::readAreParametersSaved();

        REQUIRE(parametersSaved == true);
        REQUIRE(settings.value("save").isValid());
    }
}
