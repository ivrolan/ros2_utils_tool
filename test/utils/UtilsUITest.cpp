#include "catch_ros2/catch_ros2.hpp"

#include "UtilsUI.hpp"

#include <QWidget>

TEST_CASE("Utils UI Testing", "[utils]") {
    SECTION("Font size test") {
        auto* const widget = new QWidget;
        Utils::UI::setWidgetHeaderFont(widget);

        const auto font = widget->font();
        REQUIRE(font.pointSize() == 16);
    }
}
