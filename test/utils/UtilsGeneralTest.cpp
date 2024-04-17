#include "catch_ros2/catch_ros2.hpp"

#include "UtilsGeneral.hpp"

TEST_CASE("Utils General Testing", "[utils]") {
    SECTION("Progress string test") {
        auto progressString = UtilsGeneral::drawProgressString(0);
        REQUIRE(progressString == "--------------------------------------------------");
        progressString = UtilsGeneral::drawProgressString(10);
        REQUIRE(progressString == "#####---------------------------------------------");
        progressString = UtilsGeneral::drawProgressString(25);
        REQUIRE(progressString == "############--------------------------------------");
        progressString = UtilsGeneral::drawProgressString(100);
        REQUIRE(progressString == "##################################################");
    }
}
