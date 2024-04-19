#include "catch_ros2/catch_ros2.hpp"

#include "UtilsGeneral.hpp"

TEST_CASE("Utils General Testing", "[utils]") {
    SECTION("Progress string test") {
        auto progressString = Utils::General::drawProgressString(0);
        REQUIRE(progressString == "--------------------------------------------------");
        progressString = Utils::General::drawProgressString(10);
        REQUIRE(progressString == "#####---------------------------------------------");
        progressString = Utils::General::drawProgressString(25);
        REQUIRE(progressString == "############--------------------------------------");
        progressString = Utils::General::drawProgressString(100);
        REQUIRE(progressString == "##################################################");
    }
}
