#include "catch_ros2/catch_ros2.hpp"

#include "UtilsCLI.hpp"

TEST_CASE("Utils CLI Testing", "[utils]") {
    QStringList arguments { "argument1", "arg2", "test_3" };

    SECTION("Contains test") {
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == false);

        arguments.append("-t");
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == true);

        arguments.pop_back();
        arguments.append("--test");
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == true);
    }
    SECTION("Argument index test") {
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == -1);

        arguments.append("-t");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 3);

        arguments.pop_back();
        arguments.insert(1, "--test");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 1);

        arguments.removeAt(1);
        arguments.push_front("--test");
        arguments.insert(1, "--t");
        arguments.insert(2, "--test");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 2);
    }
    SECTION("Progress string test") {
        auto progressString = Utils::CLI::drawProgressString(0);
        REQUIRE(progressString == "--------------------------------------------------");
        progressString = Utils::CLI::drawProgressString(10);
        REQUIRE(progressString == "#####---------------------------------------------");
        progressString = Utils::CLI::drawProgressString(25);
        REQUIRE(progressString == "############--------------------------------------");
        progressString = Utils::CLI::drawProgressString(100);
        REQUIRE(progressString == "##################################################");
    }
}
