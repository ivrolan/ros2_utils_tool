#include "catch_ros2/catch_ros2.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <filesystem>

TEST_CASE("Utils ROS Testing", "[utils]") {
    const auto bagDirectory = std::filesystem::path("test_bag_file");
    std::filesystem::remove_all(bagDirectory);

    rosbag2_cpp::Writer writer;
    writer.open(bagDirectory);

    for (auto i = 0; i < 5; i++) {
        sensor_msgs::msg::Image imageMessage;
        imageMessage.width = 1;
        imageMessage.height = 1;
        writer.write(imageMessage, "/topic_image", rclcpp::Clock().now());
    }
    for (auto i = 0; i < 3; i++) {
        std_msgs::msg::String stringMessage;
        stringMessage.data = "example string";
        writer.write(stringMessage, "/topic_string", rclcpp::Clock().now());
    }
    writer.close();

    SECTION("Contains topic name test") {
        auto contains = UtilsROS::doesBagContainTopicName(bagDirectory, "/topic_image");
        REQUIRE(contains == true);
        contains = UtilsROS::doesBagContainTopicName(bagDirectory, "/topic_string");
        REQUIRE(contains == true);
        contains = UtilsROS::doesBagContainTopicName(bagDirectory, "/topic_should_not_be_included");
        REQUIRE(contains == false);
    }
    SECTION("Topic message count test") {
        auto messageCount = UtilsROS::getTopicMessageCount(bagDirectory, "/topic_image");
        REQUIRE(messageCount == 5);
        messageCount = UtilsROS::getTopicMessageCount(bagDirectory, "/topic_string");
        REQUIRE(messageCount == 3);
        messageCount = UtilsROS::getTopicMessageCount(bagDirectory, "/topic_should_not_be_included");
        REQUIRE(messageCount == 0);
    }
    SECTION("Topic type test") {
        auto topicType = UtilsROS::getTopicType(bagDirectory, "/topic_image");
        REQUIRE(topicType == "sensor_msgs/msg/Image");
        topicType = UtilsROS::getTopicType(bagDirectory, "/topic_string");
        REQUIRE(topicType == "std_msgs/msg/String");
        topicType = UtilsROS::getTopicType(bagDirectory, "/topic_should_not_be_included");
        REQUIRE(topicType == "");
    }
    SECTION("Video topics test") {
        const auto videoTopics = UtilsROS::getBagVideoTopics(bagDirectory);
        REQUIRE(videoTopics.size() == 1);
        REQUIRE(videoTopics.at(0) == "/topic_image");
    }
    SECTION("Name ROS2 convention tests") {
        SECTION("Fails for special characters") {
            const auto followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}");
            REQUIRE(followsConvention == false);
        }
        SECTION("Slash and underbrackets") {
            auto followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test_topic/");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test__topic");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("//test_topic");
            REQUIRE(followsConvention == false);
        }
        SECTION("First char number") {
            auto followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("0test_topic");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test_topic");
            REQUIRE(followsConvention == true);
        }
        SECTION("Tilde and slash") {
            auto followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test~topic");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test~/topic");
            REQUIRE(followsConvention == true);
        }
        SECTION("Balanced curly braces") {
            auto followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test{topic}");
            REQUIRE(followsConvention == true);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test_{topic");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test_topic}");
            REQUIRE(followsConvention == false);
            followsConvention = UtilsROS::doesTopicNameFollowROS2Convention("test_{t{opic}");
            REQUIRE(followsConvention == false);
        }
    }

    std::filesystem::remove_all(bagDirectory);
}
