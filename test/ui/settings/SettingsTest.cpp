#include "catch_ros2/catch_ros2.hpp"

#include "AdvancedParamSettings.hpp"
#include "BagParamSettings.hpp"
#include "BasicParamSettings.hpp"
#include "DummyBagParamSettings.hpp"
#include "EditBagParamSettings.hpp"
#include "ImageParamSettings.hpp"
#include "UtilsUI.hpp"
#include "VideoParamSettings.hpp"

#include <QSettings>

TEST_CASE("Settings Testing", "[ui]") {
    QSettings settings;
    settings.setValue("save", true);

    SECTION("Concept Test") {
        REQUIRE(SettingsParameter<int>);
        REQUIRE(SettingsParameter<bool>);
        REQUIRE(SettingsParameter<QString>);

        REQUIRE(!SettingsParameter<float>);
        REQUIRE(!SettingsParameter<double>);
        REQUIRE(!SettingsParameter<char>);
        REQUIRE(!SettingsParameter<std::string>);
    }
    SECTION("Basic Params Test") {
        SECTION("Read") {
            settings.beginGroup("basic");
            REQUIRE(!settings.value("source_dir").isValid());
            REQUIRE(!settings.value("topic_name").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::BasicParameters basicParameters;
            BasicParamSettings basicParamSettings(basicParameters, "basic");

            basicParameters.sourceDirectory = "/source/dir";
            basicParameters.topicName = "/test_topic_name";
            basicParamSettings.write();

            settings.beginGroup("basic");
            REQUIRE(settings.value("source_dir").isValid());
            REQUIRE(settings.value("topic_name").isValid());
            REQUIRE(settings.value("source_dir").toString() == "/source/dir");
            REQUIRE(settings.value("topic_name").toString() == "/test_topic_name");
            settings.endGroup();
        }
    }
    SECTION("Advanced Params Test") {
        SECTION("Read") {
            settings.beginGroup("advanced");
            REQUIRE(!settings.value("target_dir").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::AdvancedParameters advancedParameters;
            AdvancedParamSettings advancedParamSettings(advancedParameters, "advanced");

            advancedParameters.targetDirectory = "/target/dir";
            advancedParamSettings.write();

            settings.beginGroup("advanced");
            REQUIRE(settings.value("target_dir").isValid());
            REQUIRE(settings.value("target_dir").toString() == "/target/dir");
            settings.endGroup();
        }
    }
    SECTION("Image Params Test") {
        SECTION("Read") {
            settings.beginGroup("images");
            REQUIRE(!settings.value("format").isValid());
            REQUIRE(!settings.value("quality").isValid());
            REQUIRE(!settings.value("bw_images").isValid());
            REQUIRE(!settings.value("jpg_optimize").isValid());
            REQUIRE(!settings.value("png_bilevel").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::ImageParameters imageParameters;
            ImageParamSettings imageParamSettings(imageParameters, "images");

            imageParameters.format = "jpg";
            imageParameters.quality = 10;
            imageParameters.useBWImages = true;
            imageParameters.jpgOptimize = true;
            imageParameters.pngBilevel = true;
            imageParamSettings.write();

            settings.beginGroup("images");
            REQUIRE(settings.value("format").isValid());
            REQUIRE(settings.value("format").toString() == "jpg");
            REQUIRE(settings.value("quality").isValid());
            REQUIRE(settings.value("quality").toInt() == 10);
            REQUIRE(settings.value("bw_images").isValid());
            REQUIRE(settings.value("bw_images").toBool() == true);
            REQUIRE(settings.value("jpg_optimize").isValid());
            REQUIRE(settings.value("jpg_optimize").toBool() == true);
            REQUIRE(settings.value("png_bilevel").isValid());
            REQUIRE(settings.value("png_bilevel").toBool() == true);
            settings.endGroup();
        }
    }
    SECTION("Video Params Test") {
        SECTION("Read") {
            settings.beginGroup("video");
            REQUIRE(!settings.value("format").isValid());
            REQUIRE(!settings.value("fps").isValid());
            REQUIRE(!settings.value("hw_acc").isValid());
            REQUIRE(!settings.value("bw_images").isValid());
            REQUIRE(!settings.value("lossless_images").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::VideoParameters videoParameters;
            VideoParamSettings videoParamSettings(videoParameters, "video");

            videoParameters.format = "mkv";
            videoParameters.fps = 20;
            videoParameters.useHardwareAcceleration = true;
            videoParameters.useBWImages = true;
            videoParameters.lossless = true;
            videoParamSettings.write();

            settings.beginGroup("video");
            REQUIRE(settings.value("format").isValid());
            REQUIRE(settings.value("format").toString() == "mkv");
            REQUIRE(settings.value("fps").isValid());
            REQUIRE(settings.value("fps").toInt() == 20);
            REQUIRE(settings.value("hw_acc").isValid());
            REQUIRE(settings.value("hw_acc").toBool() == true);
            REQUIRE(settings.value("bw_images").isValid());
            REQUIRE(settings.value("bw_images").toBool() == true);
            REQUIRE(settings.value("lossless_images").isValid());
            REQUIRE(settings.value("lossless_images").toBool() == true);
            settings.endGroup();
        }
    }
    SECTION("Bag Params Test") {
        SECTION("Read") {
            settings.beginGroup("bag");
            REQUIRE(!settings.value("cdr").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::BagParameters bagParameters;
            BagParamSettings bagParamSettings(bagParameters, "bag");

            bagParameters.fps = 40;
            bagParameters.useCustomFPS = true;
            bagParameters.useHardwareAcceleration = true;
            bagParamSettings.write();

            settings.beginGroup("bag");
            REQUIRE(settings.value("fps").isValid());
            REQUIRE(settings.value("fps").toInt() == 40);
            REQUIRE(settings.value("custom_fps").isValid());
            REQUIRE(settings.value("custom_fps").toBool() == true);
            REQUIRE(settings.value("hw_acc").isValid());
            REQUIRE(settings.value("hw_acc").toBool() == true);
            settings.endGroup();
        }
    }
    SECTION("Dummmy Bag Params Test") {
        SECTION("Read") {
            settings.beginGroup("dummy");
            REQUIRE(!settings.value("topics").isValid());
            REQUIRE(!settings.value("msg_count").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::DummyBagParameters dummyBagParameters;
            DummyBagParamSettings dummyBagParamSettings(dummyBagParameters, "dummy");

            dummyBagParameters.messageCount = 250;
            dummyBagParameters.topics.push_back({ "string", "example_name" });
            dummyBagParamSettings.write();

            settings.beginGroup("dummy");
            REQUIRE(settings.value("msg_count").isValid());
            REQUIRE(settings.value("msg_count").toInt() == 250);

            const auto size = settings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                settings.setArrayIndex(i);
                REQUIRE(settings.value("type").isValid());
                REQUIRE(settings.value("type").toString() == "string");
                REQUIRE(settings.value("name").isValid());
                REQUIRE(settings.value("name").toString() == "example_name");
            }
            REQUIRE(size == 1);
            settings.endArray();

            settings.endGroup();
        }
    }
    SECTION("Edit Bag Params Test") {
        SECTION("Read") {
            settings.beginGroup("dummy");
            REQUIRE(!settings.value("topics").isValid());
            settings.endGroup();
        }
        SECTION("Write") {
            Utils::UI::EditBagParameters editBagParameters;
            EditBagParamSettings editBagParamSettings(editBagParameters, "edit");

            editBagParameters.topics.push_back({ "renamed_topic", "original_topic", 42, 1337, true });
            editBagParamSettings.write();

            settings.beginGroup("edit");
            const auto size = settings.beginReadArray("topics");
            for (auto i = 0; i < size; ++i) {
                settings.setArrayIndex(i);
                REQUIRE(settings.value("renamed_name").isValid());
                REQUIRE(settings.value("renamed_name").toString() == "renamed_topic");
                REQUIRE(settings.value("original_name").isValid());
                REQUIRE(settings.value("original_name").toString() == "original_topic");
                REQUIRE(settings.value("lower_boundary").isValid());
                REQUIRE(settings.value("lower_boundary").toInt() == 42);
                REQUIRE(settings.value("upper_boundary").isValid());
                REQUIRE(settings.value("upper_boundary").toInt() == 1337);
                REQUIRE(settings.value("is_selected").isValid());
                REQUIRE(settings.value("is_selected").toBool() == true);
            }
            REQUIRE(size == 1);
            settings.endArray();

            settings.endGroup();
        }
    }

    settings.clear();
}
