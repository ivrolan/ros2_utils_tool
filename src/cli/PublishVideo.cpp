#include "PublishVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_publish_video path/to/video\n" << std::endl;
    std::cout << "The video must have an ending of .mp4 or .mkv." << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n" << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-s or --switch: Switch red and blue values." << std::endl;
    std::cout << "-l or --loop: Loop the video.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


bool interrupted = false;

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (arguments.size() < 2 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::PublishVideoParameters publishVideoParameters;

    // Video directory
    publishVideoParameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(publishVideoParameters.sourceDirectory.toStdString())) {
        std::cerr << "The video file does not exist. Please enter a valid video path!" << std::endl;
        return 0;
    }
    const auto fileEnding = publishVideoParameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 2) {
        // Topic name
        if (Utils::CLI::containsArguments(arguments, "-t", "--topic_name")) {
            const auto topicNameIndex = Utils::CLI::getArgumentsIndex(arguments, "-t", "--topic_name");
            if (arguments.at(topicNameIndex) == arguments.last()) {
                std::cerr << "Please enter a valid topic name!" << std::endl;
                return 0;
            }

            const auto& topicName = arguments.at(topicNameIndex + 1);
            if (!Utils::ROS::isNameROS2Conform(topicName)) {
                const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                         "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                         "Do you want to continue anyways? [y/n]";
                if (!Utils::CLI::shouldContinue(errorString)) {
                    return 0;
                }
            }
            publishVideoParameters.topicName = topicName;
        }

        // Hardware acceleration
        publishVideoParameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Switch red and blue values
        publishVideoParameters.switchRedBlueValues = Utils::CLI::containsArguments(arguments, "-s", "--switch");
        // Loop
        publishVideoParameters.loop = Utils::CLI::containsArguments(arguments, "-l", "--loop");
    }

    // Apply default topic name if not assigned
    if (publishVideoParameters.topicName.isEmpty()) {
        publishVideoParameters.topicName = "/topic_video";
    }

    // Create thread and connect to its informations
    auto* const publishVideoThread = new PublishVideoThread(publishVideoParameters);
    auto finished = false;
    QObject::connect(publishVideoThread, &PublishVideoThread::openingCVInstanceFailed, [] {
        std::cerr << "Video publishing failed. Please make sure that the video file is valid and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(publishVideoThread, &PublishVideoThread::progressChanged, [] (const QString& progressString, int /* progress */) {
        std::cout << progressString.toStdString() << "\r" << std::flush;
    });
    QObject::connect(publishVideoThread, &PublishVideoThread::finished, publishVideoThread, &QObject::deleteLater);
    QObject::connect(publishVideoThread, &PublishVideoThread::finished, [&finished] {
        finished = true;
    });

    signal(SIGINT, [] (int /* signal */) {
        interrupted = true;
    });

    std::cout << "Publishing video..." << std::endl;
    Utils::CLI::runThread(publishVideoThread, interrupted, finished);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
