#include "PublishImagesThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_publish_images path/to/video\n" << std::endl;
    std::cout << "The images must have format jpg, png or bmp." << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name. If this is empty, the name '/topic_video' will be taken.\n" << std::endl;
    std::cout << "-r or --rate: Framerate for the published video. Must be from 1 to 60." << std::endl;
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

    Utils::UI::PublishParameters publishParameters;

    // Video directory
    publishParameters.sourceDirectory = arguments.at(1);
    if (!std::filesystem::exists(publishParameters.sourceDirectory.toStdString())) {
        std::cerr << "The images directory does not exist. Please enter a valid images path!" << std::endl;
        return 0;
    }
    auto containsImageFiles = false;
    for (auto const& entry : std::filesystem::directory_iterator(publishParameters.sourceDirectory.toStdString())) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png" || entry.path().extension() == ".bmp") {
            containsImageFiles = true;
            break;
        }
    }
    if (!containsImageFiles) {
        std::cerr << "The specified directory does not contain any images!" << std::endl;
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
            publishParameters.topicName = topicName;
        }

        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", publishParameters.fps, 1, 60)) {
            std::cerr << "Please enter a framerate in the range of 1 to 60!" << std::endl;
            return 0;
        }
        std::cout << publishParameters.fps << std::endl;
        // Switch red and blue values
        publishParameters.switchRedBlueValues = Utils::CLI::containsArguments(arguments, "-s", "--switch");
        // Loop
        publishParameters.loop = Utils::CLI::containsArguments(arguments, "-l", "--loop");
    }

    // Apply default topic name if not assigned
    if (publishParameters.topicName.isEmpty()) {
        publishParameters.topicName = "/topic_video";
    }

    // Create thread and connect to its informations
    auto* const publishImagesThread = new PublishImagesThread(publishParameters);
    auto finished = false;
    QObject::connect(publishImagesThread, &PublishImagesThread::openingCVInstanceFailed, [] {
        std::cerr << "Images publishing failed. Please make sure that the video file is valid and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(publishImagesThread, &PublishImagesThread::progressChanged, [] (const QString& progressString, int /* progress */) {
        std::cout << progressString.toStdString() << "\r" << std::flush;
    });
    QObject::connect(publishImagesThread, &PublishImagesThread::finished, publishImagesThread, &QObject::deleteLater);
    QObject::connect(publishImagesThread, &PublishImagesThread::finished, [&finished] {
        finished = true;
    });

    signal(SIGINT, [] (int /* signal */) {
        interrupted = true;
    });

    std::cout << "Publishing images..." << std::endl;
    Utils::CLI::runThread(publishImagesThread, interrupted, finished);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
