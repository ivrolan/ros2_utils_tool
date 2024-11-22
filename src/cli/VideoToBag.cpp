#include "WriteToBagThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_video_to_bag path/to/video topic_name path/of/stored/ros_bag" << std::endl;
    std::cout << "The video must have an ending of .mp4 or .mkv.\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-r or --rate: Framerate for the image stream. Must be from 10 to 60. If no rate is specified, the video's rate will be taken." << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() < 4 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::BagParameters bagParameters;

    // Video directory
    bagParameters.sourceDirectory = arguments.at(1);
    auto dirPath = bagParameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    const auto fileEnding = bagParameters.sourceDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Topic name
    bagParameters.topicName = arguments.at(2);
    if (!Utils::ROS::doesTopicNameFollowROS2Convention(bagParameters.topicName)) {
        std::cerr << "The topic name does not follow the ROS2 naming convention!" << std::endl;
        return 0;
    }

    // Handle bag directory
    bagParameters.targetDirectory = arguments.at(3);
    dirPath = bagParameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 4) {
        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", bagParameters.fps, 10, 60)) {
            std::cerr << "Please enter a framerate in the range of 10 to 60!" << std::endl;
            return 0;
        }

        // Hardware acceleration
        bagParameters.useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
    }

    auto this_messageCount = 0;

    // Create thread and connect to its informations
    auto* const writeToBagThread = new WriteToBagThread(bagParameters);

    QObject::connect(writeToBagThread, &WriteToBagThread::calculatedMaximumInstances, [&this_messageCount](int count) {
        this_messageCount = count;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all parameters are set correctly "
            "and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::progressChanged, [&this_messageCount] (int iteration, int progress) {
        const auto progressString = Utils::General::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling in the terminal
        std::cout << progressString << " " << progress << "% (Frame " << iteration << " of " << this_messageCount << ")\r";
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::finished, [] {
        std::cout << "Writing finished! \r" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::finished, writeToBagThread, &QObject::deleteLater);

    std::cout << "Writing video to bag... Please wait..." << std::endl;
    writeToBagThread->start();
    // Wait until the thread is finished
    while (!writeToBagThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
