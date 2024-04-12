#include "WriteToBagThread.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_video_to_bag path/to/video topic_name path/of/stored/ros_bag use_hardware_acceleration" << std::endl;
    std::cout << "The video must have an ending of .mp4 or .mkv." << std::endl;
    std::cout << "use_hardware_acceleration is either 'true' or 'false'.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() != 5 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }


    // Video directory
    const auto vidDirectory = arguments.at(1);
    auto dirPath = vidDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the video file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }
    const auto fileEnding = vidDirectory.right(3);
    if (fileEnding != "mp4" && fileEnding != "mkv") {
        std::cerr << "The entered video name is not in correct format. Please make sure that the video file ends in mp4 or mkv!" << std::endl;
        return 0;
    }

    // Topic name
    const auto topicName = arguments.at(2);
    if (!UtilsROS::doesTopicNameFollowROS2Convention(topicName)) {
        std::cerr << "The topic name does not follow the ROS2 naming convention!" << std::endl;
        return 0;
    }

    // Handle bag directory
    const auto bagDirectory = arguments.at(3);
    dirPath = bagDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }

    // Hardware acceleration
    const auto useHardwareAccelerationString = arguments.at(4);
    if (useHardwareAccelerationString != "true" && useHardwareAccelerationString != "false") {
        std::cerr << "Please enter either 'true' or 'false' for the hardware acceleration parameter!" << std::endl;
        return 0;
    }
    const auto useHardwareAcceleration = useHardwareAccelerationString == "true";

    auto this_messageCount = 0;

    // Create thread and connect to its informations
    auto* const writeToBagThread = new WriteToBagThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration);
    QObject::connect(writeToBagThread, &WriteToBagThread::calculatedMaximumInstances, [&this_messageCount](int count) {
        this_messageCount = count;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all parameters are set correctly "
            "and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(writeToBagThread, &WriteToBagThread::progressChanged, [&this_messageCount] (int iteration, int progress) {
        const auto progressString = UtilsGeneral::drawProgressString(progress);
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
