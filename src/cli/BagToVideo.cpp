#include "EncodingThread.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_video path/to/ROSBag topic_name path/of/stored/video use_hardware_acceleration" << std::endl;
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

    // Handle bag directory
    const auto bagDirectory = arguments.at(1);
    if (!std::filesystem::exists(bagDirectory.toStdString())) {
        std::cerr << "Bag file not found. Make sure that the bag file exists!" << std::endl;
        return 0;
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(bagDirectory.toStdString()); !doesDirContainBag) {
        std::cerr << "The directory does not contain a bag file!" << std::endl;
        return 0;
    }

    // Topic name
    const auto topicName = arguments.at(2);
    if (!Utils::ROS::doesBagContainTopicName(bagDirectory.toStdString(), topicName.toStdString())) {
        std::cerr << "Topic has not been found in the bag file!" << std::endl;
        return 0;
    }
    if (Utils::ROS::getTopicType(bagDirectory.toStdString(), topicName.toStdString()) != "sensor_msgs/msg/Image") {
        std::cerr << "The entered topic is not in sensor message format!" << std::endl;
        return 0;
    }

    // Video directory
    const auto vidDirectory = arguments.at(3);
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

    // Hardware acceleration
    const auto useHardwareAccelerationString = arguments.at(4);
    if (useHardwareAccelerationString != "true" && useHardwareAccelerationString != "false") {
        std::cerr << "Please enter either 'true' or 'false' for the hardware acceleration parameter!" << std::endl;
        return 0;
    }
    const auto useHardwareAcceleration = useHardwareAccelerationString == "true";

    auto this_messageCount = 0;

    // Create encoding thread and connect to its informations
    auto* const encodingThread = new EncodingThread(bagDirectory, topicName, vidDirectory, useHardwareAcceleration);
    QObject::connect(encodingThread, &EncodingThread::calculatedMaximumInstances, [&this_messageCount](int count) {
        this_messageCount = count;
    });
    QObject::connect(encodingThread, &EncodingThread::openingCVInstanceFailed, [] {
        std::cerr << "The video writing failed. Please make sure that all parameters are set correctly and disable the hardware acceleration, if necessary." << std::endl;
        return 0;
    });
    QObject::connect(encodingThread, &EncodingThread::progressChanged, [&this_messageCount] (int iteration, int progress) {
        const auto progressString = Utils::General::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling in the terminal
        std::cout << progressString << " " << progress << "% (Frame " << iteration << " of " << this_messageCount << ")\r";
    });
    QObject::connect(encodingThread, &EncodingThread::finished, [] {
        std::cout << "Encoding finished! \r" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(encodingThread, &EncodingThread::finished, encodingThread, &QObject::deleteLater);

    std::cout << "Encoding video. Please wait..." << std::endl;
    encodingThread->start();
    // Wait until the thread is finished
    while (!encodingThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
