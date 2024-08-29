#include "WriteToImageThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/ROSBag topic_name path/to/target/image/dir format quality" << std::endl;
    std::cout << "The format must be jpg or png." << std::endl;
    std::cout << "The quality must be between 0 and 9 (9 is highest).\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() != 6 || arguments.contains("--help") || arguments.contains("-h")) {
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
    const auto imagesDirectory = arguments.at(3);
    if (!std::filesystem::is_empty(imagesDirectory.toStdString())) {
        std::cerr << "The target directory is not empty. Please make sure that the directory is empty!" << std::endl;
        return 0;
    }

    // Format
    const auto formatString = arguments.at(4);
    if (formatString != "jpg" && formatString != "png") {
        std::cerr << "Please enter either 'jpg' or 'png' for the format!" << std::endl;
        return 0;
    }

    // Quality
    const auto qualityString = arguments.at(5);
    if (qualityString.toInt() < 0 && qualityString.toInt() > 9) {
        std::cerr << "Please enter a number between 0 and 9 for the quality value!" << std::endl;
        return 0;
    }

    auto thisMessageCount = 0;

    // Create thread and connect to its informations
    auto* const writeToImageThread = new WriteToImageThread(bagDirectory, topicName, imagesDirectory, formatString, qualityString.toInt());
    QObject::connect(writeToImageThread, &WriteToImageThread::calculatedMaximumInstances, [&thisMessageCount](int count) {
        thisMessageCount = count;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::progressChanged, [&thisMessageCount] (int iteration, int progress) {
        const auto progressString = Utils::General::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling in the terminal
        std::cout << progressString << " " << progress << "% (Frame " << iteration << " of " << thisMessageCount << ")\r";
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, [] {
        std::cout << "Writing images finished! \r" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(writeToImageThread, &WriteToImageThread::finished, writeToImageThread, &QObject::deleteLater);

    std::cout << "Writing images. Please wait..." << std::endl;
    writeToImageThread->start();
    // Wait until the thread is finished
    while (!writeToImageThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
