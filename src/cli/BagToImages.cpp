#include "WriteToImageThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/ROSBag topic_name path/to/target/image/dir --format format" << std::endl;
    std::cout << "-f or --format: Must be jpg or png.\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-q 0-9 or --quality 0-9: Image quality, must be between 0 and 9 (9 is highest)." << std::endl;
    std::cout << "-c or --colorless: Encode images without color." << std::endl;
    std::cout << "-o or --optimize (jpg only): Optimize jpeg file size." << std::endl;
    std::cout << "-b or --binary (png only): Write images with only black and white pixels." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto& arguments = app.arguments();
    if (arguments.size() < 6 || arguments.contains("--help") || arguments.contains("-h") || (!arguments.contains("-f") && !arguments.contains("--format"))) {
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

    // Images directory
    const auto imagesDirectory = arguments.at(3);

    // Format
    const auto formatString = arguments.at(5);
    if (formatString != "jpg" && formatString != "png") {
        std::cerr << "Please enter either 'jpg' or 'png' for the format!" << std::endl;
        return 0;
    }

    auto quality = 8;
    auto isColorless = false;
    auto optimize = false;
    auto writeBinary = false;

    // Check for optional arguments
    if (arguments.size() > 6) {
        if (Utils::CLI::containsArguments(arguments, "-q", "--quality")) {
            if (arguments.size() < 8) {
                std::cerr << "Please specify a value for the quality!" << std::endl;
                return 0;
            }

            const auto qualityIndex = std::max(arguments.indexOf("-q"), arguments.indexOf("--quality"));
            if (quality = arguments.at(qualityIndex + 1).toInt(); quality < 0 || quality > 9) {
                std::cerr << "Please enter a number between 0 and 9 for the quality value!" << std::endl;
                return 0;
            }
        }

        isColorless = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        optimize = formatString == "jpg" && Utils::CLI::containsArguments(arguments, "-o", "--optimize");
        writeBinary = formatString == "png" && Utils::CLI::containsArguments(arguments, "-b", "--binary");
    }

    Utils::UI::ImageParameters imageParameters { { bagDirectory, topicName }, imagesDirectory, formatString, quality, false, isColorless, optimize, writeBinary };
    auto thisMessageCount = 0;

    // Create thread and connect to its informations
    auto* const writeToImageThread = new WriteToImageThread(imageParameters);
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
