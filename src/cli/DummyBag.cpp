#include "DummyBagThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_dummy_bag path/to/ROSBag topic_type_1 topic_name_1 "
        "topic_type_2 topic_name_2 topic_type_3 topic_name_3 message_count" << std::endl;
    std::cout << "Topic type is either 'String', 'Integer' or 'Image'." << std::endl;
    std::cout << "You can write up to three topics." << std::endl;
    std::cout << "The message count must be between 1 and 1000.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() < 4 || arguments.size() > 9 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::DummyBagParameters dummyBagParameters;
    dummyBagParameters.topicName = "";

    // Bag directory
    dummyBagParameters.sourceDirectory = arguments.at(1);
    auto dirPath = dummyBagParameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the bag file does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }

    // Message count
    dummyBagParameters.messageCount = arguments.at(arguments.size() - 1).toInt();
    if (dummyBagParameters.messageCount < 1 || dummyBagParameters.messageCount > 1000) {
        std::cerr << "Please enter a number between 1 and 1000 for the message count value!" << std::endl;
        return 0;
    }

    // Topics
    QVector<QString> topicTypes;
    QVector<QString> topicNames;
    QSet<QString> topicNameSet;

    for (auto i = 2; i < arguments.size() - 1; i++) {
        const auto argument = arguments.at(i);

        if (i % 2 == 0) {
            if (argument != "String" && argument != "Integer" && argument != "Image") {
                std::cerr << "The topic type must be either 'String', 'Integer' or 'Image'!" << std::endl;
                return 0;
            }
            topicTypes.push_back(argument);
        } else {
            if (!Utils::ROS::doesTopicNameFollowROS2Convention(argument)) {
                std::cerr << "The topic name does not follow the ROS2 naming convention!" << std::endl;
                return 0;
            }
            topicNames.push_back(argument);
            topicNameSet.insert(argument);
        }
    }

    if (topicTypes.size() != topicNames.size()) {
        std::cerr << "Topic type and topic name size do not match. Please make sure to enter a name for each topic!" << std::endl;
        return 0;
    }
    if (topicNameSet.size() != topicNames.size()) {
        std::cerr << "Duplicate topic names detected. Please make sure that every topic name is unique!" << std::endl;
        return 0;
    }

    // Create thread parameters
    QVector<Utils::UI::DummyBagTopic> topics;
    for (auto i = 0; i < topicTypes.size(); i++) {
        dummyBagParameters.topics.push_back({ topicTypes.at(i), topicNames.at(i) });
    }

    // Create thread and connect to its informations
    auto* const dummyBagThread = new DummyBagThread(dummyBagParameters);

    auto thisMessageCount = 0;

    QObject::connect(dummyBagThread, &DummyBagThread::calculatedMaximumInstances, [&thisMessageCount](int count) {
        thisMessageCount = count;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::progressChanged, [&thisMessageCount] (int iteration, int progress) {
        const auto progressString = Utils::CLI::drawProgressString(progress);
        // Clear the last line for a nice "progress bar" feeling
        std::cout << progressString << " " << progress << "% (Message " << iteration << " of " << thisMessageCount << ")\r" << std::flush;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Creating bag finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(dummyBagThread, &DummyBagThread::finished, dummyBagThread, &QObject::deleteLater);

    std::cout << "Creating dummy thread. Please wait..." << std::endl;
    dummyBagThread->start();
    // Wait until the thread is finished
    while (!dummyBagThread->isFinished()) {
    }

    return EXIT_SUCCESS;
}
