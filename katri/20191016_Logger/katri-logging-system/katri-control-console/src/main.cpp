#include "command.h"
#include "iostream"
#include "nodeWorker.h"
#include "rclcpp/rclcpp.hpp"
#include "runtime.h"

using namespace std;

/* Signal Handler for SIGINT */
[[noreturn]] void sigintHandler(int sig_num)
{
    cout << "Received signal " << sig_num;
    rclcpp::shutdown();
    fflush(stdout);
    exit(0);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigintHandler);

    // Start a thread for ROS node
    auto node = std::make_shared<NodeWorker>(argc, argv);
    node->start();

    char option;
    bool t = true;
    while (t) {
        cout << "******************************************************" << endl;
        cout << "*         KATRI Control System                       *" << endl;
        cout << "* 0)Start ALL recording         1)Stop ALL recodring *" << endl;
        cout << "* 2)Start Camera 000            3)Stop Camera 000    *" << endl;
        cout << "* 4)Start Camera 001            5)Stop Camera 001    *" << endl;
        cout << "* 6)Start Camera 002            7)Stop Camera 002    *" << endl;
        cout << "* 8)Exit                                             *" << endl;
        cout << "******************************************************" << endl;

        cin >> option;

        while (option < '0' || option > '8') {
            cout << "Please enter a valid number (0-8)" << endl;
            while (getchar() != '\n')
                ;
            cin >> option;
        }

        Runtime runtime;
        uint currSeq = 0;
        QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
        if (runtime.getCurrentDate() == currentDate) {
            currSeq = runtime.getCurrentSequence().toUInt();
        }

        switch (option) {
        case '0': {
            if (runtime.getCurrentDate() == currentDate) {
                currSeq = runtime.getNextSequence().toUInt();
            }

            QStringList cameras = AppConfig::instance().getCameras();
            for (int i = 0; i < cameras.size(); i++) {
                Command cmd(Command::Action::START_NEW_KITTI,
                            Command::Type::CAMERA,
                            cameras.at(i).toStdString(),
                            currSeq);
                node->publish(cmd);
            }

            Command cmd(Command::Action::START_NEW_KITTI, Command::Type::LIDAR, "000", currSeq);
            node->publish(cmd);

            Command cmd2(Command::Action::START_NEW_KITTI, Command::Type::LIDAR, "001", currSeq);
            node->publish(cmd2);

            Command cmd3(Command::Action::START_NEW_KITTI, Command::Type::LIDAR, "002", currSeq);
            node->publish(cmd3);

            // Update current sequence and current date
            runtime.setCurrentSequence(currSeq);
            runtime.setCurrentDate();

            break;
        }
        case '1': {
            QStringList cameras = AppConfig::instance().getCameras();
            for (int i = 0; i < cameras.size(); i++) {
                Command cmd(Command::Action::STOP,
                            Command::Type::CAMERA,
                            cameras.at(i).toStdString(),
                            0);
                node->publish(cmd);
            }
            Command cmd(Command::Action::STOP, Command::Type::LIDAR, "000", currSeq);
            node->publish(cmd);
            Command cmd2(Command::Action::STOP, Command::Type::LIDAR, "001", currSeq);
            node->publish(cmd2);
            Command cmd3(Command::Action::STOP, Command::Type::LIDAR, "002", currSeq);
            node->publish(cmd3);
            break;
        }
        case '2': {
            Command cmd2(Command::Action::START, Command::Type::CAMERA, "000", currSeq);
            node->publish(cmd2);

            // Update current sequence and current date
            runtime.setCurrentSequence(currSeq);
            runtime.setCurrentDate();

            break;
        }
        case '3': {
            Command cmd3(Command::Action::STOP, Command::Type::CAMERA, "000", currSeq);
            node->publish(cmd3);
            break;
        }
        case '4': {
            Command cmd4(Command::Action::START, Command::Type::CAMERA, "001", currSeq);
            node->publish(cmd4);

            // Update current sequence and current date
            runtime.setCurrentSequence(currSeq);
            runtime.setCurrentDate();
            break;
        }
        case '5': {
            Command cmd5(Command::Action::STOP, Command::Type::CAMERA, "001", currSeq);
            node->publish(cmd5);

            break;
        }
        case '6': {
            Command cmd5(Command::Action::START, Command::Type::CAMERA, "002", currSeq);
            node->publish(cmd5);
            // Update current sequence and current date
            runtime.setCurrentSequence(currSeq);
            runtime.setCurrentDate();
            break;
        }
        case '7': {
            Command cmd5(Command::Action::STOP, Command::Type::CAMERA, "002", currSeq);
            node->publish(cmd5);
            break;
        }
        case '8': {
            rclcpp::shutdown();
            exit(0);
        }
        default:
            break;
        }
    }
    return 0;
}
