#include "CameraROS.h"
#include "FindObjectROS.h"

#include <QApplication>
#include <QDir>
#include "find_object/MainWindow.h"
#include "ParametersToolBox.h"
#include "find_object/Settings.h"
#include <signal.h>

using namespace find_object;

bool gui;
std::string settingsPath;
std::string objectId; // 新增用于存储自定义ID

void my_handler_gui(int s) {
    QApplication::closeAllWindows();
    QApplication::quit();
}
void my_handler(int s) {
    QCoreApplication::quit();
}

void setupQuitSignal(bool gui) {
    // Catch ctrl-c to close the gui
    struct sigaction sigIntHandler;
    if (gui) {
        sigIntHandler.sa_handler = my_handler_gui;
    } else {
        sigIntHandler.sa_handler = my_handler;
    }
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_object_2d");

    gui = true;
    std::string objectsPath;
    std::string sessionPath;
    settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
    bool subscribeDepth = false;

    ros::NodeHandle nh("~");

    nh.param("gui", gui, gui);
    nh.param("objects_path", objectsPath, objectsPath);
    nh.param("session_path", sessionPath, sessionPath);
    nh.param("settings_path", settingsPath, settingsPath);
    nh.param("subscribe_depth", subscribeDepth, subscribeDepth);
    nh.param("object_id", objectId, std::string("default_id")); // 读取自定义ID参数

    ROS_INFO("gui=%d", (int)gui);
    ROS_INFO("objects_path=%s", objectsPath.c_str());
    ROS_INFO("session_path=%s", sessionPath.c_str());
    ROS_INFO("settings_path=%s", settingsPath.c_str());
    ROS_INFO("subscribe_depth = %s", subscribeDepth ? "true" : "false");
    ROS_INFO("object_id=%s", objectId.c_str()); // 输出自定义ID

    if (settingsPath.empty()) {
        settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
    } else {
        if (!sessionPath.empty()) {
            ROS_WARN("\"settings_path\" parameter is ignored when \"session_path\" is set.");
        }

        QString path = settingsPath.c_str();
        if (path.contains('~')) {
            path.replace('~', QDir::homePath());
            settingsPath = path.toStdString();
        }
    }

    // Load settings, should be loaded before creating other objects
    Settings::init(settingsPath.c_str());

    FindObjectROS *findObjectROS = new FindObjectROS();
    if (!sessionPath.empty()) {
        if (!objectsPath.empty()) {
            ROS_WARN("\"objects_path\" parameter is ignored when \"session_path\" is set.");
        }
        if (!findObjectROS->loadSession(sessionPath.c_str())) {
            ROS_ERROR("Failed to load session \"%s\"", sessionPath.c_str());
        }
    } else if (!objectsPath.empty()) {
        QString path = objectsPath.c_str();
        if (path.contains('~')) {
            path.replace('~', QDir::homePath());
        }
        if (!findObjectROS->loadObjects(path)) {
            ROS_ERROR("No objects loaded from path \"%s\"", path.toStdString().c_str());
        }
    }

    CameraROS *camera = new CameraROS(subscribeDepth);

    // Catch ctrl-c to close the gui
    setupQuitSignal(gui);

    if (gui) {
        QApplication app(argc, argv);
        MainWindow mainWindow(findObjectROS, camera); // take ownership

        QObject::connect(
            &mainWindow,
            SIGNAL(objectsFound(const find_object::DetectionInfo &, const find_object::Header &, const cv::Mat &, float)),
            findObjectROS,
            SLOT(publish(const find_object::DetectionInfo &, const find_object::Header &, const cv::Mat &, float)));

        QStringList topics = camera->subscribedTopics();
        if (topics.size() == 1) {
            mainWindow.setSourceImageText(mainWindow.tr(
                "<qt>Find-Object subscribed to <b>%1</b> topic.<br/>"
                "You can remap the topic when starting the node: <br/>\"rosrun find_object_2d find_object_2d image:=your/image/topic\".<br/>"
                "</qt>").arg(topics.first()));
        } else if (topics.size() == 3) {
            mainWindow.setSourceImageText(mainWindow.tr(
                "<qt>Find-Object subscribed to : <br/> <b>%1</b> <br/> <b>%2</b> <br/> <b>%3</b><br/>"
                "</qt>").arg(topics.at(0)).arg(topics.at(1)).arg(topics.at(2)));
        }
        mainWindow.show();
        app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

        // loop
        mainWindow.startProcessing();
        app.exec();
        Settings::saveSettings();
    } else {
        QCoreApplication app(argc, argv);

        // connect stuff:
        QObject::connect(camera, SIGNAL(imageReceived(const cv::Mat &, const find_object::Header &, const cv::Mat &, float)),
                         findObjectROS, SLOT(detect(const cv::Mat &, const QString &, double, const cv::Mat &, float)));

        // loop
        camera->start();
        app.exec();

        delete camera;
        delete findObjectROS;
    }
    return 0;
}
