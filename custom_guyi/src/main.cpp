#include <QApplication>
#include "custom_guyi/ros2node.hpp"
#include "custom_guyi/main_gui.hpp"

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto ros2_node = std::make_shared<Ros2Node>();
    auto orin = std::make_shared<OrinDetect>();
    auto gui_app = std::make_shared<MainGUI>(ros2_node);

    app.processEvents();
    gui_app->show();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);
    exec.add_node(orin);

    while (rclcpp::ok())
    {
        exec.spin_some();
        app.processEvents();
    }
    signal(SIGINT, siginthandler);

    exec.remove_node(ros2_node);
    exec.remove_node(orin);
    rclcpp::shutdown();
}
