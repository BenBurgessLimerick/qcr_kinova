#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <qcr_kinova/KinovaGen3.hpp>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "qcr_kinova_driver");
    ros::NodeHandle nh;

    int frequency = 1000;

    KinovaGen3 robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);

    ros::Time last_time = ros::Time::now();

    ros::Rate rate(frequency);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        robot.read();
        cm.update(ros::Time::now(), now - last_time);
        robot.write();

        last_time = now;
        rate.sleep();
    }

    return 0;
}