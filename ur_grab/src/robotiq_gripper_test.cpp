
#include "robotiq_gripper_test/gripperControl.h"
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotiq_gripper_test");
    ros::NodeHandle n;
    ros::Rate rate(30);

    ros::Publisher gripperPub = n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 20);
    gripperPubPtr = &gripperPub;
    initializeGripperMsg();

    while(ros::ok())
    {
        sendGripperMsg(0);//open
        sendGripperMsg(255);//close
        rate.sleep();
    }

    return 0;
}
