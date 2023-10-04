#include <robot_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv,"rb_ctl");
    RobotController rb_controller;
    ros::Rate Loop(20);
    while (ros::ok())
    {
        ros::spinOnce(); //execute callback
        rb_controller.execute();
        Loop.sleep();
    }
    return 0;
}