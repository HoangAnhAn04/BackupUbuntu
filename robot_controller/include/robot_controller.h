#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

class RobotController
{
    private:
        ros::ServiceServer enaber_srv_;
        bool enable_moving_=false;

        ros::NodeHandle nh_;
        nav_msgs::Path path_;
        geometry_msgs::Pose2D robot_pose_;
        
        ros::Subscriber path_sub_,robot_pose_sub_;
        ros::Publisher cmd_vel_pub_;

    public:
        RobotController()
        {
            enaber_srv_=nh_.advertiseService("enable_moving",&RobotController::enableCB,this);

            path_sub_=nh_.subscribe("path", 1, &RobotController::pathCB,this);
            robot_pose_sub_=nh_.subscribe("robot_pose", 1, &RobotController::robotPoseCB,this);
            cmd_vel_pub_= nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

        }

        ~RobotController()
        {
        }
        void pathCB(const nav_msgs::Path& path)
        {
            path_=path;
        }

        void robotPoseCB(const geometry_msgs::Pose2D& rbp)
        {
            robot_pose_=rbp;
            ROS_INFO_STREAM("Robot pose:" << rbp.x << "-" << rbp.y);
         
        }

        bool enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            enable_moving_=req.data;
            if(enable_moving_)
            {
                ROS_INFO_STREAM("Enable moving");
            }else{
                ROS_INFO_STREAM("Disable moving");               
            }
            res.success=true;
            res.message="OK";        
            return true;
        }

        void execute()
        {
            
            geometry_msgs::Twist vel;
            if(enable_moving_)
            {
                vel.linear.x=1.0;
                vel.angular.z=1.0;
        
            }else{
                vel.linear.x=0.0;
                vel.angular.z-=0.0;
            }
            cmd_vel_pub_.publish(vel);
        }
};