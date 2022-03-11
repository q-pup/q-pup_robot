#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>


ros::Subscriber sub;
ros::Publisher pub;
ros::NodeHandle nh;

void pose_marker_callback(const ar_track_alvar_msgs::AlvarMarker& msg_recv)
{
    ROS_INFO("Entered Person Tracker Callback");
    std_msgs::String msg;
    std::stringstream ss;

    if (msg_recv.id == 1)
    {
        ss << "follow";
        msg.data = ss.str();
        pub.publish(msg);

        ros::Publisher coord_pub = nh.advertise<move_base_msgs::MoveBaseAction>("move_base/goal", 1000);
        move_base_msgs::MoveBaseAction goal = move_base_msgs::MoveBaseAction();
        goal.action_goal.goal.target_pose = msg_recv.pose;
        coord_pub.publish(goal);
    }
    else if (msg_recv.id == 2)
    {
        ss << "stop";
        msg.data = ss.str();
        pub.publish(msg);
    }
    
}

int main(int argc, char **argv)
{
    ROS_INFO("Entered Person Tracker");
    ros::init(argc, argv, "person_tracker");

    sub  = nh.subscribe("/zed/ar_pose_marker", 1000, pose_marker_callback);
    pub = nh.advertise<std_msgs::String>("person_tracker", 1000);

    ros::spin();
    return 0;
}