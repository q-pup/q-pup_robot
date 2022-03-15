#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

ros::Subscriber ar_track_sub;
std_msgs::String state;
ros::Publisher state_pub;
move_base_msgs::MoveBaseGoal goal;
std::unique_ptr<MoveBaseClient> ac;

enum class State { FOLLOW, STOP, UNKNOWN };

void pose_marker_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  State new_state = State::UNKNOWN;
  geometry_msgs::Pose new_goal;

  for (const auto& marker : msg->markers) {
    switch (marker.id) {
      case 1:
        if (new_state == State::UNKNOWN) {
          new_state = State::FOLLOW;
        }
        new_goal = marker.pose.pose;
        break;
      case 2:
        new_state = State::STOP;
        break;
      default:
        // N/A
        break;
    }
  }

  // TODO: tf
  // TODO: Offset pose by 2m in the direction of the base_link

  switch (new_state) {
    case State::FOLLOW:
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position = new_goal.position;
      goal.target_pose.pose.orientation.w = 1.0;
      ac->sendGoal(goal);
      // Don't wait for result lol
      // ac->waitForResult();
      // ac->getState();
      break;
    case State::STOP:
      ac->cancelAllGoals();
      break;
    default:
      // N/A
      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "person_tracker");
  ros::NodeHandle nh;

  // Tell the action client that we want to spin a thread by default
  ac = std::make_unique<MoveBaseClient>("move_base", true);

  // Wait for the action server to come up
  while (!ac->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ar_track_sub = nh.subscribe("/zed/ar_pose_marker", 100, pose_marker_callback);
  state_pub = nh.advertise<std_msgs::String>("state", 100);

  ros::spin();
  return 0;
}
