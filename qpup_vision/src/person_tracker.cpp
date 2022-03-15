#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

ros::Subscriber ar_track_sub;
std_msgs::String state;
ros::Publisher state_pub;
move_base_msgs::MoveBaseGoal goal;
std::unique_ptr<MoveBaseClient> ac;

tf2_ros::Buffer buffer_;

enum class State { UNKNOWN , FOLLOW, STOP };

void pose_marker_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  State new_state = State::UNKNOWN;
  geometry_msgs::PoseStamped new_goal;

  for (const auto& marker : msg->markers) {
    switch (marker.id) {
      case 1:
        if (new_state == State::UNKNOWN) {
          new_state = State::FOLLOW;
        }
        new_goal = buffer_.transform(marker.pose, "base_link");
        break;
      case 2:
        new_state = State::STOP;
        break;
      default:
        // N/A
        break;
    }
  }

  // Offset pose by 2m in the direction of the base_link
  const auto mag = std::sqrt(new_goal.pose.position.x * new_goal.pose.position.x +
                             new_goal.pose.position.y * new_goal.pose.position.y);
  new_goal.pose.position.x *= (mag - 2) / mag;
  new_goal.pose.position.y *= (mag - 2) / mag;

  // Orientation based on heading to target
  const Eigen::Vector3d cur_heading(1,0,0);
  const Eigen::Vector3d to_goal(new_goal.pose.position.x, new_goal.pose.position.y, new_goal.pose.position.z);
  const auto quat = Eigen::Quaternion<double>::FromTwoVectors(cur_heading, to_goal);
  new_goal.pose.orientation.x = quat.x();
  new_goal.pose.orientation.y = quat.y();
  new_goal.pose.orientation.z = quat.z();
  new_goal.pose.orientation.w = quat.w();

  switch (new_state) {
    case State::FOLLOW:
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position = new_goal.pose.position;
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

  // Start a TF2 listener
  tf2_ros::TransformListener listener(buffer_);

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
