#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/marker_publisher.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

geometry_msgs::Pose robot_pose;
float position_tolerance = 0.3; // 30 cm

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_DEBUG("Received odometry callback.");
  robot_pose = msg->pose.pose;
}

bool arrivedToMarker(const geometry_msgs::Point &marker)
{
  if (fabs(robot_pose.position.x - marker.x) > position_tolerance ||
      fabs(robot_pose.position.y - marker.y) > position_tolerance) {
    ROS_DEBUG("Waiting for the robot to arrive to the marker.");
    return false;
  }
  return true;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_odom_sub");
  std::shared_ptr<ros::NodeHandle> nh =
    std::make_shared<ros::NodeHandle>("");
  ros::Subscriber odom_sub = nh->subscribe("/amcl_pose", 1, odomCallback);

  MarkerPublisher m_pub(nh);

  geometry_msgs::Point marker_pick_up;
  marker_pick_up.x = 0.8;
  marker_pick_up.y = -3.8;
  geometry_msgs::Point drop_off;
  drop_off.x = -8.0;
  drop_off.y = -4.5;

  enum class States {
    PublishPickUp, GoingToPickUp, ReachedPickup,
    PublishDropOff, GoingToDropOff, ReachedDropOff,
    Finished
  };

  States nav_state = States::PublishPickUp;
  bool running = true;

  ros::Rate rate(10);
  while (running && ros::ok())
  {
    switch (nav_state)
    {
      case States::PublishPickUp:
        ROS_INFO_ONCE("State - PublishPickUp");
        if (m_pub.publishMarker(marker_pick_up, 0, visualization_msgs::Marker::ADD)) {
          nav_state = States::GoingToPickUp;
        }
        break;
      case States::GoingToPickUp:
        ROS_INFO_ONCE("State - GoingToPickUp");
        if (arrivedToMarker(marker_pick_up)) {
          nav_state = States::ReachedPickup;
        }
        break;
      case States::ReachedPickup:
        ROS_INFO_ONCE("State - ReachedPickup");
        if (m_pub.publishMarker(marker_pick_up, 0, visualization_msgs::Marker::DELETE)) {
          nav_state = States::GoingToDropOff;
        }
        break;
      case States::GoingToDropOff:
        ROS_INFO_ONCE("Waiting 5 seconds");
        ros::Duration(5.0).sleep();
        ROS_INFO_ONCE("State - GoingToDropOff");
        if (arrivedToMarker(drop_off)) {
          nav_state = States::ReachedDropOff;
        }
        break;
      case States::ReachedDropOff:
        ROS_INFO_ONCE("State - ReachedDropOff");
        if (m_pub.publishMarker(drop_off, 1, visualization_msgs::Marker::ADD)) {
          nav_state = States::Finished;
        }
        break;
      case States::Finished:
        ROS_INFO_ONCE("State - Finished");
        running = false;
        break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Exiting!");
  return 0;
}
