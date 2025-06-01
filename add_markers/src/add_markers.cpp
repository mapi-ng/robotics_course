#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/marker_publisher.hpp"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  std::shared_ptr<ros::NodeHandle> nh =
    std::make_shared<ros::NodeHandle>("");

  MarkerPublisher m_pub(nh);

  geometry_msgs::Point marker_pick_up;
  marker_pick_up.x = 2.0;
  marker_pick_up.y = 1.0;
  geometry_msgs::Point drop_off;
  drop_off.x = -2.0;
  drop_off.y = 1.0;

  enum class States {
    PublishPickUp, DeletePickUp,
    PublishDropOff,
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
          nav_state = States::DeletePickUp;
        }
        break;
      case States::DeletePickUp:
        ROS_INFO_ONCE("Waiting 5 seconds");
        ros::Duration(5.0).sleep();
        ROS_INFO_ONCE("State - DeletePickUp");
        if (m_pub.publishMarker(marker_pick_up, 0, visualization_msgs::Marker::DELETE)) {
          nav_state = States::PublishDropOff;
        }
        break;
      case States::PublishDropOff:
        ROS_INFO_ONCE("Waiting 5 seconds");
        ros::Duration(5.0).sleep();
        ROS_INFO_ONCE("State - PublishDropOff");
        if (m_pub.publishMarker(drop_off, 1, visualization_msgs::Marker::ADD)) {
          nav_state = States::Finished;
        }
        break;
      case States::Finished:
        ROS_INFO_ONCE("Waiting 1 second");
        ros::Duration(1.0).sleep();
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
