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

  m_pub.publishMarker(marker_pick_up, 0, visualization_msgs::Marker::ADD);
  ros::Duration(5.0).sleep();
  m_pub.publishMarker(marker_pick_up, 0, visualization_msgs::Marker::DELETE);
  ros::Duration(5.0).sleep();
  m_pub.publishMarker(drop_off, 1, visualization_msgs::Marker::ADD);
  ros::Duration(1.0).sleep();

  ROS_INFO("Finished!");
  return 0;
}
