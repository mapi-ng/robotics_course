#include "add_markers/marker_publisher.hpp"
#include <visualization_msgs/Marker.h>

MarkerPublisher::MarkerPublisher(std::shared_ptr<ros::NodeHandle> nh) :
  nh_(nh)
{
  marker_pub = nh_->advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

bool MarkerPublisher::publishMarker(const geometry_msgs::Point &point, uint32_t id, uint32_t action)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = id;

  marker.type = visualization_msgs::Marker::CUBE;

  // Marker action. Options are ADD, DELETE, DELETEALL
  marker.action = action;

  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.orientation.w = 1.0;

  // 1x1x1 here means 1m on a side
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  if (marker_pub.getNumSubscribers() < 1)
  {
    ROS_WARN("Please create a subscriber to the marker");
    return false;
  }

  ROS_INFO("Found subscriber!");
  marker_pub.publish(marker);
  return true;
}
