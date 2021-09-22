#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

void publishMarker(const ros::Publisher &marker_pub, float x, float y, uint32_t id, uint32_t action)
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

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1.0;

  // 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      ROS_ERROR("ROS is not OK!");
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  ROS_INFO("Found subscriber!");
  marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  publishMarker(marker_pub, 2.0, 1.0, 0, visualization_msgs::Marker::ADD);
  ros::Duration(5.0).sleep();
  publishMarker(marker_pub, 2.0, 1.0, 0, visualization_msgs::Marker::DELETE);
  ros::Duration(5.0).sleep();
  publishMarker(marker_pub, -2.0, 1.0, 1, visualization_msgs::Marker::ADD);
  ros::Duration(1.0).sleep();

  ROS_INFO("Finished!");
  return 0;
}
