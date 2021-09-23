#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

#include <ros/ros.h>
#include "geometry_msgs/Point.h"

class MarkerPublisher
{
  private:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher marker_pub;

  public:
    MarkerPublisher(std::shared_ptr<ros::NodeHandle> nh);
    ~MarkerPublisher() = default;
    MarkerPublisher() = delete;
    MarkerPublisher(const MarkerPublisher&) = delete;
    MarkerPublisher(MarkerPublisher &&) = delete;

    bool publishMarker(const geometry_msgs::Point &point, uint32_t id, uint32_t action);
};

#endif