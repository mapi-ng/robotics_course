#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <atomic>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class ObjectPicker
{
  private:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber marker_sub_;
    MoveBaseClient ac_;
    std::atomic_bool goal_in_progress_;

  public:
    ObjectPicker(std::shared_ptr<ros::NodeHandle> nh) :
      nh_(nh),
      ac_(*nh, "move_base", true),
      goal_in_progress_(false)
    {
      // Wait 5 sec for move_base action server to come up
      while(!ac_.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      marker_sub_ = nh_->subscribe("visualization_marker", 1,
                                   &ObjectPicker::markerCallback, this);
    }
    
    ~ObjectPicker() = default;
    ObjectPicker() = delete;
    ObjectPicker(const ObjectPicker&) = delete;
    ObjectPicker(ObjectPicker &&) = delete;

    void markerCallback(const visualization_msgs::Marker::ConstPtr& msg)
    {
      ROS_DEBUG("Received marker callback.");
      ROS_INFO("Sending goal");
      ac_.sendGoal(createGoal(msg->pose.position.x, msg->pose.position.y));
      goal_in_progress_ = true;
    }

    bool waitForResultForever()
    {
      if (goal_in_progress_) {
        ROS_INFO("Going to wait to reach the goal...");
        ac_.waitForResult();

        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO_STREAM("Hooray, the base moved to the goa!");
          goal_in_progress_ = false;
          ROS_INFO("Waiting 5 secons");
          ros::Duration(5.0).sleep();
          return true;
        } else {
          ROS_INFO("The base failed to move to the goal...");
          goal_in_progress_ = false;
          return false;
        }
      }
    }

    move_base_msgs::MoveBaseGoal createGoal(float x, float y)
    {
      move_base_msgs::MoveBaseGoal goal;

      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = x;
      goal.target_pose.pose.position.y = y;
      goal.target_pose.pose.orientation.w = 1.0;

      return goal;
    }
};

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects_marker_sub");
  std::shared_ptr<ros::NodeHandle> nh =
    std::make_shared<ros::NodeHandle>("");

  ObjectPicker object_picker(nh);

  bool running = true;
  while (running && ros::ok()) {
    object_picker.waitForResultForever();
    ros::spinOnce();
  }

  ROS_INFO("Exiting!");
  return 0;
}