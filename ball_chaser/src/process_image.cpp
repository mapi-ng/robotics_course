#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

class ColorTracker
{
private:
    ros::ServiceClient client_;
    ros::Publisher pub_;
    float area_threshold_low_;
    float area_threshold_high_;
    float angular_gain_;
    float linear_gain_;

public:
    ColorTracker(ros::NodeHandle &nh)
    {
        area_threshold_low_ = nh.param("area_threshold_low", 5.0e+3);
        area_threshold_high_ = nh.param("area_threshold_high", 2.5e+6);
        angular_gain_ = nh.param("angular_gain", 0.5);
        linear_gain_ = nh.param("linear_gain", 0.3);

        client_ = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        pub_ = nh.advertise<geometry_msgs::Point>("normalized_point", 2);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &img)
    {
        try
        {
            cv::Mat src = cv_bridge::toCvCopy(img)->image;
            cv::Vec2i image_point = colorTracking(src);
            if (image_point[0] != -1 && image_point[1] != -1)
            {
                cv::Vec2i image_size(src.cols, src.rows);
                cv::Vec2f normalized_point = normalizePoint(image_point, image_size);
                ROS_DEBUG_STREAM("Normalized point X = " << normalized_point[0]);
                driveRobot(linear_gain_, -angular_gain_*normalized_point[0]);
                publishFeedback(normalized_point[0], normalized_point[1]);
            }
            else
            {
                driveRobot(0.0, 0.0);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
        }
    }

    // Color tracking method based on:
    // https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
    cv::Vec2i colorTracking(cv::Mat &src)
    {
        cv::Vec2i position(-1, -1);
        cv::Mat hsv, transformed;

        int hue_low_threshold = 0;
        int hue_high_threshold = 0;
        int sat_low_threshold = 0;
        int sat_high_threshold = 0;
        int val_low_threshold = 253;
        int val_high_threshold = 255;

        if (src.channels() > 1)
            cv::cvtColor(src, hsv, cv::COLOR_RGB2HSV);
        else
            return position;

        cv::inRange(hsv, cv::Scalar(hue_low_threshold, sat_low_threshold, val_low_threshold),
            cv::Scalar(hue_high_threshold, sat_high_threshold, val_high_threshold),
            transformed);

        cv::erode(transformed, transformed, cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(5, 5)) );
        cv::dilate(transformed, transformed, cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(5, 5)) );

        cv::dilate(transformed, transformed, cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(5, 5)) );
        cv::erode(transformed, transformed, cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(5, 5)) );

        cv::Moments img_moments = moments(transformed);

        if (img_moments.m00 > area_threshold_low_ &&
            img_moments.m00 < area_threshold_high_) {
            position[0] = img_moments.m10 / img_moments.m00;
            position[1] = img_moments.m01 / img_moments.m00;
        }
        
        return position;
    }

    cv::Vec2f normalizePoint(cv::Vec2i &img_point, cv::Vec2i &img_size)
    {
        ROS_DEBUG_STREAM("Image point X = " << img_point[0]);
        ROS_DEBUG_STREAM("Width = " << img_size[0]);
        ROS_DEBUG_STREAM("Height = " << img_size[1]);
        float width = static_cast<float>(img_size[0]);
        float height = static_cast<float>(img_size[1]);

        return cv::Vec2f(
            2*(static_cast<float>(img_point[0])/width - 0.5),
            2*(static_cast<float>(img_point[1])/height - 0.5)
        );
    }
        
    void driveRobot(float lin_x, float ang_z)
    {
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;
        if (!client_.call(srv))
            ROS_ERROR("Failed to call service safe_move");
    }

    void publishFeedback(float x, float y)
    {
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0;

        pub_.publish(point);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ColorTracker color_tracker(n);

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 2,
        &ColorTracker::imageCallback, &color_tracker);

    ros::spin();

    return 0;
}
