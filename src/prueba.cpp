#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void imgCallback(const sensor_msgs::ImageConsPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc,argv, "opencv_ros");
    ros::NodeHandle node_handle;

    image_transport::ImageTransport imt(node_handle);

    image_transport::Subscriber img_sub = imt.subscribe("/camera", 1, imgCallback);

    ros::spin();

    return 0;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image_input = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch(cv_bridge:Exception &e)
    {
        ROS_ERROR("image wrong");
    }
    

}