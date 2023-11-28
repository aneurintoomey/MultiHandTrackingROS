#include <ros/ros.h>
#include <joint_tracking/MediapipeTracker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>

#include <vector>
#include <fstream>
#include <string>

void ImageCallback(const sensor_msgs::ImageConstPtr &image){
    ros::NodeHandle nh("~");
    ros::ServiceClient client = nh.serviceClient<joint_tracking::MediapipeTracker>("/MediapipeTracker");

    joint_tracking::MediapipeTracker srv;
    srv.request.image = *image;

    if(client.call(srv)){
        ROS_INFO("Mediapipe Service Called");
        cv::imwrite("frame.jpg", cv_bridge::toCvShare(sensor_msgs::ImageConstPtr(new sensor_msgs::Image(srv.response.newImage)), 
            sensor_msgs::image_encodings::RGB8)->image);
    } else {
        ROS_ERROR("Failed to Call Service");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_demo");
    ros::NodeHandle nh("~");
    ros::Subscriber image_sub = nh.subscribe("/kinect2/qhd/image_color", 1, &ImageCallback);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(nh.ok()){
        try {
        cv::imshow("Live Image", cv::imread("frame.jpg"));
        cv::waitKey(1);
        } catch (){
            ROS_ERROR("Failed to display image")
        }
    }

    cv::destroyAllWindows();

    return 0;
}