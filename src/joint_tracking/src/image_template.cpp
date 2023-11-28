#include <ros/ros.h>
#include <joint_tracking/MediapipeTracker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>

#include <vector>
#include <fstream>
#include <string>

class HandTracking {
    public:
    HandTracking();

    private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
};

HandTracking::HandTracking(): nh("~"){
    client = nh.serviceClient<joint_tracking::MediapipeTracker>("/MediapipeTracker");

    //Testing code follows
    joint_tracking::MediapipeTracker srv;
    cv_bridge::CvImage out_msg;
    out_msg.image = cv::imread("/workspaces/multi_hand_tracking_ws/man-hand-with-bent-isolated-on-white-background.jpg");
    out_msg.encoding = "bgr8";

    srv.request.image = *out_msg.toImageMsg();

    while(ros::ok()){
        if(client.call(srv)){
            ROS_INFO("Service Succesfully Called");
            ROS_INFO("Right Hand is a [%s] hand", srv.response.right.type.c_str());
            cv::imshow("Live Image", cv_bridge::toCvShare(sensor_msgs::ImageConstPtr(new sensor_msgs::Image(srv.response.newImage)), sensor_msgs::image_encodings::RGB8)->image);
        } else {
            ROS_ERROR("Failed to Call Service");
        }

        unsigned int microsecond = 1000000;
        usleep(microsecond);//sleeps for 1 second
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_template");
    HandTracking ht;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    return 0;
}