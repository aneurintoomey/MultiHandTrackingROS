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
    out_msg.image = cv::imread("/workspaces/MultiHandTrackingROS/640px-Human-Hands-Front-Back.jpg");
    out_msg.encoding = "bgr8";

    srv.request.image = *out_msg.toImageMsg();

    while(ros::ok()){
        if(client.call(srv)){
            ROS_INFO("Service Succesfully Called");
            ROS_INFO("Right: [%s]", srv.response.right.type.c_str());
            ROS_INFO("Right: [%d]", srv.response.right.u.size());
        } else {
            ROS_ERROR("Failed to Call Service");
        }

        unsigned int microsecond = 1000000;
        usleep(3 * microsecond);//sleeps for 3 second
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "joint_tracking");
    HandTracking ht;
    ros::spin();

    return 0;
}