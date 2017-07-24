#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_display/Pose.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/tf.h>

#include <cmath>
#include <stdio.h>

ros::Publisher posePub;

void poseCallback(const nav_msgs::Odometry msg) {
    printf("POSE GET\n");
    geometry_msgs::PoseWithCovariance pwc = msg.pose;
    geometry_msgs::Pose p = pwc.pose;
    geometry_msgs::Point point = p.position;
    double x = (double)point.x;
    double y = (double)point.y;

    geometry_msgs::Quaternion quat = p.orientation;
    double w = 0;
    
    if(quat.z >= 0) {
        w = 2 * acos(quat.w);
    }
    else {
        w = 2 * (3.1415926 - acos(quat.w));
    }

    printf("[%lf, %lf, %lf]\n", x, y, w);

    map_display::Pose message;
    message.xPose = x;
    message.yPose = y;
    message.angle = w;  
    posePub.publish(message);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_display");
    ros::NodeHandle n;
    posePub = n.advertise<map_display::Pose>("map_display/pose", 1);
    ros::Subscriber poseSub = n.subscribe("/RosAria/pose", 1, poseCallback);

    while(n.ok()) {
        ros::spin();
    }
    
    return 0;
}


