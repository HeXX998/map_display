#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_display/Mark.h"
#include "map_display/Pose.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/tf.h>

#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

Mat partial_image, show_image, mark_image;
ros::Publisher goal_pub;

void markCallback(const map_display::Mark msg) {
    double x1 = (((double)msg.x1)*20 + 200)*2;
    double x2 = (((double)msg.x2)*20 + 200)*2;
    double y1 = ((-(double)msg.y1)*20 + 200)*2;
    double y2 = ((-(double)msg.y2)*20 + 200)*2;
    printf("(%lf, %lf) (%lf, %lf)\n", x1, y1, x2, y2);

    circle(show_image, Point((x1+x2)*0.5, (y1+y2)*0.5), sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))/2, Scalar(0, 0, 255), -1, CV_AA);
    circle(mark_image, Point((x1+x2)*0.5, (y1+y2)*0.5), sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))/2, Scalar(0, 0, 255), -1, CV_AA);
}

void poseCallback(const nav_msgs::Odometry msg) {
//  double x = (((double)msg.xPose)*20 + 200)*2;
//  double y = (((double)msg.yPose)*20 + 200)*2; 
    printf("POSE GET\n");
    geometry_msgs::PoseWithCovariance pwc = msg.pose;
    geometry_msgs::Pose p = pwc.pose;
    geometry_msgs::Point point = p.position;
    double x = (((double)point.x)*20 + 200)*2;
    double y = ((-(double)point.y)*20 + 200)*2;


    printf("[%lf, %lf]\n", x, y);
    
    Mat temp = mark_image.clone();
    circle(temp, Point(x,y), 5, Scalar(255), -1, CV_AA);
    show_image = temp.clone();
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "robot_trace");
    ros::NodeHandle n;
    ros::Subscriber markSub = n.subscribe("map_display/mark", 1, markCallback);
    ros::Subscriber poseSub = n.subscribe("/RosAria/pose", 1, poseCallback);
    

    if (argc != 4) {
        printf("usage: rosrun map_display robot_trace <Image_Path> <Width> <Height>\n");
        return -1;
    }
    Mat image;
    image = imread(argv[1], 1);
    if (!image.data) {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Map", WINDOW_AUTOSIZE);

    int width = atoi(argv[2]);
    int height = atoi(argv[3]);

    Rect rect(1800, 1800, width, height);
    partial_image = image(rect);

    resize(partial_image, show_image, Size(partial_image.cols*2.0, partial_image.rows*2.0));
    mark_image = show_image.clone();

    while(n.ok()) {
	imshow("Map", show_image);
        waitKey(40);
        ros::spinOnce();
    }

    return 0;
}
