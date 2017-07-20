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
Rect rect;
ros::Publisher goal_pub;
bool has_target, has_pose;
double xt, yt, xp, yp;

void on_mouse(int Event, int x, int y, int flags, void*) {
    Mat temp = mark_image.clone();
    Point p(x, y);
    
    switch(Event) {
    	case EVENT_LBUTTONDOWN: {
	    if(temp.at<Vec3b>(p)[0] > 250) {
	        printf("%lf %lf\n", (x * 0.5 - 200)*0.05, -(y * 0.5 - 200)*0.05);

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = (x * 0.5 - 200)*0.05;
		goal.target_pose.pose.position.y = -(y * 0.5 - 200)*0.05;
		geometry_msgs::Quaternion qua_dir;
		qua_dir = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
		goal.target_pose.pose.orientation = qua_dir;
		move_base_msgs::MoveBaseActionGoal actionGoal;
		actionGoal.goal_id.id = "goal";
		actionGoal.goal = goal;
		goal_pub.publish(actionGoal);

	        circle(temp, p, 3, Scalar(0, 255), -1, CV_AA);
                has_target = true;
                xt = x;
                yt = y;
                if(has_pose) {
                    circle(temp, Point(xp, yp), 5, Scalar(255), -1, CV_AA);
                }
		show_image = temp.clone();
	    }
	}
    }
}

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
    printf("POSE GET\n");
    geometry_msgs::PoseWithCovariance pwc = msg.pose;
    geometry_msgs::Pose p = pwc.pose;
    geometry_msgs::Point point = p.position;
    double x = (((double)point.x)*20 + 200)*2;
    double y = ((-(double)point.y)*20 + 200)*2;

    printf("[%lf, %lf]\n", x, y);
    
    Mat temp = mark_image.clone();
    circle(temp, Point(x,y), 5, Scalar(255), -1, CV_AA);
    has_pose = true;
    xp = x;
    yp = y;
    if(has_target) {
        circle(temp, Point(xt, yt), 3, Scalar(0,255), -1, CV_AA);
    }
    show_image = temp.clone();
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "map_display");
    ros::NodeHandle n;
    goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
    ros::Subscriber markSub = n.subscribe("map_display/mark", 1, markCallback);
    ros::Subscriber poseSub = n.subscribe("/RosAria/pose", 1, poseCallback);

    has_target = false;
    has_pose = false;
    xp = 0;
    yp = 0;
    xt = 0;
    yt = 0;
    

    if (argc != 4) {
        printf("usage: ./DisplayImage <Image_Path> <Width> <Height>\n");
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

    rect = Rect(1800, 1800, width, height);
    partial_image = image(rect);

    resize(partial_image, show_image, Size(partial_image.cols*2.0, partial_image.rows*2.0));
    mark_image = show_image.clone();

    setMouseCallback("Map", on_mouse, &show_image);
    while(n.ok()) {
        imshow("Map", show_image);
        waitKey(40);
        ros::spinOnce();
    }

    return 0;
}
