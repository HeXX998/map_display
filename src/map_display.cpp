#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_display/Pose.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/tf.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

Mat partial_image, show_image;
Rect rect;
ros::Publisher goal_pub;

void on_mouse(int Event, int x, int y, int flags, void*) {
    Mat temp = partial_image.clone();
    Mat show;
    resize(partial_image, show, Size(partial_image.cols*2.0, partial_image.rows*2.0));
    Point p(x, y);
    
    switch(Event) {
    	case EVENT_LBUTTONDOWN: {
	    if(show.at<Vec3b>(p)[0] > 250) {
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

	        circle(show, p, 2, Scalar(255), 3);
		show_image = show.clone();
	    }
	}
    }
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "map_display");
    ros::NodeHandle n;
    goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
    

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

    setMouseCallback("Map", on_mouse, &show_image);
    while(n.ok()) {
        imshow("Map", show_image);
        waitKey(40);
    }

    return 0;
}
