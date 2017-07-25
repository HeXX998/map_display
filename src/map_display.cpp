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
ros::Publisher goalPub;
bool has_target, has_pose;
double xt, yt, xp, yp, ap;
double pi = 3.1415927;


void draw_arrow(Mat& img, int len, int alpha, Point pStart, double angle, int thickness, int lineType) {
    Point pEnd;
    pEnd.x = pStart.x + len * cos(angle);
    pEnd.y = pStart.y - len * sin(angle);

    printf("pStart: [%d, %d]\n", pStart.x, pStart.y);
    printf("pEnd: [%d, %d]\n", pEnd.x, pEnd.y);

    Point arrow;
    line(img, pStart, pEnd, Scalar(255), thickness, lineType);

    arrow.x = pEnd.x - 10 * cos(angle + pi * alpha / 180);     
    arrow.y = pEnd.y + 10 * sin(angle + pi * alpha / 180);
    line(img, pEnd, arrow, Scalar(255), thickness, lineType);
    printf("arrow1: [%d, %d]\n", arrow.x, arrow.y);
   
    arrow.x = pEnd.x - 10 * cos(angle - pi * alpha / 180);     
    arrow.y = pEnd.y + 10 * sin(angle - pi * alpha / 180);    
    line(img, pEnd, arrow, Scalar(255), thickness, lineType);
    printf("arrow2: [%d, %d]\n", arrow.x, arrow.y);

}

void on_mouse(int Event, int x, int y, int flags, void*) {
    Mat temp = mark_image.clone();
    Point p(x, y);
    
    switch(Event) {
    	case EVENT_LBUTTONDOWN: {
	    if(temp.at<Vec3b>(p)[0] > 250) {
	        printf("%lf %lf\n", (x * 0.5 - 200)*0.05, -(y * 0.5 - 200)*0.05);

                map_display::Pose goal;
                goal.xPose = (x * 0.5 - 200)*0.05;
                goal.yPose = -(y * 0.5 - 200)*0.05;
                goal.angle = 0;

		goalPub.publish(goal);

	        circle(temp, p, 3, Scalar(0, 255), -1, CV_AA);
                has_target = true;
                xt = x;
                yt = y;

                if(has_pose) {
                    draw_arrow(temp, 25, 20, Point(xp, yp), ap, 1, CV_AA);
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

void poseCallback(const map_display::Pose msg) {
    printf("POSE GET\n");    
    double x = (((double)msg.xPose)*20 + 200)*2;
    double y = ((-(double)msg.yPose)*20 + 200)*2;

    printf("[%lf, %lf]\n", x, y);
    has_pose = true;
    xp = x;
    yp = y;
    ap = msg.angle;

    
    Mat temp = mark_image.clone();
//  circle(temp, Point(x,y), 4, Scalar(255), -1, CV_AA);
    draw_arrow(temp, 25, 20, Point(x, y), msg.angle, 1, CV_AA);

    if(has_target) {
        circle(temp, Point(xt, yt), 3, Scalar(0,255), -1, CV_AA);
    }
    show_image = temp.clone();
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "map_display");
    ros::NodeHandle n;

    goalPub = n.advertise<map_display::Pose>("map_display/goal", 1);

    ros::Subscriber markSub = n.subscribe("map_display/mark", 1, markCallback);
    ros::Subscriber poseSub = n.subscribe("map_display/pose", 1, poseCallback);

    has_target = has_pose = false;
    xp = yp = ap = xt = yt = 0;
    

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
