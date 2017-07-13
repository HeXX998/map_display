#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_display/Pose.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

Mat partial_image;
Rect rect;
ros::Publisher chatter_pub;

void on_mouse(int Event, int x, int y, int flags, void*) {
    Mat temp = partial_image.clone();
    Point p(x, y);
    
    switch(Event) {
    	case EVENT_LBUTTONDOWN: {
	    if(temp.at<Vec3b>(p)[0] > 250) {
	        printf("%lf %lf\n", (x-200)*0.05, (y-200)*0.05);

		map_display::Pose msg;
    		msg.xPose = (x-200)*0.05;
    		msg.yPose = (y-200)*0.05;

                chatter_pub.publish(msg);
	        circle(temp, p, 2, Scalar(255), 3);

		imshow("Map", temp);
                waitKey(0);
	    }
	}
    }
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "map_display");
    ros::NodeHandle n;
    chatter_pub = n.advertise<map_display::Pose>("map/Pose", 1000);

    map_display::Pose msg;
    msg.xPose = 0;
    msg.yPose = 0;

    chatter_pub.publish(msg);
    

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

    setMouseCallback("Map", on_mouse, &partial_image);

    imshow("Map", partial_image);
    waitKey(0);

    return 0;
}
