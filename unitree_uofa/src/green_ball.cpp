#include <ros/ros.h>
#include "geometry_msgs/Point.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "green_ball");
    ros::NodeHandle nh;

    // Create a publisher to publish the ball points
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("ball_coordinates", 10);

    int do_the_thing = 0;

    // Do some camera setup here

    // Publish ball points at a rate of 10 Hz
    ros::Rate rate(10);

    while(ros::ok()) {
        
        // Check rosparam current value to see if Point message is required 
        nh.getParam("/do_the_thing", do_the_thing);
        
        if(do_the_thing != 0) {
            // Do some camera shiz here

            // Define the point message and fill it with the values of x and y
            geometry_msgs::Point point;
            point.x = 1.0;  
            point.y = 2.0;

            // Publish the point message
            point_pub.publish(point);
            std::cout << "I'm doing the thing..."<< std::endl;

        } else {
            std::cout << "I'm not doing the thing..."<< std::endl;
        }       

        rate.sleep();
    }
    return 0;
}
