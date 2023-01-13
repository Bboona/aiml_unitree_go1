#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include "geometry_msgs/Point.h"


namespace movement_namespace {
    typedef struct {
        uint8_t head[2];
        uint8_t levelFlag;
        uint8_t frameReserve;
        uint32_t SN[2];
        uint32_t version[2];
        uint16_t bandWidth[4];
        int16_t footForce[4];
        int16_t footForceEst[4];
        uint8_t mode;
        float_t progress;
        uint8_t gaitType;	   
        float_t footRaiseHeight;		  
        float_t position[3];
        float_t bodyHeight;			  
        float_t velocity[3]; 
        float_t yawSpeed;				   
        float_t rangeObstacle[4];
        uint8_t wirelessRemote[40];
    } high_state_struct;

    class Movement {
        private:
            // Node handles
            ros::NodeHandle _nh;

            // Private variables
            bool _running = false;
            high_state_struct _high_state_variables;

            // Subscribers
            ros::Subscriber high_state_subscriber;
            ros::Subscriber ball_coordinates_subscriber;
        public:
            Movement(): _nh() {
                high_state_subscriber = _nh.subscribe("/high_state", 1, &Movement::highStateHandler, this); 
                high_state_subscriber = _nh.subscribe("/ball_coordinates", 1, &Movement::ballCoordinatesHandler, this); 
            };

        // callback function for the subscriber
        void highStateHandler(const unitree_legged_msgs::HighState::ConstPtr &high_state_msg) {
            memcpy(&_high_state_variables.mode, &high_state_msg->mode, sizeof(_high_state_variables.mode));
            memcpy(&_high_state_variables.progress, &high_state_msg->progress, sizeof(_high_state_variables.progress));
            memcpy(&_high_state_variables.gaitType, &high_state_msg->gaitType, sizeof(_high_state_variables.gaitType));
            memcpy(&_high_state_variables.footRaiseHeight, &high_state_msg->footRaiseHeight, sizeof(_high_state_variables.footRaiseHeight));
            memcpy(&_high_state_variables.position, &high_state_msg->position[0], sizeof(_high_state_variables.position));
            memcpy(&_high_state_variables.bodyHeight, &high_state_msg->bodyHeight, sizeof(_high_state_variables.bodyHeight));
            memcpy(&_high_state_variables.velocity, &high_state_msg->velocity[0], sizeof(_high_state_variables.velocity));
            memcpy(&_high_state_variables.yawSpeed, &high_state_msg->yawSpeed, sizeof(_high_state_variables.yawSpeed));
            memcpy(&_high_state_variables.wirelessRemote, &high_state_msg->wirelessRemote[0], sizeof(_high_state_variables.wirelessRemote));       
        }

        // callback function for the subscriber
        void ballCoordinatesHandler(const geometry_msgs::Point::ConstPtr &ball_coordinates_msg) {  
            // print the x coordinate of the point
            ROS_INFO("x = %f", ball_coordinates_msg->x);
        }

    };
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "movement_node");

  movement_namespace::Movement movement;

  ros::spin();

  return 0;
}