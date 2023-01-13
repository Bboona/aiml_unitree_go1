#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>


namespace joystick_listener_namespace {
    
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

    class JoystickListener {
        private:
            // Node handles
            ros::NodeHandle _nh;

            // Private variables
            bool _running = false;
            int REMOTE_BUTTON_L1 = 0; // Find out which values these buttons are
            int REMOTE_BUTTON_L2 = 1;

            high_state_struct _high_state_variables;

            // Subscribers
            ros::Subscriber high_state_subscriber;

        public:
            JoystickListener(): _nh() {
                high_state_subscriber = _nh.subscribe("high_state", 1, &JoystickListener::HighStateHandler, this);
            };

        
        void HighStateHandler(const unitree_legged_msgs::HighState::ConstPtr &high_state_msg) {
            
            memcpy(&_high_state_variables.wirelessRemote, &high_state_msg->wirelessRemote[0], sizeof(_high_state_variables.wirelessRemote));

            if (high_state_msg->wirelessRemote[REMOTE_BUTTON_L1] && high_state_msg->wirelessRemote[REMOTE_BUTTON_L2]) {
                if (!_running) {
                    // Start
                    std::cout << "Start doing the thing..."<< std::endl;
                    _nh.setParam("do_the_thing", 1);
                    _running = true;
                } else {
                    // Stop
                    std::cout << "Stop doing the thing..."<< std::endl;
                    _nh.setParam("do_the_thing", 0);
                    _running = false;
                }
            }
        }
    };
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_listener_node");

  joystick_listener_namespace::JoystickListener joystick;

  ros::spin();

  return 0;
}