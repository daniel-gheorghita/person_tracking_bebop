#define JOYSTICK_CONTROL 1
#define HOVER_POINT 2
#define FOLLOW_PERSON 3

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "math.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

int control_mode = 0; 
int useHovering = 1;
double global_roll, global_pitch, global_yaw, global_gaz;
ros::Publisher  takeoff_pub;
ros::Publisher  land_pub;
ros::Publisher  toggleState_pub;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void joystickCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
    double roll, pitch, yaw, gaz;
    //control_mode = JOYSTICK_CONTROL; // callback enter might be a manual command;
    
    if (joy_msg->buttons[0] == 1)
    {
        control_mode = HOVER_POINT;
        std::cout << "Joystick Callback: HOVER." << std::endl;
    }
    if (joy_msg->buttons[3] == 1)
    {
        control_mode = FOLLOW_PERSON;
        std::cout << "Joystick Callback: FOLLOW." << std::endl;
    }
    
    //ROS_INFO("I heard: [%s]", joy_msg->buttons);
    std::cout << "Joystick Callback: " << std::endl;
      
    for (int i = 0; i < 4; i++)
    {
        std::cout << "Joystick Axes " << i << ": " << joy_msg->axes[i] << std::endl;
    }
      
    for (int i = 0; i < 17; i++)
    {
        std::cout << "Joystick Buttons " << i << ": " << joy_msg->buttons[i] << std::endl;
    }
      
      
      // button[4] = take off / land
      // button[5] = reset
      // axis[0] = 
      // axis[1] = 
      // axis[2] = 
      // axis[3] = 
    
    yaw = -joy_msg->axes[2];
    gaz = joy_msg->axes[3];
    roll = -joy_msg->axes[0];
    pitch = -joy_msg->axes[1];
    if (abs(yaw) > 0.1 || abs(gaz) > 0.1 || abs(roll) > 0.1 || abs(pitch) > 0.1)
    {
        control_mode = JOYSTICK_CONTROL;
        std::cout << "Joystick Callback: MANUAL." << std::endl;
        global_roll = roll;
        global_yaw = yaw;
        global_pitch = pitch;
        global_gaz = gaz;
    }
    
    if (joy_msg->buttons[4] == 1)
    {
        // send take off message
        takeoff_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: TAKE OFF." << std::endl;
    }
    else
    {
        // send land message
        land_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: LAND." << std::endl;
    }
    
    if (joy_msg->buttons[5] == 1)
    {
        // send take off message
        toggleState_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: RESET." << std::endl;
    }
    std::cout << "-------------------------------------" << std::endl;
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "visnav_controller");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, joystickCallback);
    ros::Publisher  vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName("cmd_vel"),1);
    takeoff_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/takeoff"),1);
    land_pub    = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/land"),1);
    toggleState_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/reset"),1);
    geometry_msgs::Twist cmdT;
    
    ros::Rate loop_rate(10);
    control_mode = JOYSTICK_CONTROL;
    
    // run node - send control commands
    while (ros::ok())
    {
        // Init for safety
        global_yaw = 0;
        global_pitch = 0;
        global_gaz = 0;
        global_roll = 0;

        // Prepare command
        cmdT.angular.z = -global_yaw;
        cmdT.linear.z = global_gaz;
        cmdT.linear.x = -global_pitch;
        cmdT.linear.y = -global_roll;
        cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
        
        // Send command
        vel_pub.publish(cmdT);

        ros::spinOnce();

        loop_rate.sleep();
    
    }


  return 0;
}