#define JOYSTICK_CONTROL 1
#define HOVER_POINT 2
#define TRACK_FACE 3
#define TRACK_MARKER 4

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <sstream>
#include <pthread.h>

static pthread_mutex_t send_CS = PTHREAD_MUTEX_INITIALIZER;
int control_mode = 0; 
int useHovering = 1;
double global_roll, global_pitch, global_yaw, global_gaz;
bool lastL1Pressed = false;
bool lastR1Pressed = false;
ros::Publisher  takeoff_pub;
ros::Publisher  land_pub;
ros::Publisher  vel_pub;
ros::Publisher  toggleState_pub;
struct errorStruct {
    double x;
    double y;
    double z;
    double yaw;
    int valid; // if error information is worth considering
} poseError, d_poseError, last_poseError, i_poseError;

struct ControlCommand
{
  inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
  inline ControlCommand(double roll, double pitch, double yaw, double gaz)
  {
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    this->gaz = gaz;
  }
  double yaw, roll, pitch, gaz;
};

void sendLand()
{
  std::cout << "land" << std::endl;
  pthread_mutex_lock(&send_CS);
  land_pub.publish(std_msgs::Empty());
  pthread_mutex_unlock(&send_CS);
}
void sendTakeoff()
{
  std::cout << "take off" << std::endl;
  pthread_mutex_lock(&send_CS);
  takeoff_pub.publish(std_msgs::Empty());
  pthread_mutex_unlock(&send_CS);
}

void sendControlToDrone(ControlCommand cmd)
{
  std::cout << "send control to drone" << std::endl;

  // TODO: check converstion (!)
  geometry_msgs::Twist cmdT;
  cmdT.angular.z = -cmd.yaw;
  cmdT.linear.z = cmd.gaz;
  cmdT.linear.x = -cmd.pitch;
  cmdT.linear.y = -cmd.roll;

  cmdT.angular.x = cmdT.angular.y = 0; // use hovering

  pthread_mutex_lock(&send_CS);
  vel_pub.publish(cmdT);
  pthread_mutex_unlock(&send_CS);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void joyCb(const sensor_msgs::JoyConstPtr joy_msg)
{
//  std::cout << "joy callback" << std::endl;

  if (joy_msg->axes.size() < 4) {
    ROS_WARN_ONCE("Error: Non-compatible Joystick!");
    return;
  }

  // Avoid crashes if non-ps3 joystick is being used
  short unsigned int actiavte_index = (joy_msg->buttons.size() > 11) ? 11 : 1;

  // if not controlling: start controlling if sth. is pressed (!)
  bool justStartedControlling = false;

  ControlCommand c;
  c.yaw = -joy_msg->axes[2];
  c.gaz = joy_msg->axes[3];
  c.roll = -joy_msg->axes[0];
  c.pitch = -joy_msg->axes[1];

  sendControlToDrone(c);
  //lastJoyControlSent = c;

  if(!lastL1Pressed && joy_msg->buttons.at(actiavte_index - 1))
    sendTakeoff();
  if(lastL1Pressed && !joy_msg->buttons.at(actiavte_index - 1))
    sendLand();

  lastL1Pressed =joy_msg->buttons.at(actiavte_index - 1);
  lastR1Pressed = joy_msg->buttons.at(actiavte_index);
}

void joystickCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
    double roll, pitch, yaw, gaz;
    //control_mode = JOYSTICK_CONTROL; // callback enter might be a manual command;
    
    // TODO
    // CHECK BETTER, until then joystick cannot switch to autonomous control
    /*
    if (joy_msg->buttons[0] == 1)
    {
        control_mode = HOVER_POINT;
        std::cout << "Joystick Callback: HOVER." << std::endl;
    }
    if (joy_msg->buttons[3] == 1)
    {
        control_mode = TRACK_FACE;
        std::cout << "Joystick Callback: FOLLOW." << std::endl;
    }
    */ 
    
    //ROS_INFO("I heard: [%s]", joy_msg->buttons);
    std::cout << "Joystick Callback: " << std::endl;
      
    for (int i = 0; i < 4; i++)
        std::cout << "Joystick Axes " << i << ": " << joy_msg->axes[i] << std::endl;
      
    for (int i = 0; i < 17; i++)
        std::cout << "Joystick Buttons " << i << ": " << joy_msg->buttons[i] << std::endl;
    
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

        // Prepare command
        geometry_msgs::Twist cmdT;
        
        cmdT.angular.z = -global_yaw;
        cmdT.linear.z = global_gaz;
        cmdT.linear.x = -global_pitch;
        cmdT.linear.y = -global_roll;
        cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
        
        // Send command
        pthread_mutex_lock(&send_CS);
        vel_pub.publish(cmdT);
        pthread_mutex_unlock(&send_CS);
    }
    
    if (joy_msg->buttons[10] == 1)
    {
        // send take off message
        pthread_mutex_lock(&send_CS);
        takeoff_pub.publish(std_msgs::Empty());
        pthread_mutex_unlock(&send_CS);
        std::cout << "Joystick Callback: TAKE OFF." << std::endl;
    }
    else
    {
        // send land message
        pthread_mutex_lock(&send_CS);
        land_pub.publish(std_msgs::Empty());
        pthread_mutex_unlock(&send_CS);
        std::cout << "Joystick Callback: LAND." << std::endl;
    }
    
    if (joy_msg->buttons[11] == 1)
    {
        // send take off message
        toggleState_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: RESET." << std::endl;
    }
    std::cout << "-------------------------------------" << std::endl;
}

// TODO
// predictedPose callback : ardrone/predictedPose
// if target point and predictedPose (state) is far, send cmd_vel
// DroneController update

void markerPoseCallback(ar_track_alvar_msgs::AlvarMarkers req)
{
    // TODO
    // DroneController setTarget

    static int emptyFrames; // if last 10 (?) frames are empty, something is wrong: LAND or HOVER
    
    if (!req.markers.empty())
    {
        emptyFrames = 0;
        double x = req.markers[0].pose.pose.position.x;
        double y = req.markers[0].pose.pose.position.y;
        double z = req.markers[0].pose.pose.position.z;
        double qx = req.markers[0].pose.pose.orientation.x;
        double qy = req.markers[0].pose.pose.orientation.y;
        double qz = req.markers[0].pose.pose.orientation.z;
        double qw = req.markers[0].pose.pose.orientation.w;
        int id = req.markers[0].id;
        std::cout << "GOT MESSAGE FROM ALVAR!!!" << std::endl;

        poseError.x = x;
        poseError.y = y;
        poseError.z = z;
        // TODO, get orientation information
        
        double siny = +2.0 * (qw * qz + qx * qy);
        double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
        double yaw = atan2(siny, cosy);
        
        // CLIP yaw
        if (yaw > 3.1416 / 2) 
            yaw -= 3.1416;
        if (yaw < -3.1416 / 2)
            yaw += 3.1416;
            
        // assign yaw error
        poseError.yaw = yaw;
        poseError.valid = 1;
        std::cout << "ID: " << id <<"; X: " << x << "; Y: " << y << "; Z: " << z << "; Yaw: " << yaw << std::endl << std::endl;
    }
    else
    {
        emptyFrames++;
        if (emptyFrames > 10)
        {
            // TODO: what happens after we mark the detection as faulty?
            poseError.valid = 0;
        }
    }
}

int main(int argc, char **argv)
{
    // init controller
    if (argc == 2)
    {
        if (strcmp(argv[1], "joy") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            std::cout << "Control mode: Joystick." << std::endl;
        }
        else if (strcmp(argv[1], "marker") == 0)
        {
            control_mode = TRACK_MARKER;
            std::cout << "Control mode: Marker." << std::endl;
        }
        else if (strcmp(argv[1], "hover") == 0)
        {
            control_mode = HOVER_POINT;
            std::cout << "Control mode: Hover." << std::endl;
        }
        else if (strcmp(argv[1], "face") == 0)
        {
            control_mode = TRACK_FACE;
            std::cout << "Control mode: Face tracker." << std::endl;
        }
        else
        {
            control_mode = JOYSTICK_CONTROL;
            std::cout << "No valid control input. Joystick selected by default. " << std::endl;
            std::cout << "You can restart the node in the following modes: " << std::endl;
            std::cout << "1. Joystick: joy;\n2. Marker tracking: marker;\n3. Hover: hover;\n4. Face tracking: face. " << std::endl;
            return 0;
        }
    }
    else
    {
        control_mode = JOYSTICK_CONTROL;
        std::cout << "No selected control input. Joystick selected by default." << std::endl;
        std::cout << "You can restart the node in the following modes: " << std::endl;
        std::cout << "1. Joystick: joy;\n2. Marker tracking: marker;\n3. Hover: hover;\n4. Face tracking: face. " << std::endl;
        return 0;
    }
    
    // init node
    ros::init(argc, argv, "visnav_controller");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, joystickCallback);
    ros::Subscriber alvar_sub = n.subscribe("ar_pose_marker", 1000, markerPoseCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName("cmd_vel"),1);
    takeoff_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/takeoff"),1);
    land_pub    = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/land"),1);
    toggleState_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/reset"),1);
    
//    ros::Rate loop_rate(10);
    
    // Init for safety
    global_yaw = 0;
    global_pitch = 0;
    global_gaz = 0;
    global_roll = 0;
        
    // run node - send control commands
    std::cout << "NODE STARTED" << std::endl;
    ros::spin();
/*    while (ros::ok())
    {
        // Controller
        
        computedCmd.roll = (control_force(0) * sin(yaw) - control_force(1) * cos(yaw)) / (m * g);
        computedCmd.pitch = (control_force(0) * cos(yaw) + control_force(1) * sin(yaw)) / (m * g);
        computedCmd.yaw_rate = 0.1;
        computedCmd.thrust.x = 0;
        computedCmd.thrust.y = 0;
        computedCmd.thrust.z = control_force(2) + m * g;

        double Kp = 0.01;
        double Ki = 0;
        double Kd = 0;
        global_yaw = Kp * poseError.yaw;
        global_pitch = Kp * (poseError.x * cos(poseError.yaw) + poseError.y * sin(poseError.yaw));
        global_roll = Kp * (poseError.x * sin(poseError.yaw) - poseError.y * cos(poseError.yaw)); 
        global_gaz = Kp * poseError.z;

        // Prepare command
        cmdT.angular.z = -global_yaw;
        cmdT.linear.z = global_gaz;
        cmdT.linear.x = -global_pitch;
        cmdT.linear.y = -global_roll;
        cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
        
        // Send command
        pthread_mutex_lock(&send_CS);
        vel_pub.publish(cmdT);
        pthread_mutex_unlock(&send_CS);

        ros::spinOnce();

        loop_rate.sleep();
    }
*/
    return 0;
}
