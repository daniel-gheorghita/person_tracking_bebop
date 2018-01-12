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
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_datatypes.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
ros::Publisher  vel_pub;

int control_mode = 0; 
int useHovering = 1;
double yawMean, yawSum;
int yawCount = 30;
double global_roll, global_pitch, global_yaw, global_gaz;
int sent_TakeOff = 0;
ros::Publisher  takeoff_pub;
ros::Publisher  land_pub;
ros::Publisher  toggleState_pub;
struct poseStruct {
    double x;
    double y;
    double z;
    double yaw;
    int valid; // if error information is worth considering
} poseError, d_poseError, last_poseError, i_poseError, dronePose, desiredPose, last_desiredPose, local_desiredPose;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void joystickCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
    double roll, pitch, yaw, gaz;
    //control_mode = JOYSTICK_CONTROL; // callback enter might be a manual command;
    
    
    //std::cout << "Joystick Callback: " << std::endl;
      
    for (int i = 0; i < 4; i++)
    {
      //  std::cout << "Joystick Axes " << i << ": " << joy_msg->axes[i] << std::endl;
    }
      
    for (int i = 0; i < 17; i++)
    {
      //  std::cout << "Joystick Buttons " << i << ": " << joy_msg->buttons[i] << std::endl;
    }
      
      
    
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
        geometry_msgs::Twist cmdT;


        
    }
    
    if (joy_msg->buttons[4] == 1)
    {
        // send take off message
        takeoff_pub.publish(std_msgs::Empty());
        //std::cout << "Joystick Callback: TAKE OFF." << std::endl;
    }
    else
    {
        // send land message
        land_pub.publish(std_msgs::Empty());
        //std::cout << "Joystick Callback: LAND." << std::endl;
    }
    
    if (joy_msg->buttons[11] == 1)
    {
        // send take off message
        toggleState_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: RESET." << std::endl;
    }
    //std::cout << "-------------------------------------" << std::endl;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static int yawIdx;

    dronePose.x = msg->pose.pose.position.x;
    dronePose.y = msg->pose.pose.position.y;
    dronePose.z = msg->pose.pose.position.z;
    
    // TODO 
    // orientation
    double qw, qz, qx, qy;
    qw = msg->pose.pose.orientation.w;
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;

    double siny = +2.0 * (qw * qz + qx * qy);
    double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
    dronePose.yaw = atan2(siny, cosy);
    
    if (yawIdx < yawCount)
    {
        yawSum += dronePose.yaw;
        yawIdx++;
        std::cout << std::endl << "----- YAW CALIBRATION NOT READY !!!! -----" << std::endl << std::endl;
    }
    
    else
    {
        yawMean = yawSum / yawIdx;
        std::cout <<  "  raw drone yaw: " << dronePose.yaw << std::endl;
        dronePose.yaw -= yawMean;
        
        // CLIP yaw
            if (dronePose.yaw > 3.1416 / 2) 
                dronePose.yaw -= 3.1416;
            if (dronePose.yaw < -3.1416 / 2)
                dronePose.yaw += 3.1416;
        
        
        // represent the desirePose in the drone frame
        local_desiredPose.y = desiredPose.x * sin(dronePose.yaw) + desiredPose.y * cos(dronePose.yaw);
        local_desiredPose.x = desiredPose.x * cos(dronePose.yaw) - desiredPose.y * sin(dronePose.yaw);
        local_desiredPose.z = desiredPose.z;
    }
    
    
    
    /* 
        ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
   */
}

void markerPoseCallback(ar_track_alvar_msgs::AlvarMarkers req)
{
    //TODO
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
        //std::cout << "GOT MESSAGE FROM ALVAR!!!" << std::endl;
        //std::cout << "ID: " << id <<"; X: " << x << "; Y: " << y << "; Z: " << z << std::endl << std::endl;
        
        // TODO, get orientation information
            
            double siny = +2.0 * (qw * qz + qx * qy);
            double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);  
            double yaw = atan2(siny, cosy);
            
            // CLIP yaw
            if (yaw > 3.1416 / 2) 
                yaw -= 3.1416;
            if (yaw < -3.1416 / 2)
                yaw += 3.1416;
        
        
 //       std::cout << "ID: " << id <<"; X: " << x << "; Y: " << y << "; Z: " << z << "; Yaw: " << yaw << std::endl;
        
        if (control_mode == TRACK_MARKER)
        {
      //      std::cout << "Tracking marker." << std::endl;
            
            
            
            
            // store last desired pose
            last_desiredPose.x = desiredPose.x; 
            last_desiredPose.y = desiredPose.y;
            last_desiredPose.z = desiredPose.z;
            
            // update desired pose
            desiredPose.x = dronePose.x + x - 1; // 1m in front of the marker
            desiredPose.y = dronePose.y + y;
            desiredPose.z = dronePose.z + z;
            
            // discard large variations
            if (abs(desiredPose.x - last_desiredPose.x) > 2 || abs(desiredPose.y - last_desiredPose.y) > 2 || abs(desiredPose.z - last_desiredPose.z) > 2)
            {
                desiredPose.x = last_desiredPose.x; 
                desiredPose.y = last_desiredPose.y;
                desiredPose.z = last_desiredPose.z;
            } 
            // assign yaw error
            //poseError.yaw = yaw;
            //poseError.valid = 1;
       
        }
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
        else
            if (strcmp(argv[1], "marker") == 0)
            {
                control_mode = TRACK_MARKER;
                std::cout << "Control mode: Marker." << std::endl;
            }
            else
                if (strcmp(argv[1], "hover") == 0)
                {
                    control_mode = HOVER_POINT;
                    
                    // Dummy point for testing
                    desiredPose.x = 1;
                    desiredPose.y = 1;
                    desiredPose.z = 1;
                    std::cout << "Control mode: Hover." << std::endl;
                }
                else
                    if (strcmp(argv[1], "face") == 0)
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
                    }
    }
    else
    {
        control_mode = JOYSTICK_CONTROL;
        std::cout << "No selected control input. Joystick selected by default." << std::endl;
        std::cout << "You can restart the node in the following modes: " << std::endl;
        std::cout << "1. Joystick: joy;\n2. Marker tracking: marker;\n3. Hover: hover;\n4. Face tracking: face. " << std::endl;
                    
    }
    
    // init node
    ros::init(argc, argv, "visnav_controller");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::Subscriber odom_sub = n.subscribe("ardrone/odom", 1000, odomCallback);
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, joystickCallback);
    ros::Subscriber alvar_sub = n.subscribe("ar_pose_marker", 1000, markerPoseCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName("cmd_vel"),1);
    takeoff_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/takeoff"),1);
    land_pub    = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/land"),1);
    toggleState_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/reset"),1);
    geometry_msgs::Twist cmdT;
    
    ros::Rate loop_rate(10);
    
    
    // run node - send control commands
    std::cout << "NODE STARTED" << std::endl;
    while (ros::ok())
    {
        
        // clear console
        std::cout << "\x1B[2J\x1B[H";
 
        // Empty command
        cmdT.angular.z = 0;
        cmdT.linear.z = 0;
        cmdT.linear.x = 0;
        cmdT.linear.y = 0;
        cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
        
        // Controller
      
        switch (control_mode)
        {
            case JOYSTICK_CONTROL:
            {
                std::cout << "Control mode: JOYSTICK." << std::endl;
                // Prepare command
                cmdT.angular.z = -global_yaw;
                cmdT.linear.z = global_gaz;
                cmdT.linear.x = -global_pitch;
                cmdT.linear.y = -global_roll;
                cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
            
                // Empty callback data
                global_pitch = 0;
                global_gaz = 0;
                global_roll = 0;
                global_yaw = 0;
                
                break;
            }
            case TRACK_MARKER:
            {
                std::cout << "Control mode: TRACK MARKER." << std::endl;

                // store last error
                last_poseError.x = poseError.x;
                last_poseError.y = poseError.y;
                last_poseError.z = poseError.z;
                last_poseError.yaw = poseError.yaw;
                
                // new error
                poseError.x = desiredPose.x - dronePose.x;
                poseError.y = desiredPose.y - dronePose.y;
                poseError.z = desiredPose.z - dronePose.z;
                poseError.yaw = desiredPose.yaw - dronePose.yaw;
                
                
                // PD controller params
                double Kp = 0.5; 
                double Kd = 0;
                
                // apply dead-zone
                //poseError.z = (abs(poseError.z) < 0.02) ? 0 : poseError.z;
                //poseError.x = (abs(poseError.x) < 0.02) ? 0 : poseError.x;
                //poseError.y = (abs(poseError.y) < 0.02) ? 0 : poseError.y;
                
                // Prepare command (PD controller)
                cmdT.angular.z = 0;
                cmdT.linear.z = poseError.z * Kp + (poseError.z - last_poseError.z) * Kd;
                cmdT.linear.x = (poseError.x) * Kp + (poseError.x - last_poseError.x) * Kd;
                cmdT.linear.x = 0;
                cmdT.linear.y = poseError.y * Kp + (poseError.y - last_poseError.y) * Kd;
                //cmdT.linear.y = 0;
                cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
                
                // Clip command
                double limit = 0.1;
                cmdT.linear.z = (cmdT.linear.z > limit) ? limit : cmdT.linear.z;
                cmdT.linear.z = (cmdT.linear.z < -limit) ? -limit : cmdT.linear.z;
                cmdT.linear.x = (cmdT.linear.x > limit) ? limit : cmdT.linear.x;
                cmdT.linear.x = (cmdT.linear.x < -limit) ? -limit : cmdT.linear.x;
                cmdT.linear.y = (cmdT.linear.y > limit) ? limit : cmdT.linear.y;
                cmdT.linear.y = (cmdT.linear.y < -limit) ? -limit : cmdT.linear.y;
                
                
                break;
            }
            case TRACK_FACE:
            {
                std::cout << "Control mode: TRACK FACE." << std::endl;

                break;
            }
            case HOVER_POINT:
            {
                std::cout << "Control mode: HOVER POINT." << std::endl;                 

                // store last error
                last_poseError.x = poseError.x;
                last_poseError.y = poseError.y;
                last_poseError.z = poseError.z;
                last_poseError.yaw = poseError.yaw;
                
                // new error
                poseError.x = desiredPose.x - dronePose.x;
                poseError.y = desiredPose.y - dronePose.y;
                poseError.z = desiredPose.z - dronePose.z;
                poseError.yaw = desiredPose.yaw - dronePose.yaw;
                
                
                // PD controller params
                double Kp = 0.1; 
                double Kd = 0;
                
                // apply dead-zone
                //poseError.z = (abs(poseError.z) < 0.02) ? 0 : poseError.z;
                //poseError.x = (abs(poseError.x) < 0.02) ? 0 : poseError.x;
                //poseError.y = (abs(poseError.y) < 0.02) ? 0 : poseError.y;
                
                // Prepare command (PD controller)
                cmdT.angular.z = 0;
                cmdT.linear.z = poseError.z * Kp + (poseError.z - last_poseError.z) * Kd;
                cmdT.linear.x = (poseError.x) * Kp + (poseError.x - last_poseError.x) * Kd;
                //cmdT.linear.x /= 2;
                cmdT.linear.y = poseError.y * Kp + (poseError.y - last_poseError.y) * Kd;
                cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
                
                // Clip command
                double limit = 0.1;
                cmdT.linear.z = (cmdT.linear.z > limit) ? limit : cmdT.linear.z;
                cmdT.linear.z = (cmdT.linear.z < -limit) ? -limit : cmdT.linear.z;
                cmdT.linear.x = (cmdT.linear.x > limit) ? limit : cmdT.linear.x;
                cmdT.linear.x = (cmdT.linear.x < -limit) ? -limit : cmdT.linear.x;
                cmdT.linear.y = (cmdT.linear.y > limit) ? limit : cmdT.linear.y;
                cmdT.linear.y = (cmdT.linear.y < -limit) ? -limit : cmdT.linear.y;





                break;
            }
            default:
            {
                std::cout << "Invalid control mode!" << std::endl;
                break;
            }
        }
        
        // Send command
        vel_pub.publish(cmdT);


        // display info
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "Desired pose -> x: " << local_desiredPose.x << " y: " << local_desiredPose.y << " z: " << local_desiredPose.z << std::endl;
        std::cout << "Current pose -> x: " << dronePose.x << " y: " << dronePose.y << " z: " << dronePose.z << " yaw: " << dronePose.yaw << std::endl;
        std::cout << "Error pose -> x: " << poseError.x << " y: " << poseError.y << " z: " << poseError.z << std::endl;
        std::cout << "Command -> x: " << cmdT.linear.x << " y: " << cmdT.linear.y << " z: " << cmdT.linear.z << std::endl;


        ros::spinOnce();

        loop_rate.sleep();
    
    }


  return 0;
}
