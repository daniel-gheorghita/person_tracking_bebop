#define JOYSTICK_CONTROL 1
#define HOVER_POINT 2
#define TRACK_FACE 3
#define TRACK_MARKER 4
#define TRACK_PERSON 5

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sstream>
ros::Publisher  vel_pub;

geometry_msgs::Twist cmdT;
int control_mode = 0;
int original_control = 0; 
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

void sendCmd_byError(poseStruct errorPose)
{
    // PD controller params
    double Kp = 0.8; 
    double Kd = 1.8;
    static poseStruct last_errorPose;
    
    // Apply deadzone
//    double deadzone = 0.1;
//    errorPose.x = (abs(errorPose.x) > deadzone) ? errorPose.x : 0;
//    errorPose.y = (abs(errorPose.y) > deadzone) ? errorPose.y : 0;
//    errorPose.z = (abs(errorPose.z) > deadzone) ? errorPose.z : 0;
//    errorPose.yaw = (abs(errorPose.yaw) > deadzone) ? errorPose.yaw : 0;

    
        
    // Prepare command (PD controller)
    cmdT.angular.z = Kp * errorPose.yaw + (errorPose.yaw - last_errorPose.yaw) * Kd;
    cmdT.linear.z = errorPose.z * Kp + (errorPose.z - last_errorPose.z) * Kd;
    //cmdT.linear.z = 0; // testing
    cmdT.linear.x = (errorPose.x) * Kp / 2 + (errorPose.x - last_errorPose.x) * Kd * 1.5;
//    cmdT.linear.x = 0; // testing
    cmdT.linear.y = errorPose.y * Kp / 2  + (errorPose.y - last_errorPose.y) * Kd * 1.5;
  //  cmdT.linear.y = 0; // testing
    //cmdT.linear.y = 0;
    cmdT.angular.x = cmdT.angular.y = useHovering ? 0 : 1;
                
    // Clip command
    double limit = 0.2;
    cmdT.linear.z = (cmdT.linear.z > limit) ? limit : cmdT.linear.z;
    cmdT.linear.z = (cmdT.linear.z < -limit) ? -limit : cmdT.linear.z;
    cmdT.linear.x = (cmdT.linear.x > limit) ? limit : cmdT.linear.x;
    cmdT.linear.x = (cmdT.linear.x < -limit) ? -limit : cmdT.linear.x;
    cmdT.linear.y = (cmdT.linear.y > limit) ? limit : cmdT.linear.y;
    cmdT.linear.y = (cmdT.linear.y < -limit) ? -limit : cmdT.linear.y;
    cmdT.angular.z = (cmdT.angular.z > limit) ? limit : cmdT.angular.z;
    cmdT.angular.z = (cmdT.angular.z < -limit) ? -limit : cmdT.angular.z;
                
    vel_pub.publish(cmdT);
    last_errorPose = errorPose;
}

void sendCmd(poseStruct desiredPose)
{
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
                
    vel_pub.publish(cmdT);
}

void joystickCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
    double roll, pitch, yaw, gaz;
      
    //for (int i = 0; i < 4; i++)
    //    std::cout << "Joystick Axes " << i << ": " << joy_msg->axes[i] << std::endl;
    //for (int i = 0; i < 17; i++)
    //    std::cout << "Joystick Buttons " << i << ": " << joy_msg->buttons[i] << std::endl;
    
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
    
    if (joy_msg->buttons[4] > 0.5)
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
    
    if (joy_msg->buttons[5] > 0.5)
    {
        // send take off message
        toggleState_pub.publish(std_msgs::Empty());
        std::cout << "Joystick Callback: RESET." << std::endl;
    }
    
    if (joy_msg->buttons[0] > 0.5)
    {
        control_mode = original_control;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (control_mode != HOVER_POINT) return;

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

    local_desiredPose.y = -desiredPose.x * sin(dronePose.yaw) + desiredPose.y * cos(dronePose.yaw);
    local_desiredPose.x = desiredPose.x * cos(dronePose.yaw) + desiredPose.y * sin(dronePose.yaw);
    local_desiredPose.z = desiredPose.z;
    sendCmd(local_desiredPose);    
}

void markerPoseCallback(ar_track_alvar_msgs::AlvarMarkers req)
{
    
    
    if (control_mode != TRACK_MARKER && control_mode != TRACK_PERSON) return;
    double yaw = 0, x = 0, y = 0, z = 0;
    int countMarkers = 0;
    //TODO
    static int emptyFrames; // if last 10 (?) frames are empty, something is wrong: LAND or HOVER
    
    if (!req.markers.empty())
    {
        

        
        
        emptyFrames = 0;
        
        for (int i = 0; i < req.markers.size(); i++)
        {
            int id = req.markers[i].id;
            
            if (id != 3 && id != 4 ) continue;
            
            countMarkers++;
            x += req.markers[i].pose.pose.position.x;
            y += req.markers[i].pose.pose.position.y;
            z += req.markers[i].pose.pose.position.z;
            double qx = req.markers[i].pose.pose.orientation.x;
            double qy = req.markers[i].pose.pose.orientation.y;
            double qz = req.markers[i].pose.pose.orientation.z;
            double qw = req.markers[i].pose.pose.orientation.w;
            
            std::cout << "ID: " << id <<"; X: " << x << "; Y: " << y << "; Z: " << z << std::endl << std::endl;
        
            // TODO, get orientation information
            Eigen::Quaterniond q(qw,qx,qy,qz);
            Eigen::Vector3d marker_zAxis(0,0,1), drone_xAxis(1,0,0);
            Eigen::Matrix3d rotation; 
            Eigen::Vector3d marker_zAxis_local_frame;
            rotation = q.toRotationMatrix();
            marker_zAxis_local_frame = rotation * marker_zAxis;
        
            marker_zAxis_local_frame[2] = 0;
            double cosineValue;
            double normValue = marker_zAxis_local_frame.norm();
            cosineValue = marker_zAxis_local_frame.dot(drone_xAxis) / normValue;
            
            yaw += acos(cosineValue) * marker_zAxis_local_frame[0] * marker_zAxis_local_frame[1];            
        }
        
        // store last desired pose
        last_desiredPose.x = desiredPose.x;
        last_desiredPose.y = desiredPose.y;
        last_desiredPose.z = desiredPose.z;
       
        // update desired pose
        desiredPose.yaw = yaw / countMarkers ;
        desiredPose.x = x / countMarkers - 1;
        desiredPose.y = y / countMarkers;
        desiredPose.z = z / countMarkers;

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
        local_desiredPose = desiredPose;

        sendCmd_byError(desiredPose);
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

void facePoseCallback(tf2_msgs::TFMessage face_tf)
{

    
    //std::cout << "FACE CALLBAAAAACK!!" << std::endl;
    if (control_mode != TRACK_FACE && control_mode != TRACK_PERSON) return;
    if (face_tf.transforms.empty()) return;
    //std::cout << "No. transforms: " << face_tf.transforms.size() << std::endl;
    double yaw = 0, x = 0, y = 0, z = 0;
    double qx, qy, qz, qw;

    for (int i = 0; i < face_tf.transforms.size(); i++)
    {
        if (strcmp(face_tf.transforms[0].child_frame_id.c_str(), "face_0") != 0) continue;


        //std::cout << "FACE DETECTED!" << std::endl;
        x = face_tf.transforms[i].transform.translation.x;
        y = face_tf.transforms[i].transform.translation.y;
        z = face_tf.transforms[i].transform.translation.z;
        //std::cout << "FACE-> " <<" X: " << x << "; Y: " << y << "; Z: " << z << std::endl << std::endl;
        local_desiredPose.x = z - 1.2;
        local_desiredPose.y = -x;
        local_desiredPose.z = -y;
        
        qx = face_tf.transforms[i].transform.rotation.x;
        qy = face_tf.transforms[i].transform.rotation.y;
        qz = face_tf.transforms[i].transform.rotation.z;
        qw = face_tf.transforms[i].transform.rotation.w;
        
        // get yaw
        Eigen::Quaterniond q(qw,qx,qy,qz);
        Eigen::Vector3d marker_zAxis(0,0,1), drone_xAxis(1,0,0);
        Eigen::Matrix3d rotation;
        Eigen::Vector3d marker_zAxis_local_frame;
        rotation = q.toRotationMatrix();
        marker_zAxis_local_frame = rotation.transpose() * marker_zAxis;
        
        marker_zAxis_local_frame[2] = 0;
        double cosineValue;
        double normValue = marker_zAxis_local_frame.norm();
        cosineValue = marker_zAxis_local_frame.dot(drone_xAxis) / normValue;

        yaw = acos(cosineValue) * marker_zAxis_local_frame[0] * marker_zAxis_local_frame[1];
        
        local_desiredPose.yaw = -yaw;
        sendCmd_byError(local_desiredPose);

    }

    //TODO
    static int emptyFrames; // if last 10 (?) frames are empty, something is wrong: LAND or HOVER
}

int main(int argc, char **argv)
{
    // init controller
    if (argc == 2)
    {
        if (strcmp(argv[1], "joy") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = JOYSTICK_CONTROL;
            std::cout << "Control mode: Joystick." << std::endl;
        }
        else if (strcmp(argv[1], "marker") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = TRACK_MARKER;
            std::cout << "Control mode: Marker." << std::endl;
        }
        else if (strcmp(argv[1], "hover") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = HOVER_POINT;

            // Dummy point for testing
            desiredPose.x = 1;
            desiredPose.y = 1;
            desiredPose.z = 1;
            std::cout << "Control mode: Hover." << std::endl;
        }
        else if (strcmp(argv[1], "face") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = TRACK_FACE;
            std::cout << "Control mode: Face tracker." << std::endl;
        }
        
        else if (strcmp(argv[1], "person") == 0)
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = TRACK_PERSON;
            std::cout << "Control mode: Person tracker." << std::endl;
        }
        
        else
        {
            control_mode = JOYSTICK_CONTROL;
            original_control = JOYSTICK_CONTROL;
            std::cout << "No valid control input. Joystick selected by default. " << std::endl;
            std::cout << "You can restart the node in the following modes: " << std::endl;
            std::cout << "1. Joystick: joy;\n2. Marker tracking: marker;\n3. Hover: hover;\n4. Face tracking: face. " << std::endl;
        }
    }
    else
    {
        control_mode = JOYSTICK_CONTROL;
        original_control = control_mode;

        std::cout << "No selected control input. Joystick selected by default." << std::endl;
        std::cout << "You can restart the node in the following modes: " << std::endl;
        std::cout << "1. Joystick: joy;\n2. Marker tracking: marker;\n3. Hover: hover;\n4. Face tracking: face. " << std::endl;
    }
    
    // init node
    ros::init(argc, argv, "bebop_controller");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Subscriber odom_sub = n.subscribe("ardrone/odom", 1000, odomCallback);
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, joystickCallback);
    ros::Subscriber alvar_sub = n.subscribe("ar_pose_marker", 1000, markerPoseCallback);
    ros::Subscriber face_sub = n.subscribe("tf",1000, facePoseCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName("cmd_vel"),1);
    takeoff_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/takeoff"),1);
    land_pub    = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/land"),1);
    toggleState_pub = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/reset"),1);

    
    ros::Rate loop_rate(10);
        
    // run node - send control commands
    std::cout << "NODE STARTED" << std::endl;
    while (ros::ok())
    {
        // clear console
        std::cout << "\x1B[2J\x1B[H";
 
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
                
                vel_pub.publish(cmdT);
                
                break;
            }
            case TRACK_MARKER:
            {
                std::cout << "Control mode: TRACK MARKER." << std::endl;
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
                break;
            }
            case TRACK_PERSON:
            {
                std::cout << "Control mode: TRACK PERSON." << std::endl;                 
                break;
            }
            default:
            {
                std::cout << "Invalid control mode!" << std::endl;
                break;
            }
        }

        // display info
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "Desired pose -> x: " << local_desiredPose.x << " y: " << local_desiredPose.y << " z: " << local_desiredPose.z << " yaw: " << local_desiredPose.yaw << std::endl;
        std::cout << "Current pose -> x: " << dronePose.x << " y: " << dronePose.y << " z: " << dronePose.z << " yaw: " << dronePose.yaw << std::endl;
        std::cout << "Error pose -> x: " << poseError.x << " y: " << poseError.y << " z: " << poseError.z << " yaw: " << poseError.yaw << std::endl;
        std::cout << "Command -> x: " << cmdT.linear.x << " y: " << cmdT.linear.y << " z: " << cmdT.linear.z << " Rot z: " << cmdT.angular.z << std::endl;

        ros::spinOnce();

        loop_rate.sleep();    
    }

    return 0;
}
