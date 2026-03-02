#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>

#define ALTITUDE_TAKEOFF 2
#define ALLOW_ERROR 0.3


mavros_msgs::State current_mode;
geometry_msgs::PoseStamped init_position_take_off;
geometry_msgs::PoseStamped pose_mav_info;
bool flag_init_position = false;
bool pitch_bl,roll_bl,yaw_bl,land_bl = false;
ros::Publisher local_pub;
enum State {height, forward, right, back, left, yaw, land};
State current_state = height;
Eigen::Quaterniond attitude;

void yaw_test(){
    mavros_msgs::PositionTarget yaw_set;
    pos_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    pos_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW;
    pos_set.yaw_rate = 0.315;
    local_pub.publish(pos_set);
}


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (flag_init_position ==false && (msg->pose.position.z!=0)) //如果还未初始化 且 z坐标不为0>>>
    {
		init_position_take_off.pose.position.x = msg->pose.position.x;
	    init_position_take_off.pose.position.y = msg->pose.position.y;
	    init_position_take_off.pose.position.z = msg->pose.position.z;
        flag_init_position = true; //>>>将“当地位置坐标”设为“初始坐标”并且将“初始化flag”设为true
    }
    pose_mav_info.pose.position.x = msg->pose.position.x;
    pose_mav_info.pose.position.y = msg->pose.position.y;
    pose_mav_info.pose.position.z = msg->pose.position.z;
    pose_mav_info.pose.orientation = msg->pose.orientation;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mode = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cross_t_node");//初始化一个ROS节点
    ros::NodeHandle nh;
    ros::Rate rate(20);
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = ALTITUDE_TAKEOFF;
    bool state_initialized = false;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();//更新请求时间
    ros::Time state_entry_time = ros::Time::now();//状态进入时间
    mavros_msgs::CommandBool arm_cmd;//mavros_msgs::CommandBool：ROS消息类型，用于解锁或锁定无人机
    arm_cmd.request.value = true;//将arm_cmd.request.value设置为true，表示解锁无人机

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/iris_0/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                    ("/iris_0/mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/iris_0/mavros/setpoint_position/local", 10);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>
                    ("/iris_0/mavros/setpoint_raw/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                    ("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("/iris_0/mavros/set_mode");
    

    while(ros::ok())
    {
        if(!state_initialized){
            state_entry_time = ros::Time::now();
            state_initialized = true;
        }

        if( !current_mode.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) //内层条件：检查是否解锁无人机并尝试解锁
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                flag_init_position = false;
            }
            last_request = ros::Time::now();
        }

        if( current_mode.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){ //内层条件：检查是否为起飞模式并尝试切换
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                flag_init_position = false;
            }
            last_request = ros::Time::now();
        }
        
        if(!current_mode.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
        {
            ROS_INFO("Vehicle takeoff");
            last_request = ros::Time::now();
            flag_init_position = false;
        }
        
        switch (current_state)
        {
            case height:
                local_pos_pub.publish(pose);
                if(fabs(pose_mav_info.pose.position.z - init_position_take_off.pose.position.z - ALTITUDE_TAKEOFF) < ALLOW_ERROR)
                   {	
                       if(ros::Time::now() - state_entry_time > ros::Duration(5.0))
                       {
                           ROS_INFO("Vehicle Stable");
                           state_initialized = false;
                           current_state = forward;
                       }
                       
                   }
                break;
            case forward:
                pose.pose.position.x = 1.0;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = ALTITUDE_TAKEOFF;
                local_pos_pub.publish(pose);
                if(fabs(pose_mav_info.pose.position.x - 1.0) < ALLOW_ERROR){
                    if(ros::Time::now() - state_entry_time > ros::Duration(5.0))
                    {
                        ROS_INFO("forward OK!!!!!!!");
                        state_initialized = false;
                        current_state = left;
                    }                   
                }
                break;
            case left:
                pose.pose.position.x = 1.0;
                pose.pose.position.y = -1.0;
                pose.pose.position.z = ALTITUDE_TAKEOFF;
                local_pos_pub.publish(pose);
                if(fabs(pose_mav_info.pose.position.y - 1.0) < ALLOW_ERROR){
                    if(ros::Time::now() - state_entry_time > ros::Duration(5.0))
                    {
                        ROS_INFO("left OK!!!!!!!");
                        state_initialized = false;
                        current_state = back;
                    }                   
                }
                break;
            case back:
                pose.pose.position.x = 0.0;
                pose.pose.position.y = -1.0;
                pose.pose.position.z = ALTITUDE_TAKEOFF;
                local_pos_pub.publish(pose);
                if(fabs(pose_mav_info.pose.position.y - (-1.0)) < ALLOW_ERROR && fabs(pose_mav_info.pose.position.x - 0.0) < ALLOW_ERROR){
                    if(ros::Time::now() - state_entry_time > ros::Duration(5.0))
                    {
                        ROS_INFO("back OK!!!!!!!");
                        state_initialized = false;
                        current_state = right;
                    }                   
                }
                break;
            case right:
                pose.pose.position.x = 0.0;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = ALTITUDE_TAKEOFF;
                local_pos_pub.publish(pose);
                if(fabs(pose_mav_info.pose.position.x - 0.0) < ALLOW_ERROR && fabs(pose_mav_info.pose.position.y - 0.0) < ALLOW_ERROR){
                    if(ros::Time::now() - state_entry_time > ros::Duration(5.0))
                    {
                        ROS_INFO("right OK!!!!!!!");
                        state_initialized = false;
                        current_state = yaw;
                    }                   
                }
                break;                
            case yaw:
                attitude.x() = (pose_mav_info.pose.orientation.x);
                attitude.y() = (pose_mav_info.pose.orientation.y);
                attitude.z() = (pose_mav_info.pose.orientation.z);
                attitude.w() = (pose_mav_info.pose.orientation.w);
                Eigen::Matrix3d rotation_matrix = attitude.toRotationMatrix();
                Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); 
                double drone_yaw = euler_angles[0];
                if(drone_yaw >= 355){
                        ROS_INFO("yaw OK!!!!!!!");
                        current_state = land;                 
                }
                else{
                    yaw_test();
                }
                break;
            case land:
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Land OK!!!!!!!");
                }
                break;

        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("TEST OVER!!!!!!!!!!!!!!!!!");
    
}