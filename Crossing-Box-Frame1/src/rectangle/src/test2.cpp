#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h> 
#include <Eigen/Core>
#include <Eigen/Geometry>

#define ALTITUDE_TAKEOFF 1.0 // 使用浮点数以避免类型转换问题
#define ALLOW_ERROR 0.3

// --- 全局变量 ---
mavros_msgs::State current_mode;
geometry_msgs::PoseStamped pose_mav_info; // 当前无人机位姿
geometry_msgs::PoseStamped init_position_take_off; // 起飞时的地面位置
bool initial_pos_recorded = false; // 用于记录初始位置的标志
bool flag_init_position = false;
enum DroneState { TAKEOFF, FORWARD, LEFT, BACK, RIGHT, YAW, LAND };
DroneState current_state = TAKEOFF;
ros::Publisher local_pub;
double initial_yaw = 0.0; // 用于存储初始偏航角
bool initial_yaw_recorded = false;



//发布无人机位置
void uav_control(mavros_msgs::PositionTarget& uav_set){
    
    uav_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;;
    uav_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    uav_set.yaw = initial_yaw;
    local_pub.publish(uav_set);
}

void uav_body_control(mavros_msgs::PositionTarget& uav_set){
    
    uav_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;;
    uav_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    uav_set.yaw = initial_yaw;
    local_pub.publish(uav_set);
}


// --- 回调函数 ---
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_mode = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (flag_init_position ==false && (msg->pose.position.z!=0)) //如果还未初始化 且 z坐标不为0>>>
    {
		init_position_take_off.pose.position.x = msg->pose.position.x;
	    init_position_take_off.pose.position.y = msg->pose.position.y;
	    init_position_take_off.pose.position.z = msg->pose.position.z;
       flag_init_position = true; //>>>将“当地位置坐标”设为“初始坐标”并且将“初始化flag”设为true
     // 1. 将 geometry_msgs::Quaternion 转换为 Eigen::Quaterniond
        Eigen::Quaterniond initial_orientation(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );

        // 2. 将四元数转换为欧拉角 (Z-Y-X 顺序, 即 yaw-pitch-roll)
        // toRotationMatrix() 将四元数转为旋转矩阵
        // eulerAngles(2, 1, 0) 按 Z, Y, X 顺序提取欧拉角，返回值是一个Vector3d(yaw, pitch, roll)
        auto euler_angles = initial_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        
        // 3. 提取偏航角 (euler_angles[0] 就是 yaw)
        initial_yaw = euler_angles[0];

        initial_yaw_recorded = true;
        ROS_INFO("初始偏航角: %.2f radians (%.2f degrees)", initial_yaw, initial_yaw * 180.0 / M_PI);
    }
    pose_mav_info.pose.position.x = msg->pose.position.x;
    pose_mav_info.pose.position.y = msg->pose.position.y;
    pose_mav_info.pose.position.z = msg->pose.position.z;
    pose_mav_info.pose.orientation = msg->pose.orientation;
}

// 辅助函数：创建用于yaw旋转的四元数
geometry_msgs::Quaternion createQuaternionFromYaw(double yaw_degrees) {
    double yaw_radians = yaw_degrees * M_PI / 180.0;
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw_radians, Eigen::Vector3d::UnitZ()) * // Yaw
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *         // Pitch
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());          // Roll
    geometry_msgs::Quaternion quat_msg;
    quat_msg.w = q.w();
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    return quat_msg;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cross_t_node");
    setlocale(LC_ALL, "");//确保程序能够正确处理本地字符集。例如，如果程序中需要打印中文日志信息，或者处理中文路径
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();//更新请求时间
    mavros_msgs::CommandBool arm_cmd;//mavros_msgs::CommandBool：ROS消息类型，用于解锁或锁定无人机
    arm_cmd.request.value = true;//将arm_cmd.request.value设置为true，表示解锁无人机

    // --- ROS接口 ---
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                    ("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/mavros/setpoint_position/local", 10);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>
                    ("/mavros/setpoint_raw/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                    ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("/mavros/set_mode");

    // // 等待FCU连接
    // ROS_INFO("Waiting for FCU connection...");
    // while (ros::ok() && !current_mode.connected) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("FCU connected.");

    // --- 状态机变量 ---
    mavros_msgs::PositionTarget target_pose;
    Eigen::Quaterniond attitude;
    ros::Time state_entry_time;
    bool state_initialized = false;
    int tick = 0;

    while (ros::ok()) {
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
            flag_init_position = false;
            last_request = ros::Time::now();
        }
        

        // --- 状态机 ---
        if (!state_initialized) {
            state_entry_time = ros::Time::now();
            state_initialized = true;
        }

        switch (current_state) {
            case TAKEOFF:
                target_pose.position.x = init_position_take_off.pose.position.x;
                target_pose.position.y = init_position_take_off.pose.position.y;
                target_pose.position.z = init_position_take_off.pose.position.z + ALTITUDE_TAKEOFF;
                if (fabs(pose_mav_info.pose.position.x - target_pose.position.x) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.y - target_pose.position.y) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.z - target_pose.position.z) < ALLOW_ERROR)
                {
                    tick++;
                    ROS_INFO("count:%d",tick);
                }
                else{
                    tick = 0;
                }
                if (tick >=20 && ros::Time::now() - state_entry_time > ros::Duration(5.0))
                {
                    tick = 0;
                    ROS_INFO("TAKEOFF stable. -> FORWARD");
                    current_state = FORWARD;
                    state_initialized = false;
                }
                uav_control(target_pose);
                break;

            case FORWARD:
                target_pose.position.x = 0.5;
                target_pose.position.y = 0;
                target_pose.position.z = 0;
            
                // 高度保持不变
                if (tick >= 20 && ros::Time::now() - state_entry_time > ros::Duration(5.0))
                {
                    tick = 0;
                    ROS_INFO("FORWARD OK. -> LEFT");
                    current_state = LEFT;
                    state_initialized = false;
                }
                uav_control(target_pose);
                break;

            case LEFT:
                target_pose.position.x = init_position_take_off.pose.position.x + 1.0;
                target_pose.position.y = init_position_take_off.pose.position.y + 1.0;
                if (fabs(pose_mav_info.pose.position.x - target_pose.position.x) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.y - target_pose.position.y) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.z - target_pose.position.z) < ALLOW_ERROR)
                {
                    tick++;
                    ROS_INFO("count:%d",tick);
                }
                else{
                    tick = 0;
                }
                if (tick >= 20 && ros::Time::now() - state_entry_time > ros::Duration(5.0))
                {
                    tick = 0;
                    ROS_INFO("LEFT OK. -> BACK");
                    current_state = BACK;
                    state_initialized = false;
                }
                uav_control(target_pose);
                break;

            case BACK:
                target_pose.position.x = init_position_take_off.pose.position.x;
                target_pose.position.y = init_position_take_off.pose.position.y + 1.0;
                if (fabs(pose_mav_info.pose.position.x - target_pose.position.x) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.y - target_pose.position.y) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.z - target_pose.position.z) < ALLOW_ERROR)
                {
                    tick++;
                    ROS_INFO("count:%d",tick);
                }
                else{
                    tick = 0;
                }
                if (tick >= 20 && ros::Time::now() - state_entry_time > ros::Duration(5.0))
                {
                    tick = 0;
                    ROS_INFO("BACK OK. -> RIGHT");
                    current_state = RIGHT;
                    state_initialized = false;
                }
                uav_control(target_pose);
                break;

            case RIGHT:
                target_pose.position.x = init_position_take_off.pose.position.x;
                target_pose.position.y = init_position_take_off.pose.position.y;
                if (fabs(pose_mav_info.pose.position.x - target_pose.position.x) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.y - target_pose.position.y) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.z - target_pose.position.z) < ALLOW_ERROR)
                {
                    tick++;
                    ROS_INFO("count:%d",tick);
                }
                else{
                    tick = 0;
                }
                if (tick >= 20 && ros::Time::now() - state_entry_time > ros::Duration(5.0))
                {
                    tick = 0;
                    ROS_INFO("RIGHT OK (back to start). -> LAND");
                    current_state = LAND;
                    state_initialized = false;
                }
                uav_control(target_pose);
                break;

            // case YAW:
            //     {   
            //     attitude.x() = (pose_mav_info.pose.orientation.x);
            //     attitude.y() = (pose_mav_info.pose.orientation.y);
            //     attitude.z() = (pose_mav_info.pose.orientation.z);
            //     attitude.w() = (pose_mav_info.pose.orientation.w);
            //     Eigen::Matrix3d rotation_matrix = attitude.toRotationMatrix();
            //     Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); 
            //     double drone_yaw = euler_angles[0];
            //     // 使用角度差来判断
            //     if (drone_yaw >= 355)
            //     {
            //         ROS_INFO("YAW OK. -> LAND");
            //         current_state = LAND;
            //         state_initialized = false;
            //     }
            //     yaw_test(yaw_set);
            //     }
            //     break;

            case LAND:
                { 
                mavros_msgs::SetMode land_mode;
                land_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                    ROS_INFO("Landing command sent.");
                    ros::shutdown(); // 任务完成，关闭节点
                }
                }
                break;
        }

        

        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("TEST OVER!!!!!!!!!!!!!!!!!");
    return 0; 
}