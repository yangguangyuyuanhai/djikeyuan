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
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "PID_controller.h"
#include <deque>

#define ALTITUDE_TAKEOFF 1.0
#define X_START 0
#define Y_START 0
#define ALLOW_ERROR 0.3
#define FRAME_VY 0.5
#define FRAME_VZ 0
#define THROUGH_RECTANGLE_AREA 35000
#define CAMERA_OFFSET_PIXEL_X 0
#define CAMERA_OFFSET_PIXEL_Y 0
#define CROSS_VEL 2.5
#define THOUGH_DISTANCE 1.7
#define INITIAL_YAW -1.434

float K[] = {436.3234862285308, 0, 320.0991280652177, 0, 437.2606550351687, 240.2058585681155, 0, 0, 1};
mavros_msgs::State current_mode;
PID_Controller *pos_control;
mavros_msgs::PositionTarget vel;
mavros_msgs::PositionTarget pos;
ros::Publisher local_pub;
bool land_flag = false;
bool task = false;
int approach_step = 0;
bool flag_init_position = false;
bool init_pose = false;
bool flag_init_displacement = false;
bool flag_init_change_flag = false;
bool flag_flag_change_direction_can_true = false;
bool flag_direction_change = false;


ros::Time last_no_detect_frame;
ros::Time last_detect_frame;
ros::Time last_no_aim;
ros::Time time_0_5s_move;

geometry_msgs::PoseStamped pose_mav_info;
geometry_msgs::PoseStamped init_position_take_off;
geometry_msgs::PoseStamped pose_start_to_through;
geometry_msgs::PoseStamped pose_track;
geometry_msgs::TwistStamped g_vel_body;
int direction_y = 0;
int direction_z = 0;
double initial_yaw = INITIAL_YAW;
double pose_start_to_through_yaw; //开始穿框时的偏航角
double vel_start_to_through_y;
std::deque<int> sign_history_y;   // 保存 y 方向速度符号
// std::deque<int> sign_history_z;   // 保存 z 方向速度符号

// inline void push_sign(std::deque<int>& q, double v, std::size_t max_len = 10)
// {
//     int sign = (v > 0.0) - (v < 0.0);   // 得到 +1 / 0 / -1
//     q.push_back(sign);
//     if (q.size() > max_len) q.pop_front();
// }

// inline int segment_product(const std::deque<int>& q, std::size_t start, std::size_t len)
// {
//     int prod = 1;
//     for (std::size_t i = start; i < start + len; ++i)
//         prod *= q[i];
//     return prod;
// }

// inline bool all_same_sign(const std::deque<int>& q,
//                           std::size_t start, std::size_t len)
// {
//     int first = q[start];
//     for (std::size_t i = start + 1; i < start + len; ++i)
//         if (q[i] != first) return false;
//     return true;
// }

// void on_new_velocity(mavros_msgs::PositionTarget& new_vel)
// {
//     push_sign(sign_history_y, new_vel.velocity.y);

//     if (sign_history_y.size() == 10)   // 已收集满 10 帧
//     {
//         // y 方向
//         bool y_ok = all_same_sign(sign_history_y, 0, 5) &&
//                     all_same_sign(sign_history_y, 5, 5) &&
//                     sign_history_y[0] != sign_history_y[5];

//         // z 方向
//         // bool z_ok = all_same_sign(sign_history_z, 0, 5) &&
//         //             all_same_sign(sign_history_z, 5, 5) &&
//         //             sign_history_z[0] != sign_history_z[5];

//         if (y_ok)   // 任一方向满足条件
//         {
//             pose_start_to_through = pose_mav_info;
//             flag_direction_change = true;
//             // 如只需触发一次，可加标志位或清空队列
//         }
//     }
// }

void uav_control_vel(mavros_msgs::PositionTarget& vel_set){
        vel_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        vel_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        vel_set.yaw = initial_yaw - 1.570795;
        local_pub.publish(vel_set);
}

void uav_control_pos(mavros_msgs::PositionTarget& pos_set){

        pos_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_set.type_mask = 
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        // pos_set.yaw = initial_yaw;
        local_pub.publish(pos_set);
}


void uav_control_pos_vel(mavros_msgs::PositionTarget& pos_vel_set)
{
        pos_vel_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_vel_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_vel_set.yaw = initial_yaw - 1.570795;
        local_pub.publish(pos_vel_set);
}


void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    int center_x;
    int center_y;
    cv::Point2d center;
    cv::Point2d original_center;
    original_center.x = K[2];
    center.x = K[2];
    original_center.y = K[5];
    center.y = K[5];
    cv_bridge::CvImagePtr cv_ptr;
    static double error_y = 0;
    static double error_z = 0;
    static double error_posix_x = 0;
    static double error_posix_y = 0;
    static double biggest_area = 0.0;
    int largest_contour_index = -1;
    double max_area = 0;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask;
//// 单通道判别
    // cv::Mat gray_image;
    // cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    // cv::Mat blurred_image;
    // cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);

    // // 二值化：cv::threshold(输入图像, 输出图像, 阈值, 最大值, 方法);
    // // cv::threshold(blurred_image, mask, 90, 255, cv::THRESH_BINARY_INV);
    // cv::inRange(blurred_image, cv::Scalar(0), cv::Scalar(50), mask);
    
//// 三通道判别
    cv::Mat raw_image, hsv;
    cv::GaussianBlur(cv_ptr->image, raw_image, cv::Size(11, 11), 0);
    cv::GaussianBlur(raw_image, raw_image, cv::Size(11, 11), 0);
    cv::GaussianBlur(raw_image, raw_image, cv::Size(5, 5), 0);
    cv::cvtColor(raw_image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 70, 20), cv::Scalar(130, 255, 255), mask); // 100-130 蓝色

    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(输入二值图, 输出轮廓, 层次结构, 模式, 方法);
    // cv::RETR_EXTERNAL: 只检测最外层的轮廓
    // cv::CHAIN_APPROX_SIMPLE: 压缩水平、垂直和对角线段，只保留其端点
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
    {
        largest_contour_index = -1;
        max_area = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            std::vector<cv::Point> approxContour;
            double perimeter = cv::arcLength(contours[i], true);
            double area = cv::contourArea(contours[i]);
            //cv::approxPolyDP(contours[i], approxContour, 0.02 * perimeter, true);
            // contours[i]：原始轮廓（第 i 个物体）
            //approxContour:逼近后的多边形（顶点集合）
            //0.02 * perimeter: 逼近精度：允许轮廓点与拟合边之间的最大距离
            //tru：轮廓是闭合的
            cv::approxPolyDP(contours[i], approxContour, 0.02 * perimeter, true);
            if (area > 1000 && area > max_area) // 示例：过滤掉面积小于100的轮廓
            {
                if (approxContour.size() == 3 || approxContour.size() == 4 || approxContour.size() == 5 || approxContour.size() == 6 || approxContour.size() == 7 || approxContour.size() == 8 || approxContour.size() == 9)
                {
                    cv::RotatedRect rect = cv::minAreaRect(approxContour);//这个点集最紧凑的外接矩形
                    double rectArea = rect.size.width * rect.size.height;
                    double areaRatio = area / rectArea;
                    if (areaRatio > 0.18 && areaRatio < 0.43)
                    {
                        max_area = rectArea;
                        // ROS_INFO("max_area, %f",max_area);
                        center = rect.center;
                        largest_contour_index = i;
                    }
                    else
                    {
                        ;
                        // ROS_INFO("areaRatio, %f",areaRatio);
                    }
                }
                else
                {
                    ;
                    //  ROS_INFO("approxContour.size():%ld", approxContour.size());
                }
                if (task)
                {
                    if(approach_step != 2 && approach_step != 3)
                    {
                        ROS_INFO("____________TRACK____________");
                        last_no_aim = ros::Time::now();
                        last_detect_frame = ros::Time::now();
                        time_0_5s_move = ros::Time::now();
                        approach_step = 2;
                        pose_track.pose.position.x = pose_mav_info.pose.position.x;
                    }
                }
            }

            if (largest_contour_index != -1)
            {
                cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);

                // ROS_INFO("center:(%f,%f)",center.x, center.y);
                cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);

                // 在原始彩色图像上绘制矩形框
                // cv::rectangle(图像, 矩形, 颜色(BGR), 线宽);
                cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);

                // (可选) 绘制找到的轮廓本身
                cv::drawContours(cv_ptr->image, contours, largest_contour_index, cv::Scalar(0, 0, 255), 2);
            }
        }
    }

    //<-------PID计算------->
    //<像素误差计算>
    error_posix_x = (center.x - CAMERA_OFFSET_PIXEL_X) - original_center.x;
    error_posix_y = (center.y - CAMERA_OFFSET_PIXEL_Y) - original_center.y;
    //<转换到机体坐标系计算>
    error_y = error_posix_x * 0.04;
    error_z = error_posix_y * 0.04;
    // ROS_INFO("PID误差,y = %.3f,z = %.3f",error_y,error_z);
    //<PID控制>
    vel = pos_control->control(0, error_y, error_z);
    vel.velocity.x = 0;
    if(task && approach_step == 0)//没目标时前往目标位置，有目标1秒后进入"1"
    {
        // ROS_INFO("无人机速度,x = %.3f,y = %.3f,z = %.3f",vel.velocity.x,vel.velocity.y,vel.velocity.z);
            pos.position.x = init_position_take_off.pose.position.x;
            pos.position.y = init_position_take_off.pose.position.y;
            pos.position.z = init_position_take_off.pose.position.z + ALTITUDE_TAKEOFF;
        if(largest_contour_index == -1)
        {
            ROS_INFO("None Frame");
            last_no_detect_frame = ros::Time::now();
        }
        else 
        {
          approach_step = 1;
          // ROS_INFO("approach_step:%d",approach_step);
        }
        uav_control_pos(pos);
    }
    else if(task && approach_step == 1)//有目标往前飞，丢失目标时退后
    {
        if(largest_contour_index == -1)
        {
            pos.position.x = init_position_take_off.pose.position.x;
            pos.position.y = init_position_take_off.pose.position.y;
            pos.position.z = init_position_take_off.pose.position.z + ALTITUDE_TAKEOFF;
    
            ROS_INFO("approach_step:%d，Lost_Frame",approach_step);
            uav_control_pos(pos);
        }
        else
        {
            vel.velocity.x = 0.0;
            ROS_INFO("approach_step:%d",approach_step);
            uav_control_vel(vel);
        }
    }
    else if(task && approach_step == 2)//有目标保持距离跟踪
    {
        static geometry_msgs::PoseStamped pos_before; 
        static double distance_now_to_before_along_yaw__before;
        if(largest_contour_index == -1)
        {
            ROS_INFO("approach_step:%d，Lost_Frame",approach_step);
            vel.velocity.x = 0;
            vel.velocity.y = 0;
            vel.velocity.z = 0;

            // ROS_INFO("(real:%f)", pose_mav_info.pose.position.x);
            if(ros::Time::now() - last_detect_frame > ros::Duration(1.0))
            {
                flag_init_displacement == false;
                flag_init_change_flag == false;
                flag_direction_change = false;
                flag_flag_change_direction_can_true = false;
                approach_step = 1;
            }
            if(ros::Time::now() - last_detect_frame > ros::Duration(0.2))
            {
                time_0_5s_move = ros::Time::now();
            }
            uav_control_vel(vel);
            last_no_aim = ros::Time::now();
        }
        else
        {
            last_detect_frame = ros::Time::now();
            vel.velocity.x = 0;
            if(max_area > THROUGH_RECTANGLE_AREA)
            {
                vel.velocity.x = -0.025;
                // ROS_INFO("max_area, %f",max_area);
                // ROS_INFO("back");
            }
            else if(max_area < THROUGH_RECTANGLE_AREA - 1000)
            {
                vel.velocity.x = 0.02;
                // ROS_INFO("max_area, %f",max_area);
                // ROS_INFO("front");
                // ROS_INFO("front:%f<%d",max_area,THROUGH_RECTANGLE_AREA - 1000);
            }

            // /////// 穿静止框：稳定对准目标3秒后穿框 ////////
            // if(error_posix_x < 10 && error_posix_y < 10)
            // {
            //     ROS_INFO("持续瞄准中心: %.3f 秒", (ros::Time::now() - last_no_aim).toSec());
            //     if(ros::Time::now() - last_no_aim > ros::Duration(2.5))
            //     {
            //         pose_start_to_through = pose_mav_info;
            //         pose_start_to_through_yaw = initial_yaw; 
            //         vel_start_to_through_y = 0;
            //         approach_step = 3;
            //         ROS_INFO("approach_step:%d, AIM",approach_step);
            //     }
            // }
            // else
            // {
            //     last_no_aim = ros::Time::now();
            // }
            
            
            ///// 穿动态框：在无人机变向后执行穿框 ////////
            if(ros::Time::now() - time_0_5s_move > ros::Duration(1))
            {
                if(flag_init_change_flag == true)
                {
                    
                    double dx_2 = pose_mav_info.pose.position.x - pos_before.pose.position.x;
                    double dy_2 = pose_mav_info.pose.position.y - pos_before.pose.position.y;
                    double distance_now_to_before = sqrt(dx_2*dx_2 + dy_2*dy_2);
                    double distance_now_to_before_along_yaw = dx_2 * cos(initial_yaw - 3.14159/2) + dy_2 * sin(initial_yaw - 3.14159/2);
                    if(flag_init_displacement == true)
                    {
                        if(distance_now_to_before_along_yaw__before * distance_now_to_before_along_yaw > 0)
                        {
                            if(flag_flag_change_direction_can_true == false)
                            {
                                ROS_INFO("-------------------Track Stable----------------------");
                            }  
                            flag_flag_change_direction_can_true = true;
                        }
                        else
                        {
                            // ROS_INFO("______");
                            // ROS_INFO("before:%f", distance_now_to_before_along_yaw__before);
                            // ROS_INFO("now:%f", distance_now_to_before_along_yaw);
                            // ROS_INFO("______");
                            if(flag_flag_change_direction_can_true == false)
                            {
                                ROS_INFO("Track NO Stable");
                            }  
                        }

                        if(flag_flag_change_direction_can_true == true)
                        {
                            if(distance_now_to_before_along_yaw__before * distance_now_to_before_along_yaw < 0)
                            {
                                flag_direction_change = true;
                                ROS_INFO("----------------------Direction Reversal-------------------");
                            }
                        }  
                    }
                    else
                    {
                        flag_init_displacement = true;
                    }
                    // 下一个0.5秒的位移before
                    distance_now_to_before_along_yaw__before = distance_now_to_before_along_yaw;
                }
                else
                {
                    flag_init_change_flag = true;
                }
                // 下一个0.5秒的位置before
                pos_before.pose.position.x = pose_mav_info.pose.position.x;
                pos_before.pose.position.y = pose_mav_info.pose.position.y;
                time_0_5s_move = ros::Time::now();
            }
            else
            {
                // ROS_INFO("0.5s:%.3f", (ros::Time::now() - time_0_5s_move).toSec());
            }
            

            
            if(flag_flag_change_direction_can_true == true)
            {             
                if(flag_direction_change == true && error_posix_x < 10 && error_posix_y < 10)
                {
                    ROS_INFO("持续瞄准中心: %.3f 秒", (ros::Time::now() - last_no_aim).toSec());
                    if(ros::Time::now() - last_no_aim > ros::Duration(2))
                    {
                        pose_start_to_through = pose_mav_info;
                        pose_start_to_through_yaw = initial_yaw; 
                        vel_start_to_through_y = FRAME_VY;
                        approach_step = 3;
                        ROS_INFO("approach_step:%d, AIM",approach_step);
                    }
                }
                else
                {
                    last_no_aim = ros::Time::now();
                }
            }
            
            pose_start_to_through = pose_mav_info;

            /////// 穿动态框：根据已知框的移动范围，设定执行穿框的坐标范围(已弃用) ////////

            // if(error_posix_x < 30 & error_posix_y < 30)
            // {
            //     if(pose_mav_info.pose.position.y <= -0 && pose_mav_info.pose.position.y >= -3 && direction_y > 0)
            //     {
            //         // ROS_INFO("%d,%d,%d",pose_mav_info.pose.position.y <= -0, pose_mav_info.pose.position.y >= -3, direction_y > 0);
            //         // approach_step = 3;
            //         pose_start_to_through = pose_mav_info;
            //     }
            //     else if(pose_mav_info.pose.position.y > 0 && pose_mav_info.pose.position.y <= 3 && direction_y < 0)
            //     {
            //         // ROS_INFO("%d,%d,%d",pose_mav_info.pose.position.y > 0, pose_mav_info.pose.position.y <= 3, direction_y < 0);
            //         // approach_step = 3;
            //         pose_start_to_through = pose_mav_info;
            //     }
            // }
            // ROS_INFO("y:%.2f,z:%.2f",vel.velocity.y, vel.velocity.z);
            uav_control_vel(vel);
        }
    }
    else if(task && approach_step == 3)//穿框
    {
        
        direction_y = vel.velocity.y / abs(vel.velocity.y);
        direction_z = vel.velocity.z / abs(vel.velocity.z);
        double dx = pose_mav_info.pose.position.x - pose_start_to_through.pose.position.x;
        double dy = pose_mav_info.pose.position.y - pose_start_to_through.pose.position.y;
        double distance_from_start = sqrt(dx*dx + dy*dy);
        double distance_along_yaw = dx * cos(pose_start_to_through_yaw) + dy * sin(pose_start_to_through_yaw);
        if(land_flag == false)
        {
            ROS_INFO("--穿框中，初始朝向的位移：%f--",distance_along_yaw);
        }
        if (distance_along_yaw < THOUGH_DISTANCE)
        {
            vel.velocity.x = CROSS_VEL;
            vel.velocity.y = vel_start_to_through_y * direction_y;
            vel.velocity.z = 0;
            //ROS_INFO("Final_task:%f", pose_mav_info.pose.position.x - pose_start_to_through.pose.position.x);
            uav_control_vel(vel);
            
        }
        else
        {
            vel.velocity.x = 0;
            land_flag = true;
        }
    }
    
    // 显示带有边界框的原始图像
    cv::circle(cv_ptr->image, original_center, 5, cv::Scalar(255, 0, 0), -1);
    cv::imshow("OPENCV_WINDOW_ORIGINAL", cv_ptr->image);
    // 显示二值化后的图像，方便调试
    // cv::imshow("OPENCV_WINDOW_BINARY", mask);
    // cv::waitKey(1);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);        

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);     

    // ROS_INFO("Current yaw = %.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);
    
    if (init_pose == false)
    {
		init_position_take_off.pose.position.x = msg->pose.position.x;
	    init_position_take_off.pose.position.y = msg->pose.position.y;
	    init_position_take_off.pose.position.z = msg->pose.position.z;
        initial_yaw = yaw;
        ROS_INFO("初始偏航角: %.2f radians (%.2f degrees)", initial_yaw, initial_yaw * 180.0 / M_PI);
        init_pose = true;
    }
    pose_mav_info.pose.position.x = msg->pose.position.x;
    pose_mav_info.pose.position.y = msg->pose.position.y;
    pose_mav_info.pose.position.z = msg->pose.position.z;
    pose_mav_info.pose.orientation = msg->pose.orientation;
    // ROS_INFO("偏航角: %.2f radians (%.2f degrees)", yaw, yaw * 180.0 / M_PI);
}
    

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mode = *msg;
}

// void velBodyCb(const geometry_msgs::TwistStamped& msg){
//         g_vel_body = *msg;  // 记录
//         ROS_INFO_THROTTLE(1.0,
//             "Body-FRD vel: vx=%.3f, vy=%.3f, vz=%.3f m/s",
//             msg->twist.linear.x,
//             msg->twist.linear.y,
//             msg->twist.linear.z);
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "cross_t_node");//初始化一个ROS节点
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    pos_control = new PID_Controller();
    ROS_INFO("%d",THROUGH_RECTANGLE_AREA);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                    ("/mavros/state", 10, state_cb);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>
                    ("/usb_cam/image_raw", 1, image_cb);
    // ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>
    //                 ("/mavros/local_position/velocity_body", 10, velBodyCb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/mavros/setpoint_position/local", 10);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>
                    ("/mavros/setpoint_raw/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                    ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("/mavros/set_mode");
    
    while(init_pose == false)
    {
        ros::spinOnce();
    }

    mavros_msgs::PositionTarget target_pose;
    target_pose.position.x = init_position_take_off.pose.position.x;
    target_pose.position.y = init_position_take_off.pose.position.y;
    target_pose.position.z = init_position_take_off.pose.position.z + ALTITUDE_TAKEOFF;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();//更新请求时间
    mavros_msgs::CommandBool arm_cmd;//mavros_msgs::CommandBool：ROS消息类型，用于解锁或锁定无人机
    arm_cmd.request.value = true;//将arm_cmd.request.value设置为true，表示解锁无人机
    
    while(ros::ok())
    {
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
        uav_control_pos(target_pose);
        if (fabs(pose_mav_info.pose.position.x - target_pose.position.x) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.y - target_pose.position.y) < ALLOW_ERROR
                    && fabs(pose_mav_info.pose.position.z - target_pose.position.z) < ALLOW_ERROR)
        {	
            if(ros::Time::now() - last_request > ros::Duration(2.0))
            {
                ROS_INFO("Vehicle Stable");
                last_no_detect_frame = ros::Time::now();
                task = true;
                break;
            }
        }
        // //超过10秒说明高度定位不准，直接进入任务
        // if(ros::Time::now() - last_request > ros::Duration(10.0))
        // {
        //     ROS_INFO("Takeoff timeout, start the task ");
        //     task = true;
        //     break;
        // }
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        if(land_flag == true)
        {
            if(pose_mav_info.pose.position.z - target_pose.position.z > 0.3)
            {
                vel.velocity.x = 0.1;
                vel.velocity.y = 0;
                vel.velocity.z = -0.1;
                uav_control_vel(vel);
            }
            else
            {
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Land");
                    
                }
                last_request = ros::Time::now();
            }  
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    delete pos_control;
}