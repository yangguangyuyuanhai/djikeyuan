#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    int largest_contour_index = -1;
    cv::Point2d center;

    cv::Mat mask;
//// 单通道判别
    // cv::Mat gray_image;
    // cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    // cv::Mat blurred_image;
    // cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);

    // // 二值化：cv::threshold(输入图像, 输出图像, 阈值, 最大值, 方法);
    // // cv::threshold(blurred_image, mask, 90, 255, cv::THRESH_BINARY_INV);
    // cv::inRange(blurred_image, cv::Scalar(0), cv::Scalar(20), mask);
    
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
        double max_area = 0;
        largest_contour_index = -1;

        for (size_t i = 0; i < contours.size(); i++)
        {
            std::vector<cv::Point> approxContour;
            double perimeter = cv::arcLength(contours[i], true);
            double area = cv::contourArea(contours[i]);
            cv::approxPolyDP(contours[i], approxContour, 0.02 * perimeter, true);
            if (area > 1000 && area > max_area) // 示例：过滤掉面积小于100的轮廓
            {
                if (approxContour.size() == 7 || approxContour.size() == 8 || approxContour.size() == 9)
                {
                    cv::RotatedRect rect = cv::minAreaRect(approxContour);
                    double rectArea = rect.size.width * rect.size.height;
                    double areaRatio = area / rectArea;
                    ROS_INFO("areaRatio:%f", areaRatio);
                    if (areaRatio > 0.18 && areaRatio < 0.42)
                    {
                        max_area = area;
                        center = rect.center;
                        largest_contour_index = i;
                        ROS_INFO("area:%f",area);
                    }
                }
                // else
                
                ROS_INFO("approxContour.size():%ld", approxContour.size());
            }
        }

        if (largest_contour_index != -1)
        {
            // 计算最大轮廓的边界矩形框
            cv::Point image_center(320,240);
            cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);

            cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);

            cv::circle(cv_ptr->image, image_center, 5, cv::Scalar(255, 0, 0), -1);

            // 在原始彩色图像上绘制矩形框
            // cv::rectangle(图像, 矩形, 颜色(BGR), 线宽);
            cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);

            // (可选) 绘制找到的轮廓本身
            cv::drawContours(cv_ptr->image, contours, largest_contour_index, cv::Scalar(0, 0, 255), 2);
        }
    }

    // 在原始彩色图像上绘制矩形框
    // cv::rectangle(图像, 矩形, 颜色(BGR), 线宽);

    cv::imshow("OPENCV_WINDOW_ORIGINAL", cv_ptr->image);
    cv::imshow("OPENCV_WINDOW_BINARY", mask);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_gary");//初始化一个ROS节点
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>
    ("/usb_cam/image_raw", 1, image_cb);

    ros::spin();
}

