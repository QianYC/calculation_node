//
// Created by yc_qian on 20-3-15.
//

#ifndef PI_ROBOT_PHOTOCOMPOSER_HPP
#define PI_ROBOT_PHOTOCOMPOSER_HPP

#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/CompressedImage.h>
#include <pi_robot/SrvTrigger.h>
#include <pi_robot/SrvCapture.h>
#include "facedetectcnn.h"
#include "robotBase.hpp"
#include "ultraface.hpp"
#include "thread_pool.hpp"
#include "motion_publisher.hpp"

enum DETECT_RESULT {
    NO_FACE = 0X0,
    FACE_TOO_SMALL = 0X1,
    HAVE_FACE = 0X2
};

enum COMPOSITION_MODE {
    SINGLE = 0X1,
    MULTIPLE = 0X2
};

struct COMPOSE_RESULT {
    float x1;
    float y1;
    float x2;
    float y2;
    COMPOSITION_MODE mode;
};

struct CHECK_ERROR_RESULT {
    bool success;
    MOTION direction;
};

struct RECT {
    float x1, y1, x2, y2;
};

/**
 * FR1.2, FR1.3
 * @param img
 * @return
 */
vector<FaceInfo> object_detect(cv::Mat &img, DETECT_RESULT &result);

COMPOSE_RESULT calculate_expected_position(vector<FaceInfo> faces);

CHECK_ERROR_RESULT calculate_error(COMPOSE_RESULT &result, cv::Mat &image);

void calculate_position(cv::Mat& img);

/**
 * FR4
 */
void take_photo(ros::ServiceClient &client, const COMPOSE_RESULT &result = {.x1=-1});

COMPOSE_RESULT dynamic_get_target(vector<FaceInfo> faces);

#endif //PI_ROBOT_PHOTOCOMPOSER_HPP
