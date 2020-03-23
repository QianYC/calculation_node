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
#include <sensor_msgs/CompressedImage.h>
#include <pi_robot/SrvTrigger.h>
#include "facedetectcnn.h"
#include "robotBase.hpp"

enum DETECT_RESULT {
    NO_FACE = 0X0,
    FACE_TOO_SMALL = 0X1,
    HAVE_FACE = 0X2
};

enum COMPOSITION_RULE {
    NONE = 0X0,
    RULE_OF_THREE = 0X1,
    CENTER = 0X2
};

struct COMPOSE_RESULT {
    int x;
    int y;
    int w;
    int h;
    COMPOSITION_RULE rule;
};

/**
 * FR1.2, FR1.3
 * @param img
 * @return
 */
vector<FaceRect> object_detect(cv::Mat &img, DETECT_RESULT &result);

COMPOSE_RESULT compose(cv::Mat &img, vector<FaceRect> faces);

/**
 * FR2
 * @param img
 */
void calculate_position(cv::Mat& img);

/**
 * FR3
 */
void adjust_position();

/**
 * FR4
 */
void take_photo(ros::ServiceClient &client);

/**
 * FR5
 * @param img
 */
void process_photo(cv::Mat& img);



#endif //PI_ROBOT_PHOTOCOMPOSER_HPP
