//
// Created by yc_qian on 20-3-15.
//

#ifndef PI_ROBOT_PHOTOCOMPOSER_HPP
#define PI_ROBOT_PHOTOCOMPOSER_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "facedetectcnn.h"
#include "robotBase.hpp"

/**
 * total system requirement
 * @param img
 */
void compose(cv::Mat img);

/**
 * FR1.2, FR1.3
 * @param img
 * @return
 */
vector<FaceRect> object_detect(cv::Mat img);

/**
 * FR2
 * @param img
 * @param faces
 */
void calculate_position(cv::Mat img, vector<FaceRect> faces);

/**
 * FR3
 */
void adjust_position();

/**
 * FR4
 */
void take_photo();

/**
 * FR5
 * @param img
 */
void process_photo(cv::Mat img);

enum STATE {
    /**
     * 摆拍模式
     */
    STATIC = 0X0,
    /**
     * 摆拍，目标已确定
     */
    S_OBJECT_SELECTED = 0X1,
    /**
     * 摆拍，构图已确定
     */
    S_COMPOSITION_SELECTED = 0X2,
    /**
     * 摆拍，位置已调整
     */
    S_POSITION_ADJUSTED = 0X4,
    /**
     * 抓拍模式
     */
    DYNAMIC = 0X8,
    /**
     * 抓拍，目标已确定
     */
    D_OBJECT_SELECTED = 0X10
};

#endif //PI_ROBOT_PHOTOCOMPOSER_HPP
