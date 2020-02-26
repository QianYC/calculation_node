//
// Created by yc_qian on 20-2-6.
//
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>
#include <fstream>

#include "facedetectcnn.h"
#define DETECT_BUFFER_SIZE 0x20000

cv::CascadeClassifier face_detector;
unsigned char *buffer;

bool init_detectors() {
    cv::String face_cascade_file = "/home/yc_qian/Desktop/Body-Parts-Detection/CascadeFiles/haarcascade_frontalface_alt.xml";
    return face_detector.load(face_cascade_file);
}

void detect_and_draw_face_using_haar(cv::Mat image) {
    cv::Mat grey;
    cv::cvtColor(image, grey, CV_BGR2GRAY);

    std::vector<cv::Rect> results;
    face_detector.detectMultiScale(grey, results, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(20, 20));

    for (int i = 0; i < results.size(); ++i) {
        cv::rectangle(image, results[i], cv::Scalar(255, 255, 255), 2);
    }
}

void detect_and_draw_face_using_libfacedetect_source(cv::Mat image){
    cv::Mat clone = image.clone();
    int *results = NULL;
    results = facedetect_cnn(buffer, (unsigned char *) (clone.ptr(0)), clone.cols, clone.rows, (int) clone.step);

    for (int i = 0; i < (results ? *results : 0); ++i) {
        short * p = ((short*)(results+1))+142*i;
        int x = p[0];
        int y = p[1];
        int w = p[2];
        int h = p[3];
        int confidence = p[4];
        int angle = p[5];

        printf("face_rect=[%d, %d, %d, %d], confidence=%d, angle=%d\n", x,y,w,h,confidence, angle);
        rectangle(image, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
    }
}

void msgCallback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
//    ROS_INFO("image format : %s", msg->format.c_str());
//    ROS_INFO("seq : %d, timestamp : %d, frame_id : %s", msg->header.seq, msg->header.stamp.sec,
//            msg->header.frame_id.c_str());
//    ROS_INFO("data size : %ld", msg->data.size());

    std::time_t start = clock();
    cv::Mat image = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
    detect_and_draw_face_using_libfacedetect_source(image);
//    detect_and_draw_face_using_haar(image);
    double time = ((double) (clock() - start)) / CLOCKS_PER_SEC;
    ROS_INFO("use time : %lf", time);

    cv::imshow("subscriber", image);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_subscriber");

    if (!init_detectors()) {
        ROS_INFO("init detectors failed!");
        return -1;
    }

    buffer = (unsigned char *) malloc(DETECT_BUFFER_SIZE);
    if (!buffer) {
        ROS_INFO("cannot allocate buffer!");
        return -2;
    }

    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/camera/image/compressed", 10, msgCallback);
    ros::spin();
    return 0;
}