//
// Created by yc_qian on 20-2-6.
//
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>
#include <fstream>
#include <thread>

#include "facedetectcnn.h"
#define DETECT_BUFFER_SIZE 0x20000

#include "robotBase.hpp"
static ParameterReader params;

#include "photoComposer.hpp"

#include "pi_robot/SrvState.h"

cv::CascadeClassifier face_detector;
unsigned char *buffer;

bool init_detectors() {
    cv::String face_cascade_file = "../../../../cascadefiles/haarcascade_frontalface_alt.xml";
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
        cv::rectangle(image, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
    }

    /**
     * FR1.2 detect faces
     */
    vector<FaceRect> faces = objectdetect_cnn(clone.ptr(0), clone.cols, clone.rows, clone.step);
}

STATE state = STATIC;
bool change_state = false;
vector<FaceRect> faces;
cv::Mat image;

void loop(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    ROS_INFO("image subscriber thread id : %d",this_thread::get_id());
    switch (state) {
        case STATIC:
            ROS_INFO("STATIC");
            if (change_state) {
                state = DYNAMIC;
                change_state = false;
                break;
            }
            image = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
            faces = object_detect(image);
            if (faces.size() > 0) {
                state = S_OBJECT_SELECTED;
            }
            break;
        case S_OBJECT_SELECTED:
            ROS_INFO("S_OBJECT_SELECTED");
            calculate_position(image, faces);
            state = S_COMPOSITION_SELECTED;
            break;
        case S_COMPOSITION_SELECTED:
            ROS_INFO("S_COMPOSITION_SELECTED");
            adjust_position();
            state = S_POSITION_ADJUSTED;
            break;
        case S_POSITION_ADJUSTED:
            ROS_INFO("S_POSITION_ADJUSTED");
            take_photo();
            state = STATIC;
            break;
        case DYNAMIC:
            ROS_INFO("DYNAMIC");
            if (change_state) {
                state = STATIC;
                change_state = false;
                break;
            }
            image = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
            faces = object_detect(image);
            if (faces.size() > 0) {
                state = D_OBJECT_SELECTED;
            }
            break;
        case D_OBJECT_SELECTED:
            ROS_INFO("D_OBJECT_SELECTED");
            take_photo();
            state = DYNAMIC;
            break;
        default:
            break;
    }
    if (!image.empty()) {
        cv::imshow("subscriber", image);
        cv::waitKey(1);
    }
}

bool service_change_state(pi_robot::SrvStateRequest &request, pi_robot::SrvStateResponse &response) {
    ROS_INFO("change state server thread id : %d", this_thread::get_id());
    response.old_state = state;
    change_state = true;
    return true;
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
    ros::Subscriber image_subscriber = nodeHandle.subscribe("/camera/image/compressed", 1, loop);
    ros::ServiceServer state_server = nodeHandle.advertiseService("change_state", service_change_state);

    ros::spin();
    return 0;
}