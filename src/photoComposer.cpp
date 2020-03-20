//
// Created by yc_qian on 20-3-15.
//
#include "photoComposer.hpp"

static ParameterReader params;


vector<FaceRect> object_detect(cv::Mat& img, DETECT_RESULT &result) {
    cv::Mat clone = img.clone();
    vector<FaceRect> faces = objectdetect_cnn(clone.ptr(0), clone.cols, clone.rows, clone.step);

    int min_face_limit = params.getInt("min_face_limit");
    vector<FaceRect> filter;
    for (int i = 0; i < faces.size(); ++i) {
        if (faces[i].w * min_face_limit < img.cols || faces[i].h * min_face_limit < img.rows) {
            continue;
        }
        filter.push_back(faces[i]);
//        cv::rectangle(clone, cv::Rect(faces[i].x, faces[i].y, faces[i].w, faces[i].h), cv::Scalar(0, 255, 0), 2);
    }
    printf("faces size : %d, filter size : %d\n", faces.size(), filter.size());
    if (faces.empty()) {
        result = NO_FACE;
    } else if (filter.empty()) {
        result = FACE_TOO_SMALL;
    } else {
        result = HAVE_FACE;
    }
    return filter;
}

map<float, float> detect_lines(cv::Mat &src) {
    /**
     * detect edges
     */
    int threshold = params.getInt("threshold");
    int ratio = params.getInt("ratio");
    int s = params.getInt("kernel_size");
    cv::Mat gray, blur, canny;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(s, s), 0);
    cv::Canny(blur, canny, threshold, threshold * ratio);

    /**
     * detect and draw lines
     */
    int line_threshold = params.getInt("line_threshold");

    vector<cv::Vec2f> lines;
    cv::HoughLines(canny, lines, 1, CV_PI / 6, line_threshold);

    /**
     * filter lines
     */
    map<float, float> theta_rho_map;
    for (int i = 0; i < lines.size(); ++i) {
        float rho = lines[i][0], theta = lines[i][1];
        theta_rho_map[theta] = rho;
    }
    return theta_rho_map;
}

void calculate_position(cv::Mat& img) {
    cv::Mat src = img.clone();
    map<float, float> lines = detect_lines(src);
    if (lines.size() == 0) {
        cv::pyrDown(src, src);
        lines = detect_lines(src);
        for (auto &kv:lines) {
            lines[kv.first] = kv.second * 2;
        }
    }

    for (auto &kv:lines) {
        float theta = kv.first, rho = kv.second;
        printf("rho : %f, theta : %f\n", rho, theta);
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(img, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }


}

void adjust_position() {

}

void take_photo(ros::ServiceClient &client) {
    pi_robot::SrvTrigger trigger;
    if (client.call(trigger)) {
        ROS_INFO("camera client calling service : %d", trigger.response.success);
    } else {
        ROS_INFO("camera client failed to call service");
    }
}

void process_photo(cv::Mat& img) {

}