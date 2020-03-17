//
// Created by yc_qian on 20-3-15.
//
#include "photoComposer.hpp"

static ParameterReader params;

void compose(cv::Mat img) {
    vector<FaceRect> faces = object_detect(img);
    calculate_position(img, faces);
    adjust_position();
    take_photo();
}

vector<FaceRect> object_detect(cv::Mat img) {
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
    return filter;
}

void calculate_position(cv::Mat img, vector<FaceRect> faces) {
    /**
     * detect edges
     */
    int threshold = params.getInt("threshold");
    int ratio = params.getInt("ratio");
    int s = params.getInt("kernel_size");
    cv::Mat gray, blur, canny;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(s, s), 0);
    cv::Canny(blur, canny, threshold, threshold * ratio);

    /**
     * detect and draw lines
     */
    int line_threshold = params.getInt("line_threshold");
    int min_length = params.getInt("min_length");
    int max_gap = params.getInt("max_gap");

    vector<cv::Vec2f> lines;
    cv::HoughLines(canny, lines, 1, CV_PI / 4, line_threshold);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( img, pt1, pt2, cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }
}

void adjust_position() {

}

void take_photo() {

}

void process_photo(cv::Mat img) {

}