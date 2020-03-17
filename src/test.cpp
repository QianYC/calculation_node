//
// Created by yc_qian on 20-3-14.
//
#include <iostream>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "photoComposer.hpp"

using namespace std;
ParameterReader params;

void erode_dilate(cv::Mat src){
    int kernel_size = params.getInt("kernel_size");
    int horizontal_size = src.cols / kernel_size;
    int vertical_size = src.rows / kernel_size;
    cv::Mat horizontal_structure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontal_size, 1));
    cv::Mat vertical_structure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, vertical_size));

    cv::Mat horizontal = src.clone(), vertical = src.clone();

    cv::erode(horizontal, horizontal, horizontal_structure);
    cv::dilate(horizontal, horizontal, horizontal_structure);
    cv::imshow("horizontal", horizontal);

    cv::erode(vertical, vertical, vertical_structure);
    cv::dilate(vertical, vertical, vertical_structure);
    cv::imshow("vertical", vertical);
}

void laplace(cv::Mat src) {
    cv::Mat blur;
    cv::GaussianBlur(src, blur, cv::Size(3, 3), 0);
    cv::Mat gray;
    cv::cvtColor(blur, gray, CV_BGR2GRAY);
    cv::Mat laplace;
    cv::Laplacian(gray, laplace, CV_16S, 3);
    cv::Mat result;
    cv::convertScaleAbs(laplace, result);
    cv::imshow("laplace", result);
}

cv::Mat canny(cv::Mat src) {
    int threshold = params.getInt("threshold");
    int ratio = params.getInt("ratio");
    int s = params.getInt("kernel_size");
    cv::Mat gray, blur, canny;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(s, s), 0);
    cv::Canny(blur, canny, threshold, threshold * ratio);
    cv::imshow("canny", canny);
    return canny;
}

void lines(cv::Mat src) {
    cv::Mat c = canny(src);
    cv::Mat res1 = src.clone(), res2 = src.clone();

    int threshold = params.getInt("line_threshold");
    int min_length = params.getInt("min_length");
    int max_gap = params.getInt("max_gap");

    double t = (double) cv::getTickCount();
    vector<cv::Vec2f> lines;
    cv::HoughLines(c, lines, 1, CV_PI / 18, threshold);
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
        line( res1, pt1, pt2, cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }
    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

    double tp = (double) cv::getTickCount();
    vector<cv::Vec4i> linesP;
    cv::HoughLinesP(c, linesP, 1, CV_PI / 18, threshold, min_length, max_gap);
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        line( res2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }
    tp = ((double) cv::getTickCount() - tp) / cv::getTickFrequency();

    cout << "HoughLines time : " << t << ", HoughLinesP time : " << tp << endl;
    cv::imshow("HoughLines", res1);
    cv::imshow("HoughLinesP", res2);
}

int main(int argc, char **argv) {


    string filename = params.getString("input_file");
    cv::Mat src = cv::imread(filename);
    cv::resize(src, src, cv::Size(320, 240));
    cv::imshow("src", src);

//    erode_dilate(src);
//    laplace(src);
//    canny(src);
    lines(src);
    cv::waitKey(0);
}