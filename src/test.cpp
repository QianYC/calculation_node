//
// Created by yc_qian on 20-3-14.
//
#include <iostream>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "photoComposer.hpp"

using namespace std;
ParameterReader params;

cv::Mat erode_dilate(cv::Mat src){
    int kernel_size = params.getInt("erode_size");
    int horizontal_size = src.cols / kernel_size;
    int vertical_size = src.rows / kernel_size;
    cv::Mat horizontal_structure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontal_size, 1));
    cv::Mat vertical_structure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, vertical_size));

    cv::Mat horizontal = src.clone(), vertical = src.clone(), hv = src.clone();

    cv::erode(horizontal, horizontal, horizontal_structure);
    cv::dilate(horizontal, horizontal, horizontal_structure);
    cv::imshow("horizontal", horizontal);

    cv::erode(vertical, vertical, vertical_structure);
    cv::dilate(vertical, vertical, vertical_structure);
    cv::imshow("vertical", vertical);

    cv::erode(hv, hv, horizontal_structure);
    cv::dilate(hv, hv, horizontal_structure);
    cv::erode(hv, hv, vertical_structure);
    cv::dilate(hv, hv, vertical_structure);
    cv::imshow("hv", hv);

    return hv;
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

void lines(cv::Mat src, string name = "HoughLines") {
    cv::Mat c = canny(src);
    cv::Mat res1 = src.clone(), res2 = src.clone();

    int threshold = params.getInt("line_threshold");
//    int threshold = src.rows / 5;
    int min_length = params.getInt("min_length");
    int max_gap = params.getInt("max_gap");

    double t = (double) cv::getTickCount();
    vector<cv::Vec2f> lines;
    cv::HoughLines(c, lines, 1, CV_PI / 6, threshold);
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(res1, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }
    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

//    double tp = (double) cv::getTickCount();
//    vector<cv::Vec4i> linesP;
//    cv::HoughLinesP(c, linesP, 1, CV_PI / 4, threshold, min_length, max_gap);
//    for (size_t i = 0; i < linesP.size(); i++) {
//        cv::Vec4i l = linesP[i];
//        line(res2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
//    }
//    tp = ((double) cv::getTickCount() - tp) / cv::getTickFrequency();

    cout << "HoughLines time : " << t << endl;
    cv::imshow(name, res1);
//    cv::imshow("HoughLinesP", res2);
}

void segment(cv::Mat src) {
    //change white pixel to black
    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            if (src.at<cv::Vec3b>(i, j) == cv::Vec3b(255, 255, 255)) {
                src.at<cv::Vec3b>(i, j)[0] = 0;
                src.at<cv::Vec3b>(i, j)[1] = 0;
                src.at<cv::Vec3b>(i, j)[2] = 0;
            }
        }
    }
    cv::imshow("black", src);
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
            1, 1, 1,
            1, -8, 1,
            1, 1, 1);
    cv::Mat laplace, sharp, result;
    cv::filter2D(src, laplace, CV_32F, kernel);//转换成32F是因为原来图片是8u的，经过kernel卷积可能会出现负值，会溢出
    src.convertTo(sharp, CV_32F);
    result = sharp - laplace;
    result.convertTo(result, CV_8UC3);
    laplace.convertTo(laplace, CV_8UC3);
    cv::imshow("sharp result", result);

    cv::Mat bw;
    cv::cvtColor(src, bw, CV_BGR2GRAY);
    cv::threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::imshow("binary image", bw);

    //perform distance transform algorithm
    cv::Mat dist;
    cv::distanceTransform(bw, dist, CV_DIST_L2, 3);
    //normalize the dist
    cv::normalize(dist, dist, 0, 1.0, CV_MINMAX);
    cv::imshow("distance transform", dist);
    cv::threshold(dist, dist, 0.4, 1.0, CV_THRESH_BINARY);

    //dilate a bit dist
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8U);
    cv::dilate(dist, dist, kernel1);
    cv::imshow("peak", dist);

    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);
    vector<vector<cv::Point>> contours;
    cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Mat marker = cv::Mat::zeros(dist.size(), CV_32S);
    for (int k = 0; k < contours.size(); ++k) {
        cv::drawContours(marker, contours, k, cv::Scalar(k + 1), -1);
    }
//    cv::circle(marker, cv::Point(5, 5), 3, cv::Scalar(255), -1);
    cv::imshow("marker", marker * 10000);

//    cv::Mat null = cv::Mat(marker.size(), marker.depth());
//    cv::watershed(result, null);
    cv::watershed(result, marker);
//    cv::imshow("null", null);
//    cv::imshow("marker2", marker);

//    cv::Mat mark;
//    marker.convertTo(mark, CV_8U);
//    cv::bitwise_not(mark, mark);
//    cv::imshow("mark", mark);

    vector<cv::Vec3b> colors;//为每一个轮廓随机分配一个颜色
    for (int i = 0; i < contours.size(); ++i) {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);
        colors.push_back(cv::Vec3b(b, g, r));
    }

    //create the final result of segmentation
    cv::Mat dst = cv::Mat::zeros(marker.size(), CV_8UC3);
    for (int i = 0; i < marker.rows; ++i) {
        for (int j = 0; j < marker.cols; ++j) {
            int index = marker.at<int>(i, j);
            if (index > 0 && index < contours.size()) {
                dst.at<cv::Vec3b>(i, j) = colors[index];
            }
        }
    }
    cv::imshow("final result", dst);
}

void create_image() {
    cv::Mat image(cv::Size(640, 480), CV_8UC3);
    for (int i = 0; i < image.rows / 2; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            image.at<cv::Vec3b>(i, j)[0] = 255;
            image.at<cv::Vec3b>(i, j)[1] = 255;
            image.at<cv::Vec3b>(i, j)[2] = 255;
        }
    }
    for (int i = image.rows / 2; i < image.rows; ++i) {
        for (int j = 0; j < image.cols / 2; ++j) {
            image.at<cv::Vec3b>(i, j)[0] = 255;
            image.at<cv::Vec3b>(i, j)[1] = 255;
            image.at<cv::Vec3b>(i, j)[2] = 0;
        }
        for (int j = image.cols / 2; j < image.cols; ++j) {
            image.at<cv::Vec3b>(i, j)[0] = 0;
            image.at<cv::Vec3b>(i, j)[1] = 0;
            image.at<cv::Vec3b>(i, j)[2] = 255;
        }
    }
    cv::imwrite("img4.jpg", image);
    cv::imshow("image", image);
}

void pyr(cv::Mat src) {
//    cv::Mat dst;
//    cv::pyrDown(src, dst);
//    cv::imshow("down", dst);

    int loop = 0;
    while (src.rows > 1 && src.cols > 1) {
        lines(src, "loop" + to_string(loop));
        cv::pyrDown(src, src);
//        cv::imshow("loop" + to_string(loop), src);
        loop += 1;
    }
}

int main(int argc, char **argv) {


    string filename = params.getString("input_file");
    cv::Mat src = cv::imread(filename);
    cv::resize(src, src, cv::Size(320, 240));
    cv::imshow("src", src);

//    erode_dilate(src);
//    laplace(src);
//    canny(src);
//    lines(src);
//    lines(erode_dilate(src));
//    segment(src);
//    create_image();
    pyr(src);

    MOTION motion = (MOTION) 1;
    int i = BACKWARD + 1;
    printf("motion : %d, i : %d\n", motion, i);
    
    cv::waitKey(0);
}