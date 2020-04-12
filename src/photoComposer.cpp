//
// Created by yc_qian on 20-3-15.
//
#include "photoComposer.hpp"

static ParameterReader params;
UltraFace ultraFace("../../../../model/slim-320.mnn", 320, 240, 4, 0.65);

vector<FaceInfo> object_detect(cv::Mat &img, DETECT_RESULT &result) {
//    cv::Mat clone = img.clone();
//    vector<FaceRect> faces = objectdetect_cnn(clone.ptr(0), clone.cols, clone.rows, clone.step);
    vector<FaceInfo> faces;
    ultraFace.detect(img, faces);

    int min_face_limit = params.getInt("min_face_limit");
    vector<FaceInfo> filter;
    for (auto f :faces) {
        printf("face size : [%d,%d]\n", int(f.x2 - f.x1), int(f.y2 - f.y1));
        if ((f.x2 - f.x1) * min_face_limit < img.cols && (f.y2 - f.y1) * min_face_limit < img.rows) {
            continue;
        }
        filter.push_back(f);
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
    cv::HoughLines(canny, lines, 1, CV_PI / 2, line_threshold);

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

COMPOSE_RESULT compose(cv::Mat &img, vector<FaceInfo> faces) {
    if (faces.size() == 1) {

    } else {

    }
    COMPOSE_RESULT result{.x1=faces[0].x1, .y1=faces[0].y1, .x2=faces[0].x2, .y2=faces[0].y2, .mode=SINGLE};

    /**
     * merge faces
     */
    float x1 = result.x1, y1 = result.y1, x2 = result.x2, y2 = result.y2;
    for (auto f:faces) {
        x1 = f.x1 < x1 ? f.x1 : x1;
        y1 = f.y1 < y1 ? f.y1 : y1;
        x2 = f.x2 > x2 ? f.x2 : x2;
        y2 = f.y2 > y2 ? f.y2 : y2;
    }
    result.x1 = x1;
    result.y1 = y1;
    result.x2 = x2;
    result.y2 = y2;
//    cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);

    return result;
}

COMPOSE_RESULT calculate_expected_position(vector<FaceInfo> faces) {
    COMPOSE_RESULT result;
    if (faces.size() == 1) {
        result.mode = SINGLE;
        auto f = faces[0];
        float centerX = (f.x1 + f.x2) / 2, centerY = (f.y1 + f.y2) / 2;
        result.x1 = abs(106 - centerX) < abs(212 - centerX) ? 106 : 212;
        result.y1 = abs(80 - centerY) <= abs(160 - centerY) ? 80 : 160;
    } else {
        result.mode = MULTIPLE;
        /**
         * put faces in the upper half of the image
         */
        result.x1 = 0, result.y1 = 0, result.x2 = 320, result.y2 = 120;
    }
    return result;
}

/**
 * 计算两个矩形的重叠率
 * @param obj
 * @param tgt
 * @return
 */
float overlap_rate(RECT obj, RECT tgt) {
    float x1 = max(obj.x1, tgt.x1);
    float y1 = max(obj.y1, tgt.y1);
    float x2 = min(obj.x2, tgt.x2);
    float y2 = min(obj.y2, tgt.y2);
    return ((x2 - x1) * (y2 - y1)) / ((obj.x2 - obj.x1) * (obj.y2 - obj.y1));
}

CHECK_ERROR_RESULT calculate_error(COMPOSE_RESULT &result, cv::Mat &image) {
    CHECK_ERROR_RESULT result1{.success=true};
    DETECT_RESULT detect_result;
    vector<FaceInfo> faces = object_detect(image, detect_result);
    if ((result.mode == SINGLE && faces.size() != 1) || (result.mode == MULTIPLE && faces.size() <= 1)) {
        // mode mismatch
        result1.success = false;
    } else if (result.mode == SINGLE) {
        auto f = faces[0];
        float centerX = (f.x1 + f.x2) / 2, centerY = (f.y1 + f.y2) / 2;
        float width = f.x2 - f.x1, height = f.y2 - f.y1;
        double max_face_limit = params.getDouble("max_face_limit");

        /**
         * 实际上人脸y坐标不对时应该调整相机的高度，但是目前做不到
         * 因此只能用前进后退的方式来调整，这实际上有个前提条件：目标高度不低于相机高度
         */
        if (width * max_face_limit >= image.cols || height * max_face_limit >= image.rows) {
            // face too close
            result1.direction = BACKWARD;
        } else if (centerX + 20 < result.x1) {
            // face too left, so camera move left
            result1.direction = LEFT;
        } else if (centerX - 20 > result.x1) {
            // face too right, so camera move right
            result1.direction = RIGHT;
        } else if (centerY + 20 < result.y1) {
            // face too high, so camera move back
            result1.direction = BACKWARD;
        } else if (centerY - 20 > result.y1) {
            //face too low, so camera move forward
            result1.direction = FORWARD;
        } else {
            // error small enough
            // abs(centerX - result.x1) < 20 && abs(centerY - result.y1) < 15
            result1.direction = STOP;
        }
    } else {
        // multiple mode
        // merge faces
        float x1 = faces[0].x1, y1 = faces[0].y1, x2 = faces[0].x2, y2 = faces[0].y2;
        for (auto f:faces) {
            x1 = f.x1 < x1 ? f.x1 : x1;
            y1 = f.y1 < y1 ? f.y1 : y1;
            x2 = f.x2 > x2 ? f.x2 : x2;
            y2 = f.y2 > y2 ? f.y2 : y2;
        }
        RECT obj{.x1=x1, .y1=y1, .x2=x2, .y2=y2};
        RECT tgt{.x1=result.x1, y1 = result.y1, .x2=result.x2, .y2=result.y2};
        if (overlap_rate(obj, tgt) >= 0.9) {
            // in the right place
            result1.direction = STOP;
        } else if (x1 < result.x1) {
            // faces too left, camera move left
            result1.direction = LEFT;
        } else if (x2 > result.x2) {
            // faces too right, camera move right
            result1.direction = RIGHT;
        } else if (y1 < result.y1) {
            //faces too high, camera move back
            result1.direction = BACKWARD;
        } else if (y2 > result.y2) {
            //faces too low, camera move forward
            result1.direction = FORWARD;
        } else {
            result1.direction = STOP;
        }
    }
    return result1;
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