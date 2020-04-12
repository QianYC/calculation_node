//
// Created by yc_qian on 20-2-6.
//
#include "photoComposer.hpp"

STATE state = STATIC, last_state;
MOTION motion = STOP;
bool change_state = false;
cv::Mat image;
vector<FaceInfo> faces;
DETECT_RESULT detect_result = NO_FACE;
COMPOSE_RESULT compose_result;
CHECK_ERROR_RESULT error_result;

ros::Subscriber image_subscriber;
ros::ServiceServer state_server;
ros::ServiceClient camera_client;

void subscribe_callback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
//    ROS_INFO("thread id %d", this_thread::get_id());
    cv::Mat image = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
    switch (state) {
        case STATIC:
            if (change_state) {
                state = DYNAMIC;
                change_state = false;
                break;
            }
            faces = object_detect(image, detect_result);
            if (detect_result == HAVE_FACE) {
                state = S_OBJECT_SELECTED;
            } else {
                last_state = STATIC;
                state = ROAMING;
            }
            break;
        case S_OBJECT_SELECTED:
            /**
             * calculate expected position
             * faces -> rect, mode(single/multiple)
             */
            compose_result = calculate_expected_position(faces);
            state = S_COMPOSITION_SELECTED;
            break;
        case S_COMPOSITION_SELECTED:
            /**
             * calculate the error
             * rect, mode, image -> success, direction
             */
            error_result = calculate_error(compose_result, image);
            if (!error_result.success) {
                //mode mismatch
                break;
            } else if (error_result.direction == STOP) {
                state = S_POSITION_ADJUSTED;
                break;
            } else {
                //direction!=stop
                state = S_ADJUSTING;
                break;
            }
        case S_ADJUSTING:
            /**
             * adjust position
             */
            printf("moving %d\n", error_result.direction);
            state = S_COMPOSITION_SELECTED;
            break;
        case S_POSITION_ADJUSTED:
            ROS_INFO("S_POSITION_ADJUSTED");
            take_photo(camera_client);
            state = STATIC;
            break;
        case ROAMING:
            ROS_INFO("ROAMING");
            if (detect_result == FACE_TOO_SMALL) {
                motion = FORWARD;
                ROS_INFO("face too small, moving forward");
            } else {
                motion = (MOTION) cv::theRNG().uniform(FORWARD, RIGHT + 1);
                ROS_INFO("moving direction : %d", motion);
            }
            state = last_state;
            break;
        case DYNAMIC:
            ROS_INFO("DYNAMIC");
            if (change_state) {
                state = STATIC;
                change_state = false;
                break;
            }
            faces = object_detect(image, detect_result);
            if (detect_result == HAVE_FACE) {
                state = D_OBJECT_SELECTED;
                break;
            } else {
                state = ROAMING;
                last_state = DYNAMIC;
                break;
            }
        case D_OBJECT_SELECTED:
            ROS_INFO("D_OBJECT_SELECTED");
            take_photo(camera_client);
            state = ROAMING;
            last_state = DYNAMIC;
            break;
    }
    cv::line(image, cv::Point(0, 80), cv::Point(320, 80), cv::Scalar(0, 255, 0), 2);
    cv::line(image, cv::Point(0, 160), cv::Point(320, 160), cv::Scalar(0, 255, 0), 2);
    cv::line(image, cv::Point(106, 0), cv::Point(106, 240), cv::Scalar(0, 255, 0), 2);
    cv::line(image, cv::Point(212, 0), cv::Point(212, 240), cv::Scalar(0, 255, 0), 2);
    cv::rectangle(image, cv::Rect(66, 60, 80, 40), cv::Scalar(0, 0, 255), 2);
    cv::rectangle(image, cv::Rect(172, 60, 80, 40), cv::Scalar(0, 0, 255), 2);
    cv::rectangle(image, cv::Rect(66, 140, 80, 40), cv::Scalar(0, 0, 255), 2);
    cv::rectangle(image, cv::Rect(172, 140, 80, 40), cv::Scalar(0, 0, 255), 2);
    cv::imshow("subscriber", image);
    cv::waitKey(1);
}

bool service_change_state(pi_robot::SrvTriggerRequest &request, pi_robot::SrvTriggerResponse &response) {
//    ROS_INFO("change state server thread id : %d", this_thread::get_id());
    if ((request.state &&
         (state == DYNAMIC || state == D_OBJECT_SELECTED || (state == ROAMING && last_state == DYNAMIC)))
        || (!request.state && (state < DYNAMIC || (state == ROAMING && last_state == STATIC)))) {
        change_state = true;
    }
    response.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_subscriber");

    ros::NodeHandle nodeHandle;
    image_subscriber = nodeHandle.subscribe("/camera/image/compressed", 1, subscribe_callback);
    state_server = nodeHandle.advertiseService("/change_state", service_change_state);
    camera_client = nodeHandle.serviceClient<pi_robot::SrvTrigger>("/camera_service");

    ros::spin();
    return 0;
}