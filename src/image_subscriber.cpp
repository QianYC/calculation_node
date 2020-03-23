//
// Created by yc_qian on 20-2-6.
//
#include "photoComposer.hpp"

STATE state = STATIC, last_state;
MOTION motion = STOP;
bool change_state = false;
cv::Mat image;
vector<FaceRect> faces;
DETECT_RESULT detect_result = NO_FACE;
COMPOSE_RESULT compose_result;

ros::Subscriber image_subscriber;
ros::ServiceServer state_server;
ros::ServiceClient camera_client;

void img_loop(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    switch (state) {
        case STATIC:
            ROS_INFO("STATIC");
            if (change_state) {
                state = DYNAMIC;
                change_state = false;
                break;
            }
            image = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
            faces = object_detect(image, detect_result);
            if (faces.size() > 0) {
                state = S_OBJECT_SELECTED;
            } else {
                last_state = state;
                state = ROAMING;
            }
            break;
        case S_OBJECT_SELECTED:
            ROS_INFO("S_OBJECT_SELECTED");
            compose_result = compose(image, faces);
            state = S_COMPOSITION_SELECTED;
            break;
        case S_COMPOSITION_SELECTED:
            ROS_INFO("S_COMPOSITION_SELECTED");
            adjust_position();
            state = S_POSITION_ADJUSTED;
            break;
        case S_POSITION_ADJUSTED:
            ROS_INFO("S_POSITION_ADJUSTED");
            take_photo(camera_client);
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
            faces = object_detect(image, detect_result);
            if (faces.size() > 0) {
                state = D_OBJECT_SELECTED;
            } else {

                last_state = state;
                state = ROAMING;
            }
            break;
        case D_OBJECT_SELECTED:
            ROS_INFO("D_OBJECT_SELECTED");
            take_photo(camera_client);
            last_state = DYNAMIC;
            state = ROAMING;
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
        default:
            break;
    }
    if (!image.empty()) {
        cv::imshow("subscriber", image);
        cv::waitKey(1);
    }
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
    image_subscriber = nodeHandle.subscribe("/camera/image/compressed", 1, img_loop);
    state_server = nodeHandle.advertiseService("/change_state", service_change_state);
    camera_client = nodeHandle.serviceClient<pi_robot::SrvTrigger>("/camera_service");

    ros::spin();
    return 0;
}