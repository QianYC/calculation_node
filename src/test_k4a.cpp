//
// Created by yc_qian on 19-12-18.
//
#include <ros/ros.h>
#include <string>
#include <k4a/k4a.hpp>
#include "utils.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "thread_pool.hpp"

k4a_result_t getIrFrame(const k4a::capture &c, sensor_msgs::ImagePtr &);

k4a_result_t renderIrToROS(sensor_msgs::ImagePtr &, k4a::image &);

ros::Time timestampToROS(const std::chrono::microseconds &);

k4a_result_t getDepthFrame(const k4a::capture &, sensor_msgs::ImagePtr &, bool rectified = false);

k4a_result_t renderDepthToROS(sensor_msgs::ImagePtr &, k4a::image &);

k4a_result_t getRbgFrame(const k4a::capture &, sensor_msgs::ImagePtr &, bool rectified = false);

k4a_result_t renderBGRA32ToROS(sensor_msgs::ImagePtr &, k4a::image &);

cv::Size size(640, 480);

int main(int argc, char **argv){
    ros::init(argc, argv, "test_k4a");
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);

    /**
     * connect to DK
     */
    uint32_t count = k4a::device::get_installed_count();
    if (count < 1) {
        ROS_INFO("device not found!");
        return 1;
    }
    k4a::device device;
    for (uint32_t i = 0; i < count; ++i) {
        try {
            device = k4a::device::open(K4A_DEVICE_DEFAULT);
        } catch (k4a::error &e) {
            ROS_ERROR("open device %d failed!", i);
            continue;
        }

        ROS_INFO("connect to device %d succeeded!", i);
        break;
    }

    if (!device) {
        ROS_ERROR("failed to open any device!");
        return 1;
    }

    /**
     * print device info
     */
    std::string serial = device.get_serialnum();
    ROS_INFO("device serial: %s", serial.c_str());

    std::string version = hardwareInfo2String(device.get_version());
    ROS_INFO("device version: %s",version.c_str());

    /**
     * declare rgb, depth and ir publisher
     */
    image_transport::Publisher rgb_raw_pub = imageTransport.advertise("rgb/image_raw", 1);
    ros::Publisher rgb_info_pub = nodeHandle.advertise<sensor_msgs::CameraInfo>("rgb/info", 1);

    image_transport::Publisher depth_raw_pub = imageTransport.advertise("depth/image_raw", 1);
    ros::Publisher depth_info_pub = nodeHandle.advertise<sensor_msgs::CameraInfo>("depth/info", 1);

    image_transport::Publisher ir_raw_pub = imageTransport.advertise("ir/raw", 1);
    ros::Publisher ir_info_pub = nodeHandle.advertise<sensor_msgs::CameraInfo>("ir/info", 1);

    /**
     * set configuration
     */
    k4a_device_configuration_t conf = {K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                       K4A_COLOR_RESOLUTION_720P,
                                       K4A_DEPTH_MODE_NFOV_UNBINNED,
                                       K4A_FRAMES_PER_SECOND_30,
                                       true,
                                       0,
                                       K4A_WIRED_SYNC_MODE_STANDALONE,
                                       0,
                                       false};
    k4a::calibration calibration = device.get_calibration(conf.depth_mode, conf.color_resolution);

    /**
     * start the camera
     */
    ROS_INFO("starting camera");
    device.start_cameras(&conf);
    device.start_imu();

    k4a::capture capture;
    k4a_result_t result;
    ros::Rate loop_rate(conf.camera_fps);
    ros::Time capture_time;

    /**
     * the publisher loop
     */
    while (ros::ok() && !ros::isShuttingDown()) {
        if (!device.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE))) {
            ROS_FATAL("Failed to poll cameras: node cannot continue.");
            ros::requestShutdown();
            return 1;
        }

        sensor_msgs::ImagePtr rgb_raw_frame(new sensor_msgs::Image);
        sensor_msgs::ImagePtr depth_raw_frame(new sensor_msgs::Image);
        sensor_msgs::ImagePtr ir_raw_frame(new sensor_msgs::Image);

        /**
         * publish ir image iff. there is at least 1 subscriber && read ir image success
         */
        if (ir_raw_pub.getNumSubscribers() > 0 && capture.get_ir_image() != nullptr) {
            result = getIrFrame(capture, ir_raw_frame);
            if (result != K4A_RESULT_SUCCEEDED) {
                ROS_ERROR_STREAM("Failed to get raw IR frame");
                ros::shutdown();
                return 1;
            } else if (result == K4A_RESULT_SUCCEEDED) {
                capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());

                // Re-sychronize the timestamps with the capture timestamp
                ir_raw_frame->header.stamp = capture_time;
                ir_raw_frame->header.frame_id = "depth_camera";

                ir_raw_pub.publish(ir_raw_frame);
            }


        }

        /**
         * depth image not available in passive ir mode
         */
        if (calibration.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) {
            if (depth_raw_pub.getNumSubscribers() > 0 && capture.get_depth_image() != nullptr) {
                result = getDepthFrame(capture, depth_raw_frame);

                if (result != K4A_RESULT_SUCCEEDED)
                {
                    ROS_ERROR_STREAM("Failed to get raw depth frame");
                    ros::shutdown();
                    return 1;
                }
                else if (result == K4A_RESULT_SUCCEEDED)
                {
                    capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

                    // Re-sychronize the timestamps with the capture timestamp
                    depth_raw_frame->header.stamp = capture_time;
                    depth_raw_frame->header.frame_id = "depth_camera";

                    depth_raw_pub.publish(depth_raw_frame);
                }
            }
        }

        /**
         * publish the color image
         */
        if (rgb_raw_pub.getNumSubscribers() > 0 && capture.get_color_image() != nullptr) {
            result = getRbgFrame(capture, rgb_raw_frame);

            if (result != K4A_RESULT_SUCCEEDED)
            {
                ROS_ERROR_STREAM("Failed to get RGB frame");
                ros::shutdown();
                return 1;
            }

            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

            rgb_raw_frame->header.stamp = capture_time;
            rgb_raw_frame->header.frame_id = "rgb_camera";
            rgb_raw_pub.publish(rgb_raw_frame);
        }

        loop_rate.sleep();
    }

    device.stop_cameras();
    device.close();
    return 0;
}

k4a_result_t getIrFrame(const k4a::capture &capture, sensor_msgs::ImagePtr &ir_image) {

    k4a::image k4a_ir_frame = capture.get_ir_image();

    if (!k4a_ir_frame) {
        ROS_ERROR("Cannot render IR frame: no frame");
        return K4A_RESULT_FAILED;
    }

    return renderIrToROS(ir_image, k4a_ir_frame);
}

k4a_result_t renderIrToROS(sensor_msgs::ImagePtr &ir_image, k4a::image &k4a_ir_frame) {

    cv::Mat ir_buffer_mat(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_16UC1,
                          k4a_ir_frame.get_buffer());

    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();

    return K4A_RESULT_SUCCEEDED;
}

k4a_result_t getDepthFrame(const k4a::capture &capture, sensor_msgs::ImagePtr &depth_image, bool rectified) {

    k4a::image k4a_depth_frame = capture.get_depth_image();

    if (!k4a_depth_frame) {
        ROS_ERROR("Cannot render depth frame: no frame");
        return K4A_RESULT_FAILED;
    }

    return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t renderDepthToROS(sensor_msgs::ImagePtr &depth_image, k4a::image &k4a_depth_frame) {

    cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                                   k4a_depth_frame.get_buffer());
//    cv::Mat new_image(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16U);

//    depth_frame_buffer_mat.convertTo(new_image, CV_16U);
    cv::resize(depth_frame_buffer_mat, depth_frame_buffer_mat, size);

    depth_image =
            cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();

    return K4A_RESULT_SUCCEEDED;
}

k4a_result_t getRbgFrame(const k4a::capture &capture, sensor_msgs::ImagePtr &rgb_image, bool rectified) {

    k4a::image k4a_bgra_frame = capture.get_color_image();

    if (!k4a_bgra_frame) {
        ROS_ERROR("Cannot render BGRA frame: no frame");
        return K4A_RESULT_FAILED;
    }

    size_t color_image_size =
            static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) *
            sizeof(BgraPixel);

    if (k4a_bgra_frame.get_size() != color_image_size) {
        ROS_WARN("Invalid k4a_bgra_frame returned from K4A");
        return K4A_RESULT_FAILED;
    }

    return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t renderBGRA32ToROS(sensor_msgs::ImagePtr &rgb_image, k4a::image &k4a_bgra_frame) {

    cv::Mat rgb_buffer_mat(k4a_bgra_frame.get_height_pixels(), k4a_bgra_frame.get_width_pixels(), CV_8UC4,
                           k4a_bgra_frame.get_buffer());
    cv::Mat new_mat;
    cv::cvtColor(rgb_buffer_mat, new_mat, cv::COLOR_BGRA2BGR);
    cv::resize(new_mat, new_mat, size);

    rgb_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8,
                                   new_mat).toImageMsg();

    return K4A_RESULT_SUCCEEDED;
}

ros::Time start_time;

ros::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
    ros::Duration duration_since_device_startup(std::chrono::duration<double>(k4a_timestamp_us).count());

    // Set the time base if it is not set yet. Possible race condition should cause no harm.
    if (start_time.isZero())
    {
        const ros::Duration transmission_delay(0.11);
        ROS_WARN_STREAM(
                "Setting the time base using a k4a_image_t timestamp. This will result in a "
                "larger uncertainty than setting the time base using the timestamp of a k4a_imu_sample_t sample. "
                "Assuming the transmission delay to be "
                        << transmission_delay.toSec() * 1000.0 << " ms.");
        start_time = ros::Time::now() - duration_since_device_startup - transmission_delay;
    }
    return start_time + duration_since_device_startup;
}
