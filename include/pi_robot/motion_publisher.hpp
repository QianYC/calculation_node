//
// Created by yc_qian on 20-4-18.
//

#ifndef PI_ROBOT_MOTION_PUBLISHER_HPP
#define PI_ROBOT_MOTION_PUBLISHER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "thread_pool.hpp"
#include "robotBase.hpp"

class MotionPublisher {
private:
    ros::Publisher publisher;
    std::string topic_name;
    ThreadPool pool;

    //把motion转换成ROS中控制机器人运动的消息类型
    geometry_msgs::Twist motion2twist(MOTION motion) {
        geometry_msgs::Twist msg;
        switch (motion) {
            case FORWARD:
                msg.linear.x = 0.1;
                msg.angular.z = 0;
                break;
            case BACKWARD:
                msg.linear.x = -0.1;
                msg.angular.z = 0;
                break;
            case LEFT:
                msg.linear.x = 0;
                msg.angular.z = 0.1;
                break;
            case RIGHT:
                msg.linear.x = 0;
                msg.angular.z = -0.1;
                break;
            default:
                msg.linear.x = 0;
                msg.angular.z = 0;
        }
        return msg;
    }
public:
    MotionPublisher() : pool(1) {}

    MotionPublisher(std::string topic_name, ros::NodeHandle nodeHandle) 
        : topic_name(topic_name), pool(1) {
        publisher = nodeHandle.advertise<geometry_msgs::Twist>(topic_name, 1);
    }

    MotionPublisher &operator=(const MotionPublisher &motionPublisher) {
        publisher = std::move(motionPublisher.publisher);
        topic_name = motionPublisher.topic_name;
    }

    void move(MOTION motion) {
        geometry_msgs::Twist msg = motion2twist(motion);
        pool.submit([&] {
            publisher.publish(msg);
        });
    }

    void move_for(MOTION motion, int millisec) {
        geometry_msgs::Twist move = motion2twist(motion);
        geometry_msgs::Twist stop = motion2twist(STOP);
        pool.submit([&] {
            publisher.publish(move);
            std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
            publisher.publish(stop);
        });
    }
};
#endif //PI_ROBOT_MOTION_PUBLISHER_HPP
