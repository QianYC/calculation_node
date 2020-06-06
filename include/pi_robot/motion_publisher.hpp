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
        //已略去
    }
public:
    //构造函数已略去
    //控制机器人朝指定方向运动
    void move(MOTION motion) {
        geometry_msgs::Twist msg = motion2twist(motion);
        pool.submit([&] {
            publisher.publish(msg);
        });
    }

    //控制机器人朝指定方向运动一段时间
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
