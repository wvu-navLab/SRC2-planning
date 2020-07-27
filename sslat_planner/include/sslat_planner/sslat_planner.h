/*!
 * \waypoint_nav.h
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef SSLAT_PLANNER_H
#define SSLAT_PLANNER_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 
#include <chrono>
#include <thread>
#include <termios.h>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


class SSLatPlanner
{
public:
    SSLatPlanner(ros::NodeHandle & nh);

    void sendWaypoint();

private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber subOdom;
    ros::Subscriber subLidar;

    // Publisher
    ros::Publisher pubPose;

    geometry_msgs::Pose goalPos_;
    geometry_msgs::Pose localPos_curr_;
    geometry_msgs::Twist localVel_curr_;
    sensor_msgs::LaserScan scan_curr_;
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};


#endif // SSLAT_PLANNER_H