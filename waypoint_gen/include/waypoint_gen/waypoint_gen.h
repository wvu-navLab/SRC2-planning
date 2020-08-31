/*!
 * \waypoint_gen.h
 * \brief waypoint_gen (...).
 *
 * Waypoint generation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef WAYPOINT_GEN_H
#define WAYPOINT_GEN_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>


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
#include <waypoint_gen/GenerateWaypoint.h>


#define REGULAR_WP 0
#define HOMING_WP 1
#define CRATER0_WP 2
#define CRATER1_WP 3
#define CRATER2_WP 4
#define CRATER3_WP 5

class WaypointGeneration
{
public:
    WaypointGeneration(ros::NodeHandle & nh);

    bool generateWaypoint(waypoint_gen::GenerateWaypoint::Request &req, waypoint_gen::GenerateWaypoint::Response &res);
    void setupPathStart();

    bool startedOdom = false;

private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber sub_odometry_;

    // // Publisher
    // ros::Publisher pubGoalPose;

    // Service Servers
    ros::ServiceServer srv_waypoint_gen_;

    geometry_msgs::Pose localPos_curr_;
    geometry_msgs::Twist localVel_curr_;
    geometry_msgs::Pose goalPos_;
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    std::vector<double> x_, y_, type_;
    int counter_ = 0;

};


#endif // WAYPOINT_GEN_H
