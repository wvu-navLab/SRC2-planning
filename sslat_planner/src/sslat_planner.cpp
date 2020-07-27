/*!
 * \sslat_planner.cpp
 * \brief sslat_planner (...).
 *
 * Sensor Space Lattice Planner (...).
 * 
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 20, 2020
 */

#include "sslat_planner/sslat_planner.h"

SSLatPlanner::SSLatPlanner(ros::NodeHandle & nh)
    : nh_(nh)
{
    // Subscriber
    subOdom = nh_.subscribe("odometry/truth", 1, &SSLatPlanner::odometryCallback, this);
    subLidar = nh.subscribe("laser/scan",1, &SSLatPlanner::lidarCallback, this);

    // Publisher
    pubPose = nh_.advertise<geometry_msgs::Pose>("cmd_vel", 1000);
}

void SSLatPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
}

void SSLatPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_curr_ = *msg;
}

void SSLatPlanner::sendWaypoint()
{
    goalPos_.position.x = 0;
    goalPos_.position.y = 0;
    goalPos_.position.z = 0;
    goalPos_.orientation.x = 0;
    goalPos_.orientation.y = 0;
    goalPos_.orientation.z = 0;
    goalPos_.orientation.w = 0;

    // ROS_INFO_STREAM("Commanded vel" << cmd_vel);
    pubPose.publish(goalPos_);
}

/*!
 * \brief Creates and runs the waypoint_nav node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sslat_planner");
    ros::NodeHandle nh("");
    
    ros::Rate rate(300);
    
    ROS_INFO("Sensor Space Lattice Planner Node initializing...");
    SSLatPlanner sslat_planner(nh);


    while(ros::ok()) 
    {
        sslat_planner.sendWaypoint();
        ros::spinOnce();
        rate.sleep();
    }
                
    return 0;
}