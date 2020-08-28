/*!
 * \waypoint_gen.cpp
 * \brief waypoint_gen (...).
 *
 * Waypoint generation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 *
 * \date June 20, 2020
 */

#include "waypoint_gen/waypoint_gen.h"

WaypointGeneration::WaypointGeneration(ros::NodeHandle & nh)
    : nh_(nh)
{
    // Subscriber
    sub_odometry_ = nh_.subscribe("localization/odometry/sensor_fusion", 1, &WaypointGeneration::odometryCallback, this);

    // // Publisher
    // pubGoalPose = nh_.advertise<geometry_msgs::Pose>("navigation/goal_pos", 1000);

    // Service Servers
    srv_waypoint_gen_ = nh_.advertiseService("navigation/generate_goal", &WaypointGeneration::generateWaypoint, this);

    if (!nh_.hasParam("/waypoints/"))
    {
        ROS_INFO("No waypoints found.");
    }
    else
    {
        ROS_INFO("Waypoints loaded.");
    }

    if (nh_.getParam("/start_index", counter_)){
	  
	ROS_INFO("Found Start Index ");
      }else{
	counter_ =0;
    }


    nh_.getParam("/waypoints/x", x_);
    nh_.getParam("/waypoints/y", y_);

    ROS_INFO_STREAM("x size: " << x_.size());
    ROS_INFO_STREAM("y size: " << y_.size());
}

void WaypointGeneration::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
    // ROS_INFO_STREAM("odometry"<<localPos_curr_);
}


void WaypointGeneration::setupPathStart()
{
    double min_distance = 200;

    double ex, ey, distance;
    if(counter_==0){
    	for (int i = 0; i < x_.size(); ++i)
    	{
        // ROS_INFO_STREAM("x: " << localPos_curr_.position.x);
        // ROS_INFO_STREAM("y: " << localPos_curr_.position.y);
        ex = x_[i] - localPos_curr_.position.x;
        ey = y_[i] - localPos_curr_.position.y;

        distance = std::hypot(ex, ey);
        // ROS_INFO_STREAM("Distance: " << distance);
        if (distance < min_distance)
        {
            min_distance = distance;
            counter_ = i;
            ROS_INFO_STREAM("Distance: " << min_distance);
        }
   	 }
    }

    ROS_INFO_STREAM("Starting from index: " << counter_);

    goalPos_.position.x = x_[counter_];
    goalPos_.position.y = y_[counter_];
    goalPos_.position.z = 0;
    goalPos_.orientation.w = 0.0;
    goalPos_.orientation.x = 0.0;
    goalPos_.orientation.y = 0.0;
    goalPos_.orientation.z = 0.0;

    // ROS_INFO_STREAM("New goal pose" << goalPos_);
    // pubGoalPose.publish(goalPos_);
}

bool WaypointGeneration::generateWaypoint(waypoint_gen::GenerateWaypoint::Request &req, waypoint_gen::GenerateWaypoint::Response &res)
{
    if (req.start == true)
    {
        if (req.next == true)
        {
            ++counter_;
        }

        double ex, ey, distance;

        ex = x_[counter_] - localPos_curr_.position.x;
        ey = y_[counter_] - localPos_curr_.position.y;

        distance = std::hypot(ex, ey);

        if (distance < 2.0)
        {
            (++counter_ > (x_.size()-1))? counter_ = 0 : counter_;

            goalPos_.position.x = x_[counter_];
            goalPos_.position.y = y_[counter_];

            ROS_INFO_STREAM("New goal pose" << goalPos_);
            // pubGoalPose.publish(goalPos_);
            res.goal = goalPos_;
            res.success = true;
        }
        else
        {
            goalPos_.position.x = x_[counter_];
            goalPos_.position.y = y_[counter_];

            // ROS_INFO_STREAM("New goal pose" << goalPos_);
            // pubGoalPose.publish(goalPos_);
            res.goal = goalPos_;
            res.success = true;
        }

    }
    else
    {
        res.goal = localPos_curr_;
        res.success = false;
    }
    return true;
}

/*!
 * \brief Creates and runs the waypoint_gen node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_gen");
    ros::NodeHandle nh("");

    ros::Rate rate(10);

    ROS_INFO("Waypoint Gen Node initializing...");
    WaypointGeneration waypoint_gen(nh);

    while(!waypoint_gen.startedOdom)
    {
        ros::spinOnce();
        rate.sleep();
    }

    waypoint_gen.setupPathStart();

    // while(ros::ok())
    // {
    //     waypoint_gen.generateWaypoint();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    ros::spin();

    return 0;
}
