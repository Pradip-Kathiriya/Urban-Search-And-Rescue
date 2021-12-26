/**
 * @file bot.h
 * @author Jeffin Johny K (jeffinjk@umd.edu) Pradip(pradip@umd.edu) Hemanth(hemanth1@umd.edu)
 * @brief This file contains the class 'bot'
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef BOT_H
#define BOT_H

#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>    //for nav_msgs::Odometry
#include <ros/ros.h>
#include <tf/transform_datatypes.h> 
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <list>
#include <iostream>
#include <iterator>
#include <stack>
#include <array>
#include <vector>
#include <utility>

namespace project
{
    /**
     * @brief 
     * 
     */
    class Bot
    {
        public:

            Bot(ros::NodeHandle* nodehandle );
            bool m_aurco_detect ; // boolean to set flag to move to next detection location when AuRco marker detected.
            bool m_start_detection ; // boolean to set flag to start detection when follower reach to detection location
            bool m_explorer_goal_sent; // boolean to set flag to send next explorer goal position to move_base client server.
            bool m_follower_goal_sent; // boolean to set flag to send next follower goal position to move_base client server.
            std::stack<std::pair<double,double>>m_position_stack; // stack to store the explorer vising location reteived from parameter server
            std::array<std::pair<double,double>,5>m_array; // array to store the location of the AuRco marker as per fidicual ID
            std::pair<double,double>m_temp_pos; // temporary pair
            int m_i; // flag to exit follower while loop
            /**
             * @brief this function will retrieve the AuRco marker detection location from the parameter server and stored it into a stack. 
             * explorer will then use this stack to visit each of these locations one by one to detect AuRco marker
             */
            void get_position();

            /**
             * @brief this function will transform the marker frame published by fiducial_callback function on the topic /tf to the /map  
             * frame. this function will then stored this location into the m_array as per fiducial_id of the AuRco marker.
             * @param tfBuffer 
             */
            void listen(tf2_ros::Buffer& tfBuffer);
            
            /**
             * @brief this function will rotate the robot at current location with the constant angular velocity of 0.2 rd/sec 
             * 
             */
            void rotate_bot();

        private:

            int m_fiducial_id;
            ros::NodeHandle m_nh;  // declaration of NodeHandle object
            ros::Publisher m_pub;   // declaration of publisher object
            ros::Subscriber m_sub_msgs; // declaration of subscriber object
            std::pair<double,double>m_position_1; // declaration of pair to retrive the AuRco detection location from the parameter server.
            std::pair<double,double>m_position_2; // declaration of pair to retrive the AuRco detection location from the parameter server.
            std::pair<double,double>m_position_3; // declaration of pair to retrive the AuRco detection location from the parameter server.
            std::pair<double,double>m_position_4; // declaration of pair to retrive the AuRco detection location from the parameter server.
            std::pair<double,double>m_position_5; // declaration of pair to store the start position of te explorer
            /**
             * @brief this is call back function of the subscriber initialised by initialise_subscribers function. it 
             * will create a marker_frame as a child of /explorer_tf/camera_rgb_optical_frame and publish to the topic /tf
             * @param msgs 
             */
            void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msgs);
            
            /**
             * @brief this function will initialise the subsriber to the topic /fiducial_tranforms. the call back function of this subscriber
             * will create a marker_frame as a child of /explorer_tf/camera_rgb_optical_frame and publish to the topic /tf
             */
            void initialise_subscribers();
            
            /**
             * @brief this function initialise the publisher whill will publish angular velocity on the topic /cmd_vel 
             * 
             */
            void initialise_publisher();

    };
}
#endif