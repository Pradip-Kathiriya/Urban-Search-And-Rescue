#include "../include/bot.h"
#include <geometry_msgs/Quaternion.h>


project::Bot::Bot(ros::NodeHandle* nodehandle ) :
    m_nh{*nodehandle},
    m_aurco_detect { false },
    m_start_detection { false },
    m_explorer_goal_sent { false },
    m_follower_goal_sent { false },
    m_i {0}
{
    initialise_subscribers();
    initialise_publisher();
}


void project::Bot::initialise_publisher()
{   //publisher to publish command on the topic /cmd_vel
    m_pub = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1);
}


void project::Bot::initialise_subscribers()
{
    //subscriber to subscribe the topic/fiducial_transforms
    m_sub_msgs = m_nh.subscribe("fiducial_transforms", 1, &project::Bot::fiducial_callback,this);
}


void project::Bot::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msgs)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame"; //name of the frame

    if (!msgs->transforms.empty() && m_start_detection) //check marker is detected
    {
        //broadcaster object
        //broadcast the new frame to /tf Topic
        
        transformStamped.transform.translation.x = msgs->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = msgs->transforms[0].transform.translation.y;
        transformStamped.transform.translation.z = (msgs->transforms[0].transform.translation.z-0.5); // set the (z to z-0.5) mm as marker is on wall and follower can't go there. 
        transformStamped.transform.rotation.x = msgs->transforms[0].transform.rotation.x; 
        transformStamped.transform.rotation.y = msgs->transforms[0].transform.rotation.y;
        transformStamped.transform.rotation.z = msgs->transforms[0].transform.rotation.z;
        transformStamped.transform.rotation.w = msgs->transforms[0].transform.rotation.w;
        m_fiducial_id=msgs->transforms[0].fiducial_id; // stored the fiducial ID of the detected marker 
        ROS_INFO_STREAM("AuRco marker detected");
        br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
    }
}


void project::Bot::listen(tf2_ros::Buffer& tfBuffer)
{
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
        // transform the marker_frame into map_frame
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0), ros::Duration(4.0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        auto trans_z = transformStamped.transform.translation.z;

        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << ","
        << trans_z << "]"
        );
         // update the m_array as per the fiducial ID
        if(m_fiducial_id==0)
        {
            ROS_INFO_STREAM("fiducial ID of the AuRco marker is 0");
            m_array.at(m_fiducial_id).first=trans_x;
            m_array.at(m_fiducial_id).second=trans_y;
            ROS_INFO_STREAM("marker location is stored in array for the follower");
            m_aurco_detect = true;
        }

        if(m_fiducial_id==1)
        {
            ROS_INFO_STREAM("fiducial ID of the AuRco marker is 1");
            m_array.at(m_fiducial_id).first=trans_x;
            m_array.at(m_fiducial_id).second=trans_y;
            ROS_INFO_STREAM("marker location is stored in array for the follower");
            m_aurco_detect = true;
        
        }

        if(m_fiducial_id==2)
        {
            ROS_INFO_STREAM("fiducial ID of the AuRco marker is 2");;
            m_array.at(m_fiducial_id).first=trans_x;
            m_array.at(m_fiducial_id).second=trans_y;
            ROS_INFO_STREAM("marker location is stored in array for the follower");
            m_aurco_detect = true;

        }

        if(m_fiducial_id==3)
        {
            ROS_INFO_STREAM("fiducial ID of the AuRco marker is 3");
            m_array.at(m_fiducial_id).first=trans_x;
            m_array.at(m_fiducial_id).second=trans_y;
            ROS_INFO_STREAM("marker location is stored in array for the follower");
            m_aurco_detect = true;
        }
    }
    catch (tf2::TransformException& ex) 
    {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    }
}

void project::Bot::get_position()
{
    // retrieve the AuRco marker detection location from the parameter server and stored it into pair
    XmlRpc::XmlRpcValue position1;
    m_nh.getParam("/aruco_lookup_locations/target_1", position1);


    XmlRpc::XmlRpcValue position2;
    m_nh.getParam("/aruco_lookup_locations/target_2", position2);


    XmlRpc::XmlRpcValue position3;
    m_nh.getParam("/aruco_lookup_locations/target_3", position3);


    XmlRpc::XmlRpcValue position4;
    m_nh.getParam("/aruco_lookup_locations/target_4", position4);

    m_position_1.first = position1[0];
    m_position_1.second = position1[1];

    m_position_2.first = position2[0];
    m_position_2.second = position2[1];

    m_position_3.first = position3[0];
    m_position_3.second = position3[1];

    m_position_4.first = position4[0];
    m_position_4.second = position4[1];

    // set the 5th elements of the stack as a home position of the explorer
    m_position_5.first = -4;
    m_position_5.second = 2.5;

    m_position_stack.push(m_position_5);
    m_position_stack.push(m_position_4);
    m_position_stack.push(m_position_3);
    m_position_stack.push(m_position_2);
    m_position_stack.push(m_position_1);

}

// this function will publish constant angular velocity of 0.2 rad/s to topic /cmd_vel
void project::Bot::rotate_bot()
{
    // create and object of geometry_msgs type
    geometry_msgs::Twist msg;
    //set angular velocity to 0.2
    msg.angular.z = 0.2;
    // publish the command ot topic /cmd_vel
    m_pub.publish(msg);
}
