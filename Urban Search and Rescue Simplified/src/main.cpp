
#include "../include/bot.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  //intialise the object of class bot
  project::Bot object(&nh);

  // get the location to be visited by explorer from the parameter server to search for Aurco marker
  object.get_position();

  // set the last elemtents of the m_array to be home position of follower
  object.m_array.at(4).first = -4.0;
  object.m_array.at(4).second = 3.5;

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  //Build goal for explorer. X and Y position of the goal will be updated in later.
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.orientation.w = 1.0;


  //Build goal for follower. X and Y position of the goal will be updated in later.
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.orientation.w = 1.0;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok()) 
  {
    while(!object.m_position_stack.empty())  // this while loop will run till explorer visit all the location and return home position
    {
      // set the x and y position of explorer's goal
      object.m_temp_pos = object.m_position_stack.top();
      explorer_goal.target_pose.pose.position.x = object.m_temp_pos.first;
      explorer_goal.target_pose.pose.position.y = object.m_temp_pos.second;

      if (!object.m_explorer_goal_sent)
      {
        ROS_INFO_STREAM("explorer is moving to AuRco marker detection location");
        //send goal to move_base client server. Move_base will drive robot from current location to goal position 
        explorer_client.sendGoal(explorer_goal);  
        object.m_explorer_goal_sent = true;
        object.m_position_stack.pop();
      }

      //enter the loop when explorer reaches to the AuRco marker detection location
      if(explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
      {
        ROS_INFO_STREAM("explorer has reached to AuRco marker detection location");
        ROS_INFO_STREAM("explorer has started detecting AuRco marker");
        object.m_start_detection = true;  // this boolean is set as condition to start the transformation of the AuRco frame in camera frame
        ROS_INFO_STREAM("explorer is rotating to detect AuRco marker");

        while (true)
        {
          // start rotating the robot once it reach to AuRco marker detection location
          object.rotate_bot();
          // start broadcaster and listner
          ros::spinOnce();
          object.listen(tfBuffer);
          //stop rotating when AuRco marker is detected

          if(object.m_aurco_detect==true )
          {
            ROS_INFO_STREAM("explorer has stopped rotating");
            break;
          }

        }
        // when explorer detect the AuRco marker set the flag to flase to stop detection
        object.m_start_detection = false;
        object.m_aurco_detect=false;
        // when explorer reached the goal set the flag to flase to take new goal position
        object.m_explorer_goal_sent = false;
      }
    }

    ROS_INFO_STREAM("explorer has visted all the AuRco marker detection location and come back to home position");
    //once the explorer checked all the AuRco marker and come back to home position, send follower to visit the AuRco marker location
    if((object.m_position_stack.empty()) && (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ) 
    {
      while(object.m_i<5)
      {
        // set the x and y position of follower's goal
        follower_goal.target_pose.pose.position.x = object.m_array.at(object.m_i).first;
        follower_goal.target_pose.pose.position.y = object.m_array.at(object.m_i).second;
      
        if (!object.m_follower_goal_sent)     
        {
          //send goal to move_base client server. Move_base will drive robot from current location to goal position
          follower_client.sendGoal(follower_goal);
          ROS_INFO_STREAM("explorer is moving to AuRco marker location");
          object.m_follower_goal_sent = true;
        }
        if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
          ROS_INFO_STREAM("explorer has reached to AuRco marker location");
          // when follower reached the goal set the flag to flase to take new goal position
          object.m_follower_goal_sent = false;
          ++object.m_i;
        }
      }
      //when follower visit all the AuRcolocation and come back to home, shutdown the ROS
      if (object.m_i==5 && follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO_STREAM("follower has visted all the AuRco marker detection location and come back to home position");
        ROS_INFO_STREAM("task accomplished");
        ROS_INFO_STREAM("ROS shutown");
        ros::shutdown();
      }
    }

    loop_rate.sleep();
  }
}