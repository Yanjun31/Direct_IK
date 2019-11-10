#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "ur_kinematics/ur_kin.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <math.h>

std::vector<osrf_gear::Order> order_vector;
osrf_gear::LogicalCameraImage cameramessage;
sensor_msgs::JointState jointmessage;

int order_size = 0;

void orderCallback(const osrf_gear::Order& msg)
{ 
  order_vector.push_back(msg);
}

void cameraCallback(const osrf_gear::LogicalCameraImage& msg)
{ 
  cameramessage = msg;
}

void jointstateCallback(const sensor_msgs::JointState& msg)
{ 
  jointmessage = msg;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "lab6");
  ros::NodeHandle n;

  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::PoseStamped part_pose, goal_pose;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber sub = n.subscribe("/ariac/orders", 1, orderCallback);
  ros::Subscriber sub2 = n.subscribe("/ariac/logical_camera", 1, cameraCallback);
  ros::Subscriber sub3 = n.subscribe("/ariac/joint_states", 1, jointstateCallback);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  //ros::Publisher joint_trajectories = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 1);             
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm/follow_joint_trajectory", true);
  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

  std_srvs::Trigger begin_comp;
  osrf_gear::GetMaterialLocations get_loca;

  double T_pose[4][4], T_des[4][4];
  double q_pose[6], q_des[8][6];

  double dt = 1; //1s integration time step 
  double sample_rate = 1.0 / dt; // compute the corresponding update frequency

  trajectory_msgs::JointTrajectory joint_trajectory;

  ros::Rate naptime(sample_rate);

  begin_client.call(begin_comp);
  if (begin_comp.response.success)
  {
    ROS_WARN("Competition service gave feedback: %s", begin_comp.response.message.c_str());
  }
  else
  {
    ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  }

  order_vector.clear();
  int k;
  while (ros::ok()){  
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    if (order_vector.size()>order_size){
      order_size++;
      get_loca.request.material_type = order_vector[0].kits[0].objects[0].type;
      ROS_INFO("Orders received. The object type is: [%s]",get_loca.request.material_type.c_str());
      if (material_location_client.call(get_loca)){
          ROS_INFO("The storage locations of the material type is: [%s]",get_loca.response.storage_units[0].unit_id.c_str());
          for (k = 0; k<cameramessage.models.size(); k++){
            if (cameramessage.models[k].type == get_loca.request.material_type){
              ROS_INFO("The position of the material type is: [x = %f, y = %f, z = %f]",cameramessage.models[k].pose.position.x,cameramessage.models[k].pose.position.y,cameramessage.models[k].pose.position.z);
              ROS_INFO("The orientation of the material type is: [qx = %f, qy = %f, qz = %f, qw = %f]",cameramessage.models[k].pose.orientation.x,cameramessage.models[k].pose.orientation.y,cameramessage.models[k].pose.orientation.z,cameramessage.models[k].pose.orientation.w);
              try {
                tfStamped = tfBuffer.lookupTransform("base_link","logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
              }
              catch (tf2::TransformException &ex){
                ROS_ERROR("%s", ex.what());
              }

              q_pose[0] = jointmessage.position[1];
              q_pose[1] = jointmessage.position[2];
              q_pose[2] = jointmessage.position[3];
              q_pose[3] = jointmessage.position[4];
              q_pose[4] = jointmessage.position[5];
              q_pose[5] = jointmessage.position[6];

              part_pose.pose = cameramessage.models[k].pose;
              tf2::doTransform(part_pose, goal_pose, tfStamped);

              T_des[0][3] = goal_pose.pose.position.x;
              T_des[1][3] = goal_pose.pose.position.y;
              T_des[2][3] = goal_pose.pose.position.z + 0.1;
              T_des[3][3] = 1.0;       
              T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
              T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
              T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
              T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

              int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
              ROS_INFO("The number of solutions is: %i ",num_sols);

              joint_trajectory.header.seq = order_size - 1; // Each joint trajectory should have an incremented sequence number
              joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
              joint_trajectory.header.frame_id = "/base_link"; // Frame in which this is specified.

              // Set the names of the joints being used. All must be present.
              joint_trajectory.joint_names.clear();
              joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
              joint_trajectory.joint_names.push_back("shoulder_pan_joint");
              joint_trajectory.joint_names.push_back("shoulder_lift_joint");
              joint_trajectory.joint_names.push_back("elbow_joint");
              joint_trajectory.joint_names.push_back("wrist_1_joint");
              joint_trajectory.joint_names.push_back("wrist_2_joint");
              joint_trajectory.joint_names.push_back("wrist_3_joint");

              // Set a start and end point.
              joint_trajectory.points.resize(2);

              // Set the start point to the current position of the joints from jointmessage.
              joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
              for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++){
                for (int indz = 0; indz < jointmessage.name.size(); indz++){
                  if (joint_trajectory.joint_names[indy] == jointmessage.name[indz]){
                    joint_trajectory.points[0].positions[indy] = jointmessage.position[indz];
                    break;
                  }
                }
              }

              // When to start (immediately upon receipt).
              joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

              // Must select which of the num_sols solution to use. Just start with the first.
              int q_des_indx;
              for (q_des_indx = 0; q_des_indx < num_sols; q_des_indx++){
                if (q_des[q_des_indx][0]>M_PI/2 && q_des[q_des_indx][0]<3*M_PI/2 && q_des[q_des_indx][1]>3*M_PI/2){
                  break;
                }
              }
              ROS_INFO("q_des_indx: %i", q_des_indx);
              ROS_INFO("q0[0]: %f", q_des[0][0]);
              ROS_INFO("q1[0]: %f", q_des[1][0]);
              ROS_INFO("q2[0]: %f", q_des[2][0]);
              ROS_INFO("q3[0]: %f", q_des[3][0]);
              ROS_INFO("q4[0]: %f", q_des[4][0]);
              ROS_INFO("q5[0]: %f", q_des[5][0]);
              ROS_INFO("q6[0]: %f", q_des[6][0]);
              ROS_INFO("q7[0]: %f", q_des[7][0]);

              ROS_INFO("q0[1]: %f", q_des[0][1]);
              ROS_INFO("q1[1]: %f", q_des[1][1]);
              ROS_INFO("q2[1]: %f", q_des[2][1]);
              ROS_INFO("q3[1]: %f", q_des[3][1]);
              ROS_INFO("q4[1]: %f", q_des[4][1]);
              ROS_INFO("q5[1]: %f", q_des[5][1]);
              ROS_INFO("q6[1]: %f", q_des[6][1]);
              ROS_INFO("q7[1]: %f", q_des[7][1]);

              // Set the end point for the movement
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

              // Set the linear_arm_actuator_joint from jointmessage as it is not part of the inverse kinematics solution.
              joint_trajectory.points[1].positions[0] = jointmessage.position[1];

              // The actuators are commanded in an odd order, enter the joint positions in the correct positions
              for (int indy = 0; indy < 6; indy++) {
                joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
              }

              // How long to take for the movement.
              joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

              //joint_trajectories.publish(joint_trajectory);

              // It is possible to reuse the JointTrajectory from above
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
              joint_trajectory_as.action_goal.header.seq = order_size - 1;
              joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.header.frame_id = "/base_link";

              actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
              ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());

              break;
            }
          }
      }
      else
      {
          ROS_ERROR("Failed to call service /ariac/material_locations");
      } 
    }
    else
    {
      if (order_vector.size()<1)
      { 
      ROS_WARN("Orders haven't been received.");
      }
    }
    naptime.sleep(); // wait for remainder of specified period
  }
  return 0; // should never get here, unless roscore dies 
}
