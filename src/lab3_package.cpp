#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

std::vector<osrf_gear::Order> order_vector;
osrf_gear::LogicalCameraImage cameramessage;
int order_size = 0;

void orderCallback(const osrf_gear::Order& msg)
{ 
  order_vector.push_back(msg);
}

void cameraCallback(const osrf_gear::LogicalCameraImage& msg)
{ 
  cameramessage = msg;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "lab3");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::PoseStamped part_pose, goal_pose;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::MoveGroupInterface::Plan the_plan;

  ros::Subscriber sub = n.subscribe("/ariac/orders", 1, orderCallback);
  ros::Subscriber sub2 = n.subscribe("/ariac/logical_camera", 1, cameraCallback);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  std_srvs::Trigger begin_comp;
  osrf_gear::GetMaterialLocations get_loca;

  double dt = 1; //10ms integration time step 
  double sample_rate = 1.0 / dt; // compute the corresponding update frequency 
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
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  order_vector.clear();
  int k;
  while (ros::ok()) {  
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //ros::spinOnce(); //allow data update from callback
    if (order_vector.size()>order_size)
    {
      order_size++;
      get_loca.request.material_type = order_vector[0].kits[0].objects[0].type;
      ROS_INFO("Orders received. The object type is: [%s]",get_loca.request.material_type.c_str());
      if (material_location_client.call(get_loca)) {
          ROS_INFO("The storage locations of the material type is: [%s]",get_loca.response.storage_units[0].unit_id.c_str());
          for (k = 0; k<cameramessage.models.size(); k++)
          {
            //ROS_INFO("We find %s",cameramessage.models[k].type.c_str());
            if (cameramessage.models[k].type == get_loca.request.material_type)
            {
              ROS_INFO("The position of the material type is: [x = %f, y = %f, z = %f]",cameramessage.models[k].pose.position.x,cameramessage.models[k].pose.position.y,cameramessage.models[k].pose.position.z);
              ROS_INFO("The orientation of the material type is: [qx = %f, qy = %f, qz = %f, qw = %f]",cameramessage.models[k].pose.orientation.x,cameramessage.models[k].pose.orientation.y,cameramessage.models[k].pose.orientation.z,cameramessage.models[k].pose.orientation.w);
              try {
                tfStamped = tfBuffer.lookupTransform("world","logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
                //tfStamped = tfBuffer.lookupTransform(move_group.getPlanningFrame().c_str(),"logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
                //ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
              }
              catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
              }
              part_pose.pose = cameramessage.models[k].pose;
              tf2::doTransform(part_pose, goal_pose, tfStamped);
              goal_pose.pose.position.z += 0.10; // 10 cm above the part
              goal_pose.pose.orientation.w = 0.707;
	      goal_pose.pose.orientation.x = 0.0;
              goal_pose.pose.orientation.y = 0.707;
              goal_pose.pose.orientation.z = 0.0;
	      move_group.setPoseTarget(goal_pose);
              if (move_group.plan(the_plan))
              {
                ROS_INFO("The plan has been successfully generated!");
                while(!move_group.execute(the_plan)){
                }
              }
              else{
                ROS_WARN("Fail to generate the plan!");
              }
              
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
