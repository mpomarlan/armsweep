#include "ros/ros.h"
#include <signal.h>
#include <armsweep/armsweep.h>

#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <cstdlib>
#include <iostream>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_listener.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <armsweep/armsweep.h>
#include <armsweep/armsweep_internal.h>

#define KEYPAUSE() std::cin.get()

#define SCENE_TOPIC "planning_scene"
#define ROBOT_DESCRIPTION "robot_description"

void onQuit(int sig)
{
  ros::NodeHandle node_handle;
  ros::ServiceClient client_UnloadMesh = node_handle.serviceClient<armsweep::UnloadMesh>("UnloadMesh");
  armsweep::UnloadMesh srv_UnloadMesh;
  ROS_INFO("+++Unloading test meshes.");
  srv_UnloadMesh.request.meshNames.clear(); 
  {
      srv_UnloadMesh.request.meshNames.push_back("chop_fw");
      srv_UnloadMesh.request.meshNames.push_back("chop_in");
      srv_UnloadMesh.request.meshNames.push_back("chop_out");
      srv_UnloadMesh.request.meshNames.push_back("swerve_in");
      srv_UnloadMesh.request.meshNames.push_back("swerve_out");
  }
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
  }

  ros::shutdown();
}

int main(int argc, char **argv)
{

  //init ROS node
  ros::init (argc, argv, "pickhax");
  ros::WallDuration sleep_t(0.5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Started pickhax.");

  ROS_INFO("Getting robot model.");
  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  robot_model::RobotModel model_copy(*((robot_model_loader.getModel()).get()));

  robot_model::RobotModelConstPtr kinematic_model(&model_copy);

  ROS_INFO("Setting up a copy of the robot's planning scene: ");
  ROS_INFO("    Getting a transform listener.");
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  ROS_INFO("    Starting a planning scene monitor.");
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  psm.startStateMonitor();
  psm.startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC);
  psm.startSceneMonitor(SCENE_TOPIC);
  ROS_INFO("    Creating a planning scene object.");
  planning_scene::PlanningScenePtr robotPlanningScene(psm.getPlanningScene());
  ROS_INFO("    Requesting current planning scene state ... %d.", psm.requestPlanningSceneState("/get_planning_scene"));
  sleep_t.sleep();
  ROS_INFO("    Created a planning scene object and updated it with current state.");

  ROS_INFO("Setting up a collision world for the trial meshes:");
  collision_detection::WorldPtr trialWorld(new collision_detection::World());
  collision_detection::CollisionWorldFCL trialCollisionWorld(trialWorld);
  ROS_INFO("    Created trial collision world.");

  ROS_INFO("Setting up a publisher for planning scene diffs: used to modify the planning scene by adding/removing objects, for debug/visualization purposes.");
  ros::Publisher planning_scene_diff_publisher;// = node_handle.advertise<moveit_msgs::PlanningScene>(SCENE_TOPIC, 1);
  ros::Publisher collision_object_publisher;// = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  //while((planning_scene_diff_publisher.getNumSubscribers() < 1) || (collision_object_publisher.getNumSubscribers() < 1))
  //{
  //  sleep_t.sleep();
  //}
  //ROS_INFO("Advertized production of planning_scene_diff and collision_object messages.");


  armsweep::ArmsweepContext armsweepContext(kinematic_model, robotPlanningScene, trialWorld, trialCollisionWorld, planning_scene_diff_publisher, collision_object_publisher);

  ros::NodeHandle node_handle;

  ros::ServiceClient client_LoadMesh = node_handle.serviceClient<armsweep::LoadMesh>("LoadMesh");
  ros::ServiceClient client_ListMesh = node_handle.serviceClient<armsweep::ListMesh>("ListMesh");
  ros::ServiceClient client_UnloadMesh = node_handle.serviceClient<armsweep::UnloadMesh>("UnloadMesh");
  ros::ServiceClient client_CollideMesh = node_handle.serviceClient<armsweep::CollideMesh>("CollideMesh");
  ros::ServiceClient client_VisualizeMesh = node_handle.serviceClient<armsweep::VisualizeMesh>("VisualizeMesh");
  ros::ServiceClient client_UnvisualizeMesh = node_handle.serviceClient<armsweep::UnvisualizeMesh>("UnvisualizeMesh");

  armsweep::LoadMesh srv_LoadMesh;
  armsweep::ListMesh srv_ListMesh;
  armsweep::UnloadMesh srv_UnloadMesh;
  armsweep::CollideMesh srv_CollideMesh;
  armsweep::VisualizeMesh srv_VisualizeMesh;
  armsweep::UnvisualizeMesh srv_UnvisualizeMesh;

  sleep_t.sleep();
  ROS_INFO("Setting up services ...");

  ros::ServiceServer servicePickTest = node_handle.advertiseService("PickTest", boost::function<bool(armsweep::PickTest::Request  &req,
         armsweep::PickTest::Response &res)>(boost::bind(armsweep::pickTest, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... PickTest. ");
  sleep_t.sleep();


  ROS_INFO("+++Loading test meshes.");
  srv_LoadMesh.request.meshNames.clear(); 
  srv_LoadMesh.request.meshURLs.clear();
  {
      srv_LoadMesh.request.meshNames.push_back("chop_fw");
      srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/chop_fw.stl");
      srv_LoadMesh.request.meshNames.push_back("chop_in");
      srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/chop_in.stl");
      srv_LoadMesh.request.meshNames.push_back("chop_out");
      srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/chop_out.stl");
      srv_LoadMesh.request.meshNames.push_back("swerve_in");
      srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/swerve_in.stl");
      srv_LoadMesh.request.meshNames.push_back("swerve_out");
      srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/swerve_out.stl");
  }
  if (client_LoadMesh.call(srv_LoadMesh))
  {
    ROS_INFO("LoadMesh:");
    ROS_INFO("    errcode %d", srv_LoadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_LoadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service LoadMesh");
    return 1;
  }

  ROS_INFO("+++List currently loaded meshes.");
  if (client_ListMesh.call(srv_ListMesh))
  {
    ROS_INFO("ListMesh:");
    ROS_INFO("    errcode %d", srv_ListMesh.response.errcode);
    ROS_INFO("    msg: %s", srv_ListMesh.response.errmsg.c_str());
    ROS_INFO("List of loaded meshes (there are %d of them): ", srv_ListMesh.response.meshNames.size());
    int maxK = srv_ListMesh.response.meshNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_ListMesh.response.meshNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service ListMesh");
    return 1;
  }

  signal(SIGINT, onQuit);
  
  ROS_INFO("Just chillin'.");

  ros::spin();

  return 0;
}
