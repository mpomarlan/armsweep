#include "ros/ros.h"

// MoveIt!
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


#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <shape_msgs/Mesh.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <vector>
#include <utility>
#include <map>

#include <armsweep/armsweep.h>
#include <armsweep/armsweep_internal.h>

#define SCENE_TOPIC "planning_scene"
#define ROBOT_DESCRIPTION "robot_description"

int main(int argc, char **argv)
{

  //init ROS node
  ros::init (argc, argv, "armsweep");
  ros::WallDuration sleep_t(0.5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Started armsweep.");

  ros::NodeHandle node_handle;

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
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>(SCENE_TOPIC, 1);
  ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while((planning_scene_diff_publisher.getNumSubscribers() < 1) || (collision_object_publisher.getNumSubscribers() < 1))
  {
    sleep_t.sleep();
  }
  ROS_INFO("Advertized production of planning_scene_diff and collision_object messages.");


  armsweep::ArmsweepContext armsweepContext(kinematic_model, robotPlanningScene, trialWorld, trialCollisionWorld, planning_scene_diff_publisher, collision_object_publisher);

  ROS_INFO("Setting up services ...");

  ros::ServiceServer serviceLoadMesh = node_handle.advertiseService("LoadMesh", boost::function<bool(armsweep::LoadMesh::Request  &req,
         armsweep::LoadMesh::Response &res)>(boost::bind(armsweep::loadMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... LoadMesh ... ");

  ros::ServiceServer serviceListMeshes = node_handle.advertiseService("ListMesh", boost::function<bool(armsweep::ListMesh::Request  &req,
         armsweep::ListMesh::Response &res)>(boost::bind(armsweep::listMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... ListMesh ... ");

  ros::ServiceServer serviceUnloadMesh = node_handle.advertiseService("UnloadMesh", boost::function<bool(armsweep::UnloadMesh::Request  &req,
         armsweep::UnloadMesh::Response &res)>(boost::bind(armsweep::unloadMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... UnloadMesh ... ");

  ros::ServiceServer serviceCollideMeshes = node_handle.advertiseService("CollideMesh", boost::function<bool(armsweep::CollideMesh::Request  &req,
         armsweep::CollideMesh::Response &res)>(boost::bind(armsweep::collideMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... CollideMesh ... ");

  ros::ServiceServer serviceVisualizeMesh = node_handle.advertiseService("VisualizeMesh", boost::function<bool(armsweep::VisualizeMesh::Request  &req,
         armsweep::VisualizeMesh::Response &res)>(boost::bind(armsweep::visualizeMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... VisualizeMesh ... ");

  ros::ServiceServer serviceUnvisualizeMesh = node_handle.advertiseService("UnvisualizeMesh", boost::function<bool(armsweep::UnvisualizeMesh::Request  &req,
         armsweep::UnvisualizeMesh::Response &res)>(boost::bind(armsweep::unvisualizeMesh, &armsweepContext, _1, _2)));
  ROS_INFO("                    ... UnvisualizeMesh ... ");

  ROS_INFO("                    ... done.");

  ROS_INFO("Just chillin'.");


  ros::spin();

  return 0;
}

