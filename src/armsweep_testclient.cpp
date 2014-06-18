#include "ros/ros.h"
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

#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/profiler/profiler.h>

#define KEYPAUSE() std::cin.get()

#define SCENE_TOPIC "planning_scene"
#define ROBOT_DESCRIPTION "robot_description"

int main(int argc, char **argv)
{

  //init ROS node
  ros::init (argc, argv, "armsweep_testclient");
  ros::WallDuration sleep_t(0.5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Started armsweep_testclient.");

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


  ros::NodeHandle node_handle;

  ros::ServiceClient client_LoadMesh = node_handle.serviceClient<armsweep::LoadMesh>("LoadMesh");
  ros::ServiceClient client_ListMesh = node_handle.serviceClient<armsweep::ListMesh>("ListMesh");
  ros::ServiceClient client_UnloadMesh = node_handle.serviceClient<armsweep::UnloadMesh>("UnloadMesh");
  ros::ServiceClient client_CollideMesh = node_handle.serviceClient<armsweep::CollideMesh>("CollideMesh");
  ros::ServiceClient client_VisualizeMesh = node_handle.serviceClient<armsweep::VisualizeMesh>("VisualizeMesh");
  ros::ServiceClient client_UnvisualizeMesh = node_handle.serviceClient<armsweep::UnvisualizeMesh>("UnvisualizeMesh");

  ros::ServiceClient client_PickTest = node_handle.serviceClient<armsweep::PickTest>("PickTest");

  armsweep::LoadMesh srv_LoadMesh;
  armsweep::ListMesh srv_ListMesh;
  armsweep::UnloadMesh srv_UnloadMesh;
  armsweep::CollideMesh srv_CollideMesh;
  armsweep::VisualizeMesh srv_VisualizeMesh;
  armsweep::UnvisualizeMesh srv_UnvisualizeMesh;
  armsweep::PickTest srv_PickTest;

  ros::Publisher trajectoryPublisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  planning_interface::MotionPlanResponse res;
  moveit_msgs::MotionPlanResponse msg;
  moveit_msgs::DisplayTrajectory dMsg;

  ROS_INFO("+++Call PickTest.");

  Eigen::Affine3d torsoPose, targetPose;

  torsoPose = robotPlanningScene->getCurrentState().getGlobalLinkTransform("torso_lift_link");
  //targetPose(0,3) = 0.6452;//torsoPose(0, 3) + 0.2;
  //targetPose(1,3) = -0.4328;//torsoPose(1, 3) - 0.1;
  //targetPose(2,3) = 0.6366;//torsoPose(1, 3) - 0.22;

  targetPose(0,3) = 0.6452;//torsoPose(0, 3) + 0.2;
  targetPose(1,3) = -0.2328;//torsoPose(1, 3) - 0.1;
  targetPose(2,3) = 0.6366;//torsoPose(1, 3) - 0.22;
  targetPose(0, 0) = 0; targetPose(0, 1) = 1; targetPose(0, 2) = 0;
  targetPose(1, 0) = -1; targetPose(1, 1) = 0; targetPose(1, 2) = 0;
  targetPose(2, 0) = 0; targetPose(2, 1) = 0; targetPose(2, 2) = 1;

  geometry_msgs::Pose tPoseMsg;
  tf::poseEigenToMsg(targetPose, srv_PickTest.request.target);
  srv_PickTest.request.allowedCollisions.clear();

  moveit::tools::Profiler::Clear();
  moveit::tools::Profiler::Start();
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(kinematic_model, "right_arm"));
  res.trajectory_->clear();
  if (client_PickTest.call(srv_PickTest))
  {
    ROS_INFO("PickTest:");
    ROS_INFO("    errcode %d", srv_PickTest.response.errcode);
    ROS_INFO("    msg: %s", srv_PickTest.response.errmsg.c_str());
    ROS_INFO("    Trajectory status: %d", srv_PickTest.response.foundTrajectory);
    if(srv_PickTest.response.foundTrajectory)
    {
        ROS_INFO("    points on approach path: %d", srv_PickTest.response.approach.size());
        for(int k = 0; k < srv_PickTest.response.approach.size(); k++)
        {
            robot_state::RobotState aux(kinematic_model);
            robot_state::robotStateMsgToRobotState(srv_PickTest.response.approach[k], aux);
            res.trajectory_->addSuffixWayPoint(aux, 0.1);
        }
        res.getMessage(msg);
        dMsg.model_id = "pr2";
        dMsg.trajectory_start = msg.trajectory_start;
        dMsg.trajectory.push_back(msg.trajectory);
        trajectoryPublisher.publish(dMsg);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service PickTest");
    return 1;
  }
  moveit::tools::Profiler::Stop();
  moveit::tools::Profiler::Console();


#if 0

  ROS_INFO("+++Call ListMesh at the start for an initial view of the loaded mesh names. Cube should not be on the list.");

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

  ROS_INFO("+++Try to unload mesh cube, which should not be on the list.");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("cube");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }

  ROS_INFO("+++Try to load mesh cube, but path is wrong.");
  srv_LoadMesh.request.meshNames.clear(); srv_LoadMesh.request.meshNames.push_back("cube");
  srv_LoadMesh.request.meshURLs.clear(); srv_LoadMesh.request.meshURLs.push_back("cube.stl");
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

  ROS_INFO("+++Try to load mesh cube, with correct path.");
  srv_LoadMesh.request.meshURLs.clear(); srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/cube.stl");
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

  ROS_INFO("+++Try to overload mesh cube.");
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

  ROS_INFO("+++List meshes. Cube should be there.");
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

  ROS_INFO("+++Unload mesh cube.");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("cube");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }

  ROS_INFO("+++List meshes again, cube should not be present.");
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

  ROS_INFO("+++Try to re/overload mesh cube.");
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


  Eigen::Affine3d pose;
  pose(0, 0) = pose(1, 1) = pose(2, 2) = pose(3, 3) = 1;
  pose(0, 1) = pose(0, 2) = pose(0, 3) = pose(1, 0) = 0;
  pose(1, 2) = pose(1, 3) = pose(2, 0) = pose(2, 1) = 0;
  pose(2, 3) = pose(3, 0) = pose(3, 1) = pose(3, 2) = 0;

  ROS_INFO("+++Try to visualize mesh torus (which is not loaded).");
  srv_VisualizeMesh.request.meshNames.clear(); srv_VisualizeMesh.request.meshNames.push_back("torus");
  geometry_msgs::Pose poseMsg;
  tf::poseEigenToMsg(pose, poseMsg);
  srv_VisualizeMesh.request.poses.clear(); srv_VisualizeMesh.request.poses.push_back(poseMsg);
  if (client_VisualizeMesh.call(srv_VisualizeMesh))
  {
    ROS_INFO("VisualizeMesh:");
    ROS_INFO("    errcode %d", srv_VisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_VisualizeMesh.response.errmsgs[0].c_str());
    ROS_INFO("List of visualized mesh instances (there are %d of them): ", srv_VisualizeMesh.response.instanceNames.size());
    int maxK = srv_VisualizeMesh.response.instanceNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_VisualizeMesh.response.instanceNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service VisualizeMesh");
    return 1;
  }

  ROS_INFO("+++Try to visualize three cube meshes.");
  srv_VisualizeMesh.request.meshNames.clear(); srv_VisualizeMesh.request.meshNames.push_back("cube"); srv_VisualizeMesh.request.meshNames.push_back("cube"); srv_VisualizeMesh.request.meshNames.push_back("cube");
  srv_VisualizeMesh.request.poses.resize(3);
  pose(0,3) = 0;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[0]);
  pose(0,3) = 1;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[1]);
  pose(0,3) = 2;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[2]);
  if (client_VisualizeMesh.call(srv_VisualizeMesh))
  {
    ROS_INFO("VisualizeMesh:");
    ROS_INFO("    errcode %d", srv_VisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_VisualizeMesh.response.errmsgs[0].c_str());
    ROS_INFO("List of visualized mesh instances (there are %d of them): ", srv_VisualizeMesh.response.instanceNames.size());
    int maxK = srv_VisualizeMesh.response.instanceNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_VisualizeMesh.response.instanceNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service VisualizeMesh");
    return 1;
  }

  ROS_INFO("Press ENTER key to continue ...");
  KEYPAUSE();

  ROS_INFO("+++Unvisualize mesh intance torus (not loaded, not an armsweep instance name).");
  srv_UnvisualizeMesh.request.meshNames.clear(); srv_UnvisualizeMesh.request.meshNames.push_back("torus");
  if (client_UnvisualizeMesh.call(srv_UnvisualizeMesh))
  {
    ROS_INFO("UnvisualizeMesh:");
    ROS_INFO("    errcode %d", srv_UnvisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnvisualizeMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnvisualizeMesh");
    return 1;
  }
  ROS_INFO("+++Unvisualize mesh intance armsweepViz_torus_1 (not loaded).");
  srv_UnvisualizeMesh.request.meshNames.clear(); srv_UnvisualizeMesh.request.meshNames.push_back("armsweepViz_torus_1");
  if (client_UnvisualizeMesh.call(srv_UnvisualizeMesh))
  {
    ROS_INFO("UnvisualizeMesh:");
    ROS_INFO("    errcode %d", srv_UnvisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnvisualizeMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnvisualizeMesh");
    return 1;
  }
  ROS_INFO("+++Unvisualize mesh intance armsweepViz_cube_2 (loaded and fine, check that middle cube goes away).");
  srv_UnvisualizeMesh.request.meshNames.clear(); srv_UnvisualizeMesh.request.meshNames.push_back(srv_VisualizeMesh.response.instanceNames[1]);
  if (client_UnvisualizeMesh.call(srv_UnvisualizeMesh))
  {
    ROS_INFO("UnvisualizeMesh:");
    ROS_INFO("    errcode %d", srv_UnvisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnvisualizeMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnvisualizeMesh");
    return 1;
  }
  ROS_INFO("Press ENTER key to continue ...");
  KEYPAUSE();
  ROS_INFO("+++Unload mesh cube (should also unvisualize remaining cube instances).");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("cube");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }

  ROS_INFO("+++List meshes again, cube should not be present.");
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

  ROS_INFO("+++Reload mesh cube.");
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

  ROS_INFO("+++Collision check between robot and two instances of cube. One of them is allowed to touch base_link and should report no collision. The other is not allowed to touch the robot and should report collision.");
  srv_CollideMesh.request.meshNames.clear(); srv_CollideMesh.request.meshNames.push_back("cube"); srv_CollideMesh.request.meshNames.push_back("cube");
  srv_CollideMesh.request.poses.clear(); srv_CollideMesh.request.poses.resize(2);
  pose(0,3) = 0;
  tf::poseEigenToMsg(pose, srv_CollideMesh.request.poses[0]);
  tf::poseEigenToMsg(pose, srv_CollideMesh.request.poses[1]);
  armsweep::AllowedCollisions acmMsg;
  acmMsg.linkGroupNames.clear();
  acmMsg.linkNames.clear();
  acmMsg.objectNames.clear();
  srv_CollideMesh.request.allowedCollisions.clear(); srv_CollideMesh.request.allowedCollisions.push_back(acmMsg);
  acmMsg.linkNames.push_back("base_link"); srv_CollideMesh.request.allowedCollisions.push_back(acmMsg);
  if (client_CollideMesh.call(srv_CollideMesh))
  {
    int maxK = srv_CollideMesh.response.errcodes.size();
    ROS_INFO("CollideMesh received %d answers: ", maxK);
    for(int k = 0 ; k < maxK; k++)
    {
        ROS_INFO("    errcode %d", srv_CollideMesh.response.errcodes[k]);
        ROS_INFO("    msg: %s", srv_CollideMesh.response.errmsgs[k].c_str());
        ROS_INFO("    inFreespace: %d", srv_CollideMesh.response.inFreespace[k]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service CollideMesh");
    return 1;
  }

  ROS_INFO("+++Try to visualize three cube meshes. One of them will also be used as a collision object in a subsequent call to CollideMesh.");
  srv_VisualizeMesh.request.meshNames.clear(); srv_VisualizeMesh.request.meshNames.push_back("cube"); srv_VisualizeMesh.request.meshNames.push_back("cube"); srv_VisualizeMesh.request.meshNames.push_back("cube");
  srv_VisualizeMesh.request.poses.resize(3);
  pose(0,3) = 0;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[0]);
  pose(0,3) = 1;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[1]);
  pose(0,3) = 2;
  tf::poseEigenToMsg(pose, srv_VisualizeMesh.request.poses[2]);
  if (client_VisualizeMesh.call(srv_VisualizeMesh))
  {
    ROS_INFO("VisualizeMesh:");
    ROS_INFO("    errcode %d", srv_VisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_VisualizeMesh.response.errmsgs[0].c_str());
    ROS_INFO("List of visualized mesh instances (there are %d of them): ", srv_VisualizeMesh.response.instanceNames.size());
    int maxK = srv_VisualizeMesh.response.instanceNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_VisualizeMesh.response.instanceNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service VisualizeMesh");
    return 1;
  }

  ROS_INFO("+++Collision check between robot, %s, and two instances of cube. Both cube instances are allowed to touch base_link, but only one may touch %s.", srv_VisualizeMesh.response.instanceNames[0].c_str(), srv_VisualizeMesh.response.instanceNames[0].c_str());
  srv_CollideMesh.request.meshNames.clear(); srv_CollideMesh.request.meshNames.push_back("cube"); srv_CollideMesh.request.meshNames.push_back("cube");
  srv_CollideMesh.request.poses.clear(); srv_CollideMesh.request.poses.resize(2);
  pose(0,3) = 0;
  tf::poseEigenToMsg(pose, srv_CollideMesh.request.poses[0]);
  tf::poseEigenToMsg(pose, srv_CollideMesh.request.poses[1]);
  acmMsg.linkGroupNames.clear();
  acmMsg.linkNames.clear(); acmMsg.linkNames.push_back("base_link");
  acmMsg.objectNames.clear();
  srv_CollideMesh.request.allowedCollisions.clear(); srv_CollideMesh.request.allowedCollisions.push_back(acmMsg);
  acmMsg.objectNames.push_back(srv_VisualizeMesh.response.instanceNames[0]); srv_CollideMesh.request.allowedCollisions.push_back(acmMsg);
  if (client_CollideMesh.call(srv_CollideMesh))
  {
    int maxK = srv_CollideMesh.response.errcodes.size();
    ROS_INFO("CollideMesh received %d answers: ", maxK);
    for(int k = 0 ; k < maxK; k++)
    {
        ROS_INFO("    errcode %d", srv_CollideMesh.response.errcodes[k]);
        ROS_INFO("    msg: %s", srv_CollideMesh.response.errmsgs[k].c_str());
        ROS_INFO("    inFreespace: %d", srv_CollideMesh.response.inFreespace[k]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service CollideMesh");
    return 1;
  }

  ROS_INFO("+++Unload mesh cube for a clean end (should also unvisualize remaining cube instances).");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("cube");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }

  ROS_INFO("+++Try to load PR2_liftovertable_sv.stl as mesh name sweptvolume, with correct path.");
  srv_LoadMesh.request.meshURLs.clear(); srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/PR2_liftovertable_sv.stl");
  srv_LoadMesh.request.meshNames.clear(); srv_LoadMesh.request.meshNames.push_back("sweptvolume");
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

  pose = robotPlanningScene->getCurrentState().getGlobalLinkTransform("r_shoulder_pan_link");

  ROS_INFO("+++Try to visualize mesh sweptvolume (which should appear at robot's right shoulder).");
  srv_VisualizeMesh.request.meshNames.clear(); srv_VisualizeMesh.request.meshNames.push_back("sweptvolume");
  tf::poseEigenToMsg(pose, poseMsg);
  srv_VisualizeMesh.request.poses.clear(); srv_VisualizeMesh.request.poses.push_back(poseMsg);
  if (client_VisualizeMesh.call(srv_VisualizeMesh))
  {
    ROS_INFO("VisualizeMesh:");
    ROS_INFO("    errcode %d", srv_VisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_VisualizeMesh.response.errmsgs[0].c_str());
    ROS_INFO("List of visualized mesh instances (there are %d of them): ", srv_VisualizeMesh.response.instanceNames.size());
    int maxK = srv_VisualizeMesh.response.instanceNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_VisualizeMesh.response.instanceNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service VisualizeMesh");
    return 1;
  }

  ROS_INFO("Press ENTER key to continue ...");
  KEYPAUSE();
  ROS_INFO("+++Unload mesh sweptvolume for a clean end (should also unvisualize remaining instances).");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("sweptvolume");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }

  ROS_INFO("+++Try to load PR2_reach_sv.stl as mesh name sweptvolume, with correct path.");
  srv_LoadMesh.request.meshURLs.clear(); srv_LoadMesh.request.meshURLs.push_back("package://armsweep/res/PR2_reach_sv.stl");
  srv_LoadMesh.request.meshNames.clear(); srv_LoadMesh.request.meshNames.push_back("sweptvolume");
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

  pose = robotPlanningScene->getCurrentState().getGlobalLinkTransform("r_shoulder_lift_link");

  ROS_INFO("+++Try to visualize mesh sweptvolume (which should appear at robot's right shoulder).");
  srv_VisualizeMesh.request.meshNames.clear(); srv_VisualizeMesh.request.meshNames.push_back("sweptvolume");
  tf::poseEigenToMsg(pose, poseMsg);
  srv_VisualizeMesh.request.poses.clear(); srv_VisualizeMesh.request.poses.push_back(poseMsg);
  if (client_VisualizeMesh.call(srv_VisualizeMesh))
  {
    ROS_INFO("VisualizeMesh:");
    ROS_INFO("    errcode %d", srv_VisualizeMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_VisualizeMesh.response.errmsgs[0].c_str());
    ROS_INFO("List of visualized mesh instances (there are %d of them): ", srv_VisualizeMesh.response.instanceNames.size());
    int maxK = srv_VisualizeMesh.response.instanceNames.size();
    for(int k = 0 ; k < maxK; k++)
        ROS_INFO("    %s", srv_VisualizeMesh.response.instanceNames[k].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service VisualizeMesh");
    return 1;
  }

  ROS_INFO("Press ENTER key to continue ...");
  KEYPAUSE();
  ROS_INFO("+++Unload mesh sweptvolume for a clean end (should also unvisualize remaining instances).");
  srv_UnloadMesh.request.meshNames.clear();srv_UnloadMesh.request.meshNames.push_back("sweptvolume");
  if (client_UnloadMesh.call(srv_UnloadMesh))
  {
    ROS_INFO("UnloadMesh:");
    ROS_INFO("    errcode %d", srv_UnloadMesh.response.errcodes[0]);
    ROS_INFO("    msg: %s", srv_UnloadMesh.response.errmsgs[0].c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service UnloadMesh");
    return 1;
  }
#endif

  ros::spin();


  return 0;
}
