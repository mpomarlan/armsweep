#ifndef __ARMSWEEP_INTERNAL_H__

#define __ARMSWEEP_INTERNAL_H__

#include <ros/ros.h>
#include <armsweep/meshitem.h>
#include <armsweep/strmsgutils.h>

namespace armsweep
{

class ArmsweepContext
{
    public:
        ArmsweepContext(robot_model::RobotModelConstPtr &robotModel, planning_scene::PlanningScenePtr &robotPlanningScene, collision_detection::WorldPtr &trialWorld, collision_detection::CollisionWorldFCL &trialCollisionWorld, ros::Publisher &pSD, ros::Publisher &cO):
            items(),
            robotModel(robotModel),
            robotPlanningScene(robotPlanningScene),
            trialWorld(trialWorld),
            trialCollisionWorld(&trialCollisionWorld),
            planningSceneDiffPublisher(pSD),
            collisionObjectPublisher(cO)
        {
        }
        ItemMap items;
        robot_model::RobotModelConstPtr robotModel;
        planning_scene::PlanningScenePtr robotPlanningScene;
        collision_detection::WorldPtr trialWorld;
        collision_detection::CollisionWorldPtr trialCollisionWorld;
        ros::Publisher &planningSceneDiffPublisher;
        ros::Publisher &collisionObjectPublisher;
};

bool pickTest(armsweep::ArmsweepContext *context, armsweep::PickTest::Request  &req, armsweep::PickTest::Response &res);

bool loadMesh(armsweep::ArmsweepContext *context, armsweep::LoadMesh::Request  &req, armsweep::LoadMesh::Response &res);

bool listMesh(armsweep::ArmsweepContext *context, armsweep::ListMesh::Request  &req, armsweep::ListMesh::Response &res);

bool unloadMesh(armsweep::ArmsweepContext *context, armsweep::UnloadMesh::Request  &req, armsweep::UnloadMesh::Response &res);

bool collideMesh(armsweep::ArmsweepContext *context, armsweep::CollideMesh::Request  &req, armsweep::CollideMesh::Response &res);

bool visualizeMesh(armsweep::ArmsweepContext *context, armsweep::VisualizeMesh::Request  &req, armsweep::VisualizeMesh::Response &res);

bool unvisualizeMesh(armsweep::ArmsweepContext *context, armsweep::UnvisualizeMesh::Request  &req, armsweep::UnvisualizeMesh::Response &res);


}

#endif
