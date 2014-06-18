#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_listener.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/profiler/profiler.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>

#include <armsweep/armsweep.h>
#include <armsweep/armsweep_internal.h>

#include <boost/lexical_cast.hpp>

void meshToMsg(shapes::Mesh const*mesh, shape_msgs::Mesh *msg)
{
    geometry_msgs::Point aux;
    shape_msgs::MeshTriangle auxT;
    msg->triangles.clear();
    msg->vertices.clear();
    int maxV = mesh->vertex_count;
    for(int v = 0; v < maxV; v++)
    {
        aux.x = mesh->vertices[3*v + 0];
        aux.y = mesh->vertices[3*v + 1];
        aux.z = mesh->vertices[3*v + 2];
        msg->vertices.push_back(aux);
    }
    int maxT = mesh->triangle_count;
    for(int t = 0; t < maxT; t++)
    {
        auxT.vertex_indices[0] = mesh->triangles[3*t + 0];
        auxT.vertex_indices[1] = mesh->triangles[3*t + 1];
        auxT.vertex_indices[2] = mesh->triangles[3*t + 2];
        msg->triangles.push_back(auxT);
    }
}

bool getMeshNameAndInstanceId(std::string const&meshName, std::string &auxName, int &auxInt)
{
    std::string prefix = meshName.substr(0, armsweep::standardVizPrefix.length());
    if(prefix != armsweep::standardVizPrefix)
    {
        return false;
    }
    int lastUnderscore = meshName.rfind('_');
    if(meshName.length()-1 <= lastUnderscore)
        return false;
    std::string suffix = meshName.substr(lastUnderscore + 1);
    try
    {
        auxInt = boost::lexical_cast<int>(suffix.c_str());
        auxName = meshName.substr(prefix.length(), lastUnderscore-prefix.length());
        return true;
    }
    catch(boost::bad_lexical_cast const& e)
    {
        ROS_INFO("Failed to read int from %s", suffix.c_str());
        return false;
    }
}

void getRobotLinkNames(armsweep::ArmsweepContext const&context, std::vector<std::string> &linkNames)
{
    std::vector<std::string> linkNamesAux = context.robotModel->getLinkModelNames();
    int lK, maxK;
    lK = linkNames.size();
    maxK = lK + linkNamesAux.size();
    linkNames.resize(maxK);
    for(int k = 0; k < maxK-lK; k++)
        linkNames[lK+k] = linkNamesAux[k];
}

void getGroupLinkNames(armsweep::ArmsweepContext const&context, std::string const&groupName, std::vector<std::string> &linkNames)
{
    std::vector<std::string> linkNamesAux = context.robotModel->getJointModelGroup(groupName)->getLinkModelNames();
    int lK, maxK;
    lK = linkNames.size();
    maxK = lK + linkNamesAux.size();
    linkNames.resize(maxK);
    for(int k = 0; k < maxK-lK; k++)
        linkNames[lK+k] = linkNamesAux[k];
}

void SetEntriesInACM(std::vector<std::string> const&objsA, std::vector<std::string> const&objsB, planning_scene::PlanningScenePtr &planningScene)
{
    collision_detection::AllowedCollisionMatrix &acm(planningScene->getAllowedCollisionMatrixNonConst());
    acm.setEntry(objsA, objsB, true);
}

void RemoveEntriesFromACM(std::vector<std::string> const&objsA, std::vector<std::string> const&objsB, planning_scene::PlanningScenePtr &planningScene)
{
    collision_detection::AllowedCollisionMatrix &acm(planningScene->getAllowedCollisionMatrixNonConst());
    int maxK = objsA.size();
    int maxJ = objsB.size();
    for(int k = 0; k < maxK; k++)
        for(int j = 0; j < maxJ; j++)
            acm.removeEntry(objsA[k], objsB[j]);
}

bool loadMesh_internal(std::string const&meshName, std::string const&meshURL, armsweep::ArmsweepContext &context, int &errcode, std::string &errmsg)
{
    if(context.items.end() == context.items.find(meshName))
    {
        shapes::Mesh *newMesh = shapes::createMeshFromResource(meshURL, Eigen::Vector3d(1, 1, 1));
        if(newMesh)
        {
            armsweep::MeshConstPtr auxPtr(newMesh);
            armsweep::MeshItem aux(auxPtr, meshName, meshURL);
            armsweep::ItemMapElement auxPair(meshName, aux);
            context.items.insert(auxPair);
            errcode = armsweep::AS_AllOk;
            armsweep::getStrMsg(armsweep::AS_AllOk, meshName, meshURL, errmsg);
        }
        else
        {
            errcode = armsweep::AS_CannotLoad;
            armsweep::getStrMsg(armsweep::AS_CannotLoad, meshName, meshURL, errmsg);
        }
    }
    else
    {
        errcode = armsweep::AS_AlreadyLoaded;
        armsweep::getStrMsg(armsweep::AS_AlreadyLoaded, meshName, meshURL, errmsg);
    }
    return true;
}

void getAllTheStrings(armsweep::ArmsweepContext const&context, armsweep::AllowedCollisions const&allowedCollisions, std::vector<std::string> &strVec)
{
    strVec.clear(); strVec.reserve(allowedCollisions.linkNames.size() + allowedCollisions.objectNames.size());
    int maxK = allowedCollisions.linkNames.size();
    for(int k = 0; k < maxK; k++)
        strVec.push_back(allowedCollisions.linkNames[k]);
    maxK = allowedCollisions.objectNames.size();
    for(int k = 0; k < maxK; k++)
        strVec.push_back(allowedCollisions.objectNames[k]);
    maxK = allowedCollisions.linkGroupNames.size();
    for(int k = 0; k < maxK; k++)
        getGroupLinkNames(context, allowedCollisions.linkGroupNames[k], strVec);
}

bool collideMesh_internal(std::string const&meshName, geometry_msgs::Pose const&pose, armsweep::ArmsweepContext &context, int &errcode, std::string &errmsg, armsweep::ItemMap::iterator &iteratorForCollide, std::vector<std::string> &auxStrVec, armsweep::AllowedCollisions const&allowedCollisions)
{
    armsweep::ItemMap::iterator it = context.items.find(meshName);
    iteratorForCollide = it;
    auxStrVec.clear();
    if(it == context.items.end())
    {
        errcode = armsweep::AS_NotLoaded;
        armsweep::getStrMsg(armsweep::AS_NotLoaded, meshName, "", errmsg);
        return false;
    }
    errcode = armsweep::AS_AllOk;
    armsweep::getStrMsg(armsweep::AS_AllOk, meshName, "", errmsg);
    getAllTheStrings(context, allowedCollisions, auxStrVec);
    return true;
}

int findStrsInList(std::string const&a, std::string const&b, std::vector<std::string> const&strList)
{
    int retq = -1;
    int maxK = strList.size();
    std::string aux;
    for(int k = 0; (retq < 0) && (k < maxK); k++)
    {
        aux = armsweep::standardTrialPrefix+strList[k]+boost::lexical_cast<std::string>(k);
        if((a == aux) || (b == aux))
            retq = k;
    }
    return retq;
}

void collideWithWorld(armsweep::ArmsweepContext &context, std::vector<armsweep::ItemMap::iterator> const&iteratorsForCollide, std::vector<std::string> const&meshesForCollide, std::vector<Eigen::Affine3d> const&posesForCollide, armsweep::StrVecVec const&allowedForCollide, std::vector<int> const&indicesForCollide, armsweep::CollideMesh::Response &res)
{
    moveit::tools::Profiler::Begin("collideWithWorld::build acm");
    collision_detection::AllowedCollisionMatrix acm;
    std::vector<std::string> auxVec;
    auxVec.clear();auxVec.push_back("");
    int maxK = meshesForCollide.size();
    for(int k = 0; k < maxK; k++)
        if(res.inFreespace[indicesForCollide[k]])
        {
            std::string auxStr(armsweep::standardTrialPrefix+meshesForCollide[k]+boost::lexical_cast<std::string>(k));
            context.trialWorld->addToObject(auxStr, shapes::ShapeConstPtr((*iteratorsForCollide[k]).second.mesh), posesForCollide[k]);
            auxVec[0] = auxStr;
            acm.setEntry(auxVec, allowedForCollide[k], true);
        }
    moveit::tools::Profiler::End("collideWithWorld::build acm");


    moveit::tools::Profiler::Begin("collideWithWorld::checkWorldCollision");
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.contacts = true;
    creq.distance = true;
    creq.max_contacts = 1000000;
    creq.max_contacts_per_pair = 1;
    creq.verbose = true;

    context.trialCollisionWorld->checkWorldCollision(creq, cres, *(context.robotPlanningScene->getCollisionWorld().get()), acm);
    moveit::tools::Profiler::End("collideWithWorld::checkWorldCollision");

    moveit::tools::Profiler::Begin("collideWithWorld::result extraction");
    collision_detection::CollisionResult::ContactMap::iterator cit = cres.contacts.begin();
    for(; cit != cres.contacts.end(); cit++)
    {
        int k = findStrsInList((*cit).first.first, (*cit).first.second, meshesForCollide);
        if(0 <= k)
        {
            res.inFreespace[indicesForCollide[k]] = 0;
        }
    }
    moveit::tools::Profiler::End("collideWithWorld::result extraction");

    moveit::tools::Profiler::Begin("collideWithWorld::clear trialWorld");
    context.trialWorld->clearObjects();
    moveit::tools::Profiler::End("collideWithWorld::clear trialWorld");
}

void collideWithRobot(armsweep::ArmsweepContext &context, std::vector<armsweep::ItemMap::iterator> const&iteratorsForCollide, std::vector<std::string> const&meshesForCollide, std::vector<Eigen::Affine3d> const&posesForCollide, armsweep::StrVecVec const&allowedForCollide, std::vector<int> const&indicesForCollide, armsweep::CollideMesh::Response &res)
{
    moveit::tools::Profiler::Begin("collideWithRobot::build acm");
    collision_detection::AllowedCollisionMatrix acm(context.robotPlanningScene->getAllowedCollisionMatrix());
    std::vector<std::string> auxVec;
    std::vector<std::string> regIds;
    auxVec.clear();auxVec.push_back("");
    regIds.clear(); regIds.reserve(iteratorsForCollide.size());
    int maxK = meshesForCollide.size();
    for(int k = 0; k < maxK; k++)
        if(res.inFreespace[indicesForCollide[k]])
        {
            std::string auxStr(armsweep::standardTrialPrefix+meshesForCollide[k]+boost::lexical_cast<std::string>(k));
            context.robotPlanningScene->getWorldNonConst()->addToObject(auxStr, shapes::ShapeConstPtr((*iteratorsForCollide[k]).second.mesh), posesForCollide[k]);
            auxVec[0] = auxStr;
            acm.setEntry(auxVec, allowedForCollide[k], true);
            regIds.push_back(auxStr);
        }
    moveit::tools::Profiler::End("collideWithRobot::build acm");

    moveit::tools::Profiler::Begin("collideWithRobot::checkCollision");
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.contacts = true;
    creq.distance = true;
    creq.max_contacts = 1000000;
    creq.max_contacts_per_pair = 1;
    creq.verbose = true;

    context.robotPlanningScene->checkCollision(creq, cres, context.robotPlanningScene->getCurrentState(), acm);
    moveit::tools::Profiler::End("collideWithRobot::checkCollision");

    moveit::tools::Profiler::Begin("collideWithRobot::result extraction");
    collision_detection::CollisionResult::ContactMap::iterator cit = cres.contacts.begin();
    for(; cit != cres.contacts.end(); cit++)
    {
        int k = findStrsInList((*cit).first.first, (*cit).first.second, meshesForCollide);
        if(0 <= k)
        {
            res.inFreespace[indicesForCollide[k]] = 0;
        }
    }
    moveit::tools::Profiler::End("collideWithRobot::result extraction");

    moveit::tools::Profiler::Begin("collideWithRobot::restore robotPlanningScene");
    int maxJ = regIds.size();
    for(int j = 0; j < maxJ; j++)
        context.robotPlanningScene->getWorldNonConst()->removeObject(regIds[j]);
    moveit::tools::Profiler::End("collideWithRobot::restore robotPlanningScene");
}


bool unloadMesh_internal(std::string const&meshName, armsweep::ArmsweepContext &context, int &errcode, std::string &errmsg, std::vector<std::string> &unvisualizedMeshNames)
{
    armsweep::ItemMap::iterator it = context.items.find(meshName);
    if(context.items.end() == it)
    {
        errcode = armsweep::AS_NotLoaded;
        armsweep::getStrMsg(armsweep::AS_NotLoaded, meshName, "", errmsg);
        return false;
    }
    errcode = armsweep::AS_AllOk;
    armsweep::getStrMsg(armsweep::AS_AllOk, meshName, "", errmsg);
    int maxK = (*it).second.displayedInstances.size();
    for(int k = 0; k < maxK; k++)
    {
        unvisualizedMeshNames.push_back(armsweep::standardVizPrefix + meshName + "_" + boost::lexical_cast<std::string>((*it).second.displayedInstances[k]));
    }
    return true;
}

bool visualizeMesh_internal(std::string const&meshName, geometry_msgs::Pose const&pose, armsweep::ArmsweepContext &context, int &errcode, std::string &errmsg, std::string &instanceName, shapes::Mesh const*&meshPtr)
{
    armsweep::ItemMap::iterator it = context.items.find(meshName);
    if(context.items.end() == it)
    {
        errcode = armsweep::AS_NotLoaded;
        armsweep::getStrMsg(armsweep::AS_NotLoaded, meshName, "", errmsg);
        instanceName = "";
        meshPtr = NULL;
        return false;
    }
    else
    {
        errcode = armsweep::AS_AllOk;
        armsweep::getStrMsg(armsweep::AS_AllOk, meshName, "", errmsg);
        instanceName = armsweep::standardVizPrefix;
        int newLast = (++(*it).second.displayedLastInstance);
        (*it).second.displayedInstances.push_back(newLast);
        instanceName = instanceName + meshName + "_" + boost::lexical_cast<std::string>(newLast);
        meshPtr = (*it).second.mesh.get();
        return true;
    }
}

bool unvisualizeMesh_internal(std::string const&meshName, armsweep::ArmsweepContext &context, int &errcode, std::string &errmsg, std::string &auxName, int &auxInt)
{
    if(!getMeshNameAndInstanceId(meshName, auxName, auxInt))
    {
        errcode = armsweep::AS_NotAnArmsweepId;
        armsweep::getStrMsg(armsweep::AS_NotAnArmsweepId, meshName, "", errmsg);
        return false;
    }
    armsweep::ItemMap::iterator it = context.items.find(auxName);
    if(context.items.end() == it)
    {
        errcode = armsweep::AS_NotLoaded;
        armsweep::getStrMsg(armsweep::AS_NotLoaded, auxName, "", errmsg);
        return false;
    }
    int maxK = (*it).second.displayedInstances.size();
    int index = -1;
    int crLast = 0;
    for(int k = 0; k < maxK; k++)
    {
        if(auxInt == (*it).second.displayedInstances[k])
            index = k;
        else if(crLast < (*it).second.displayedInstances[k])
            crLast = (*it).second.displayedInstances[k];
    }
    (*it).second.displayedLastInstance = crLast;
    if(index < 0)
    {
        errcode = armsweep::AS_NotAnInstance;
        armsweep::getStrMsg(armsweep::AS_NotAnInstance, auxName, "", errmsg);
        return false;
    }
    errcode = armsweep::AS_AllOk;
    armsweep::getStrMsg(armsweep::AS_AllOk, auxName, "", errmsg);
    int auxX = (*it).second.displayedInstances[(*it).second.displayedInstances.size()-1];
    (*it).second.displayedInstances[(*it).second.displayedInstances.size()-1] = (*it).second.displayedInstances[index];
    (*it).second.displayedInstances[index] = auxX;
    (*it).second.displayedInstances.pop_back();
    return true;
}

bool unvisualizer(armsweep::ArmsweepContext &context, std::vector<std::string> const&unvisualizedMeshNames)
{
    ros::WallDuration sleep_t(0.2);
    int maxK = unvisualizedMeshNames.size();
    if(!maxK)
        return true;

    std::vector<std::string> robotLinkNames;
    robotLinkNames.clear();
    getRobotLinkNames(context, robotLinkNames);

    moveit_msgs::PlanningScene planning_scene_diff;
    geometry_msgs::TransformStamped transformStampedMsg;
    transformStampedMsg.header.frame_id = "odom_combined";
    planning_scene_diff.world.collision_objects.resize(maxK);
    planning_scene_diff.is_diff = true;
    RemoveEntriesFromACM(unvisualizedMeshNames, robotLinkNames, context.robotPlanningScene);
    context.robotPlanningScene->getAllowedCollisionMatrix().getMessage(planning_scene_diff.allowed_collision_matrix);

    for(int k = 0; k < maxK; k++)
    {
        moveit_msgs::CollisionObject &del_object(planning_scene_diff.world.collision_objects[k]);
        del_object.meshes.clear();
        del_object.mesh_poses.clear();
        del_object.id = unvisualizedMeshNames[k];
        del_object.header.frame_id = "odom_combined";
        del_object.operation = del_object.REMOVE;
        context.collisionObjectPublisher.publish(del_object);
    }
    sleep_t.sleep();

    context.planningSceneDiffPublisher.publish(planning_scene_diff);
    sleep_t.sleep();
}

bool armsweep::pickTest(armsweep::ArmsweepContext *contextP, armsweep::PickTest::Request  &req, armsweep::PickTest::Response &res)
{
    double tgx, tgy, tgz;
    double rgx, rgy, rgz;
    double ab_in, ab_out;
    double r_in, r_out;
    double dx, dy, dz;

    res.errcode = armsweep::AS_AllOk;
    armsweep::getStrMsg(armsweep::AS_AllOk, "", "", res.errmsg);
    res.foundTrajectory = 0;
    res.approach.clear();
    res.goBack.clear();

    Eigen::Affine3d targetPose_torsoFrame, targetPose_baseFrame;
    Eigen::Affine3d refPose_torsoFrame, refPose_baseFrame;
    Eigen::Affine3d torsoPose = contextP->robotPlanningScene->getCurrentState().getGlobalLinkTransform("torso_lift_link");
    tf::poseMsgToEigen(req.target, targetPose_baseFrame);
    targetPose_torsoFrame = torsoPose.inverse()*targetPose_baseFrame;

    Eigen::Affine3d approachPose_torsoFrame = targetPose_torsoFrame;
    approachPose_torsoFrame(0, 0) = 1; approachPose_torsoFrame(0, 1) = 0; approachPose_torsoFrame(0, 2) = 0;
    approachPose_torsoFrame(1, 0) = 0; approachPose_torsoFrame(1, 1) = 1; approachPose_torsoFrame(1, 2) = 0;
    approachPose_torsoFrame(2, 0) = 0; approachPose_torsoFrame(2, 1) = 0; approachPose_torsoFrame(2, 2) = 1;

    tgx = targetPose_torsoFrame(0, 3);
    tgy = targetPose_torsoFrame(1, 3);
    tgz = targetPose_torsoFrame(2, 3);
    rgx = 0.330;//0.2659 - 0.05;
    rgy = -0.467;//-0.5153 - 0.0;
    rgz = 0.058;//0.7925 - 0.7905;
    refPose_torsoFrame(0, 0) = 1; refPose_torsoFrame(0, 1) = 0; refPose_torsoFrame(0, 2) = 0; refPose_torsoFrame(0, 3) = rgx;
    refPose_torsoFrame(1, 0) = 0; refPose_torsoFrame(1, 1) = 1; refPose_torsoFrame(1, 2) = 0; refPose_torsoFrame(1, 3) = rgy;
    refPose_torsoFrame(2, 0) = 0; refPose_torsoFrame(2, 1) = 0; refPose_torsoFrame(2, 2) = 1; refPose_torsoFrame(2, 3) = rgz;
    refPose_torsoFrame(3, 0) = 0; refPose_torsoFrame(3, 1) = 0; refPose_torsoFrame(3, 2) = 0; refPose_torsoFrame(3, 3) = 1;
    refPose_baseFrame = torsoPose*refPose_torsoFrame;
    dx = -0.2216;
    dy = -0.2745;
    dz = 0.2732;
    r_out = r_in = std::sqrt(dx*dx + dy*dy + dz*dz);
    ab_in = std::atan2(-0.2745, -0.2216);
    ab_out = std::atan2(-0.2745, 0.2216);
    
    double cosAngleRefTargetX = targetPose_torsoFrame(0, 0);
    double distTargetRefX = targetPose_torsoFrame(1, 3);
    
    double theta_in = std::atan2(targetPose_torsoFrame(1, 3) - rgy, targetPose_torsoFrame(0, 3) - rgx);
    double theta_out = theta_in - ab_out;
    theta_in = theta_in - ab_in;
    double cti = std::cos(theta_in);
    double sti = std::sin(theta_in);
    double cto = std::cos(theta_out);
    double sto = std::sin(theta_out);
    double ctib = std::cos(theta_in + ab_in);
    double stib = std::sin(theta_in + ab_in);
    double ctob = std::cos(theta_out + ab_out);
    double stob = std::sin(theta_out + ab_out);

    double auxX = tgx + (ctib*r_in) - rgx;
    double auxY = tgy + (stib*r_in) - rgy;
    double dtr_in = std::sqrt(auxX*auxX + auxY*auxY);
    auxX = tgx + (ctob*r_out) - rgx;
    auxY = tgy + (stob*r_out) - rgy;
    double dtr_out = std::sqrt(auxX*auxX + auxY*auxY);

    Eigen::Vector3d position, normal, approach, vertical;
    std::vector<double> corVecX, corVecY, corVecZ;
    corVecX.clear(); corVecY.clear(); corVecZ.clear();

    std::string meshName;

    position(0) = tgx;
    position(1) = tgy;
    position(2) = tgz;
    normal(0) = 0;
    normal(1) = 1;
    normal(2) = 0;
    approach(0) = 1;
    approach(1) = 0;
    approach(2) = 0;
    if(0.95 < cosAngleRefTargetX)
    {
        meshName = "chop_fw";
        corVecX.push_back(-0.2418); corVecY.push_back(0); corVecZ.push_back(0.2973);
        corVecX.push_back(-0.2038); corVecY.push_back(0); corVecZ.push_back(0.2638);
        corVecX.push_back(-0.1698); corVecY.push_back(0); corVecZ.push_back(0.2261);
        corVecX.push_back(-0.1405); corVecY.push_back(0); corVecZ.push_back(0.1847);
        corVecX.push_back(-0.1162); corVecY.push_back(0); corVecZ.push_back(0.1403);
        corVecX.push_back(-0.0973); corVecY.push_back(0); corVecZ.push_back(0.0933);
        corVecX.push_back(-0.0591); corVecY.push_back(0); corVecZ.push_back(0.0452);
    }
    else if(std::fabs(distTargetRefX) < 0.1)
    {
        if(distTargetRefX < 0.0)
        {
            meshName = "chop_out";
            corVecX.push_back(-0.1832); corVecY.push_back(0.1025); corVecZ.push_back(0.3226);
            corVecX.push_back(-0.1418); corVecY.push_back(0.1025); corVecZ.push_back(0.2873);
            corVecX.push_back(-0.1047); corVecY.push_back(0.1025); corVecZ.push_back(0.2474);
            corVecX.push_back(-0.0725); corVecY.push_back(0.1025); corVecZ.push_back(0.2035);
            corVecX.push_back(-0.0450); corVecY.push_back(0.1025); corVecZ.push_back(0.1561);
            corVecX.push_back(-0.0243); corVecY.push_back(0.0960); corVecZ.push_back(0.1060);
            corVecX.push_back(-0.0090); corVecY.push_back(0.0600); corVecZ.push_back(0.0537);
        }
        else
        {
            meshName = "chop_in";
            corVecX.push_back(-0.1832); corVecY.push_back(-0.1025); corVecZ.push_back(0.3226);
            corVecX.push_back(-0.1418); corVecY.push_back(-0.1025); corVecZ.push_back(0.2873);
            corVecX.push_back(-0.1047); corVecY.push_back(-0.1025); corVecZ.push_back(0.2474);
            corVecX.push_back(-0.0725); corVecY.push_back(-0.1025); corVecZ.push_back(0.2035);
            corVecX.push_back(-0.0450); corVecY.push_back(-0.1025); corVecZ.push_back(0.1561);
            corVecX.push_back(-0.0243); corVecY.push_back(-0.0960); corVecZ.push_back(0.1060);
            corVecX.push_back(-0.0090); corVecY.push_back(-0.0600); corVecZ.push_back(0.0537);
        }
    }
    else
    {
        if(dtr_in < dtr_out)
        {
            meshName = "swerve_in";
            normal(0) = -sti;
            normal(1) = cti;
            normal(2) = 0;
            approach(0) = cti;
            approach(1) = sti;
            approach(2) = 0;
            corVecX.push_back(-0.2216); corVecY.push_back(-0.2745); corVecZ.push_back(0.2732);
            corVecX.push_back(-0.1851); corVecY.push_back(-0.2665); corVecZ.push_back(0.2419);
            corVecX.push_back(-0.1524); corVecY.push_back(-0.2562); corVecZ.push_back(0.2066);
            corVecX.push_back(-0.1241); corVecY.push_back(-0.2402); corVecZ.push_back(0.1678);
            corVecX.push_back(-0.1005); corVecY.push_back(-0.2196); corVecZ.push_back(0.1259);
            corVecX.push_back(-0.0818); corVecY.push_back(-0.1900); corVecZ.push_back(0.0816);
            corVecX.push_back(-0.0685); corVecY.push_back(-0.1213); corVecZ.push_back(0.0354);
        }
        else
        {
            meshName = "swerve_out";
            normal(0) = -sto;
            normal(1) = cto;
            normal(2) = 0;
            approach(0) = cto;
            approach(1) = sto;
            approach(2) = 0;
            corVecX.push_back(-0.2216); corVecY.push_back(0.2745); corVecZ.push_back(0.2732);
            corVecX.push_back(-0.1851); corVecY.push_back(0.2665); corVecZ.push_back(0.2419);
            corVecX.push_back(-0.1524); corVecY.push_back(0.2562); corVecZ.push_back(0.2066);
            corVecX.push_back(-0.1241); corVecY.push_back(0.2402); corVecZ.push_back(0.1678);
            corVecX.push_back(-0.1005); corVecY.push_back(0.2196); corVecZ.push_back(0.1259);
            corVecX.push_back(-0.0818); corVecY.push_back(0.1900); corVecZ.push_back(0.0816);
            corVecX.push_back(-0.0685); corVecY.push_back(0.1213); corVecZ.push_back(0.0354);
        }
    }

    vertical(0) = approach(1)*normal(2) - approach(2)*normal(1);
    vertical(1) = approach(2)*normal(0) - approach(0)*normal(2);
    vertical(2) = approach(0)*normal(1) - approach(1)*normal(0);

    Eigen::Affine3d meshPose;

    meshPose(0, 0) = approach(0); meshPose(0, 1) = normal(0); meshPose(0, 2) = vertical(0); meshPose(0, 3) = tgx;
    meshPose(1, 0) = approach(1); meshPose(1, 1) = normal(1); meshPose(1, 2) = vertical(1); meshPose(1, 3) = tgy;
    meshPose(2, 0) = approach(2); meshPose(2, 1) = normal(2); meshPose(2, 2) = vertical(2); meshPose(2, 3) = tgz;
    meshPose(3, 0) = 0; meshPose(3, 1) = 0; meshPose(3, 2) = 0; meshPose(3, 3) = 1;

    ROS_INFO("+++Collision check with mesh %s.", meshName.c_str());
    ros::NodeHandle node_handle;
    ros::ServiceClient client_CollideMesh = node_handle.serviceClient<armsweep::CollideMesh>("CollideMesh");
    armsweep::CollideMesh srv_CollideMesh;
    srv_CollideMesh.request.meshNames.clear(); srv_CollideMesh.request.meshNames.push_back(meshName.c_str());
    srv_CollideMesh.request.poses.clear(); srv_CollideMesh.request.poses.resize(1);
    tf::poseEigenToMsg(meshPose, srv_CollideMesh.request.poses[0]);
    armsweep::AllowedCollisions acmMsg;
    acmMsg.linkGroupNames.clear(); acmMsg.linkGroupNames.push_back("right_arm");
    acmMsg.linkNames.clear();
    acmMsg.objectNames.clear(); acmMsg.objectNames = req.allowedCollisions;
    srv_CollideMesh.request.allowedCollisions.clear(); srv_CollideMesh.request.allowedCollisions.push_back(acmMsg);
    bool freeCorridor = false;
    if (client_CollideMesh.call(srv_CollideMesh))
    {
      int maxK = srv_CollideMesh.response.errcodes.size();
      ROS_INFO("CollideMesh received %d answers: ", maxK);
      for(int k = 0 ; k < maxK; k++)
      {
          ROS_INFO("    errcode %d", srv_CollideMesh.response.errcodes[k]);
          ROS_INFO("    msg: %s", srv_CollideMesh.response.errmsgs[k].c_str());
          ROS_INFO("    inFreespace: %d", srv_CollideMesh.response.inFreespace[k]);
          freeCorridor = srv_CollideMesh.response.inFreespace[k];
      }
    }
    else
    {
      ROS_ERROR("Failed to call service CollideMesh");
    }

    if(freeCorridor)
    {
        ROS_INFO("    Free corridor, attempt IK ...");
        //generate IK trajectory from cr. end effector to refPose
        std::vector<robot_state::RobotStatePtr> traj; traj.clear();
        std::vector<robot_state::RobotStatePtr> retTraj; retTraj.clear();
        robot_state::RobotState wstate(contextP->robotPlanningScene->getCurrentState());
        wstate.getJointModelGroup("right_arm")->printGroupInfo();
        ROS_INFO("    current r_wrist_roll_link (base frame)");
        Eigen::Affine3d cPose = wstate.getGlobalLinkTransform("r_wrist_roll_link");
        ROS_INFO("        %f    %f    %f    %f", cPose(0, 0), cPose(0, 1), cPose(0, 2), cPose(0, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(1, 0), cPose(1, 1), cPose(1, 2), cPose(1, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(2, 0), cPose(2, 1), cPose(2, 2), cPose(2, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(3, 0), cPose(3, 1), cPose(3, 2), cPose(3, 3));
        //ROS_INFO("    <dummy set to cr pos from IK: %d>", wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1));
        ROS_INFO("    reference r_wrist_roll_link (base frame)");
        cPose = /*wstate.getGlobalLinkTransform("torso_lift_link").inverse()*cPose*/refPose_baseFrame;
        ROS_INFO("        %f    %f    %f    %f", cPose(0, 0), cPose(0, 1), cPose(0, 2), cPose(0, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(1, 0), cPose(1, 1), cPose(1, 2), cPose(1, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(2, 0), cPose(2, 1), cPose(2, 2), cPose(2, 3));
        ROS_INFO("        %f    %f    %f    %f", cPose(3, 0), cPose(3, 1), cPose(3, 2), cPose(3, 3));
        //ROS_INFO("    <dummy set to cr pos from IK: %d>", wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1));
        robot_state::RobotState tstate = wstate;
        robot_state::RobotState cstate = wstate;
        tstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1);
        wstate.interpolate(tstate, 0.2, cstate);
        traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
        wstate.interpolate(tstate, 0.4, cstate);
        traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
        wstate.interpolate(tstate, 0.6, cstate);
        traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
        wstate.interpolate(tstate, 0.8, cstate);
        traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
        traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(tstate))));
        wstate = tstate;
        double done = 1.0;
        if(0.9 < done)
        {
            Eigen::Affine3d cPose = approachPose_torsoFrame;
            cPose(0, 3) += corVecX[0];
            cPose(1, 3) += corVecY[0];
            cPose(2, 3) += corVecZ[0];
            cPose = torsoPose*cPose;
            ROS_INFO("    entry r_wrist_roll_link (base frame)");
            ROS_INFO("        %f    %f    %f    %f", cPose(0, 0), cPose(0, 1), cPose(0, 2), cPose(0, 3));
            ROS_INFO("        %f    %f    %f    %f", cPose(1, 0), cPose(1, 1), cPose(1, 2), cPose(1, 3));
            ROS_INFO("        %f    %f    %f    %f", cPose(2, 0), cPose(2, 1), cPose(2, 2), cPose(2, 3));
            ROS_INFO("        %f    %f    %f    %f", cPose(3, 0), cPose(3, 1), cPose(3, 2), cPose(3, 3));
            //ROS_INFO("    <dummy set to cr pos from IK: %d>", wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1));
            tstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1);
            wstate.interpolate(tstate, 0.2, cstate);
            traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
            wstate.interpolate(tstate, 0.4, cstate);
            traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
            wstate.interpolate(tstate, 0.6, cstate);
            traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
            wstate.interpolate(tstate, 0.8, cstate);
            traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(cstate))));
            traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(tstate))));
            wstate = tstate;
            if(0.9 < done)
            {
                bool allOk = true;
                for(int k = 0; (k < corVecX.size()) && allOk; k++)
                {
                    Eigen::Affine3d cPose;
                    cPose = approachPose_torsoFrame;
                    cPose(0, 3) += corVecX[k];
                    cPose(1, 3) += corVecY[k];
                    cPose(2, 3) += corVecZ[k];
                    cPose = torsoPose*cPose;
                    //ROS_INFO("    corridor r_wrist_roll_link (base frame) at waypoint %d", k);
                    //ROS_INFO("        %f    %f    %f    %f", cPose(0, 0), cPose(0, 1), cPose(0, 2), cPose(0, 3));
                    //ROS_INFO("        %f    %f    %f    %f", cPose(1, 0), cPose(1, 1), cPose(1, 2), cPose(1, 3));
                    //ROS_INFO("        %f    %f    %f    %f", cPose(2, 0), cPose(2, 1), cPose(2, 2), cPose(2, 3));
                    //ROS_INFO("        %f    %f    %f    %f", cPose(3, 0), cPose(3, 1), cPose(3, 2), cPose(3, 3));
                    allOk = wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1);
                    //ROS_INFO("        <dummy set to wp pos from IK: %d>", wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1));
                    if(!allOk)
                        ROS_INFO("        IK error on in-corridor waypoint %d", k);
                    traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(wstate))));
                    retTraj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(wstate))));
                }
                if(allOk)
                {
                    Eigen::Affine3d cPose = targetPose_torsoFrame;
                    cPose = torsoPose*cPose;
                    allOk = wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1);
                    //ROS_INFO("        <dummy set to wp pos from IK: %d>", wstate.setFromIK(wstate.getJointModelGroup("right_arm"), cPose, 10, 0.1));
                    if(!allOk)
                        ROS_INFO("        IK error at target");
                    traj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(wstate))));
                    retTraj.push_back(*(new robot_state::RobotStatePtr(new robot_state::RobotState(wstate))));
                }
                if(allOk)
                {
                    res.foundTrajectory = 1;
                    for(int k = 0; k < traj.size(); k++)
                    {
                        moveit_msgs::RobotState rSMsg;
                        robot_state::robotStateToRobotStateMsg(*(traj[k]), rSMsg, false);
                        res.approach.push_back(rSMsg);
                    }
                    for(int k = 0; k < retTraj.size(); k++)
                    {
                        moveit_msgs::RobotState rSMsg;
                        robot_state::robotStateToRobotStateMsg(*(retTraj[retTraj.size() - 1 - k]), rSMsg, false);
                        res.goBack.push_back(rSMsg);
                    }
                }
            }
            else
            {
                ROS_INFO("        IK error on path to entry configuration; fraction done %f", done);
            }
        }
        else
        {
            ROS_INFO("        IK error on path to reference configuration; fraction done %f", done);
        }

    }

    return true;
}

bool armsweep::loadMesh(armsweep::ArmsweepContext *contextP, armsweep::LoadMesh::Request  &req, armsweep::LoadMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    if(req.meshNames.size() != req.meshURLs.size())
    {
        res.errcodes.resize(1);
        res.errmsgs.resize(1);
        res.errcodes[0] = armsweep::AS_MalformedRequestLengths;
        armsweep::getStrMsg(armsweep::AS_MalformedRequestLengths, "", "", res.errmsgs[0]);
        return true;
    }
    res.errcodes.resize(req.meshNames.size());
    res.errmsgs.resize(req.meshNames.size());
    for(int k = 0; k < req.meshNames.size(); k++)
    {
        loadMesh_internal(req.meshNames[k], req.meshURLs[k], context, res.errcodes[k], res.errmsgs[k]);
    }
    return true;
}

bool armsweep::listMesh(armsweep::ArmsweepContext *contextP, armsweep::ListMesh::Request  &req, armsweep::ListMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    res.errcode = armsweep::AS_AllOk;
    armsweep::getStrMsg(armsweep::AS_AllOk, "", "", res.errmsg);
    armsweep::ItemMap::iterator it;
    for(it = context.items.begin(); it != context.items.end(); ++it)
    {
        res.meshNames.push_back((*it).first);
    }
    return true;
}

bool armsweep::unloadMesh(armsweep::ArmsweepContext *contextP, armsweep::UnloadMesh::Request  &req, armsweep::UnloadMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    std::vector<std::string> unvisualizedMeshNames;
    std::vector<std::string> unloadedMeshes;
    int maxK = req.meshNames.size();
    unvisualizedMeshNames.clear();
    unvisualizedMeshNames.reserve(maxK);
    unloadedMeshes.clear();
    unloadedMeshes.reserve(maxK);
    res.errcodes.resize(maxK);
    res.errmsgs.resize(maxK);
    for(int k = 0; k < maxK; k++)
    {
        if(unloadMesh_internal(req.meshNames[k], context, res.errcodes[k], res.errmsgs[k], unvisualizedMeshNames))
        {
            unloadedMeshes.push_back(req.meshNames[k]);
        }
    }
    unvisualizer(context, unvisualizedMeshNames);
    maxK = unloadedMeshes.size();
    for(int k = 0; k < maxK; k++)
    {
        context.items.erase(unloadedMeshes[k]);
    }
    return true;
}

bool armsweep::collideMesh(armsweep::ArmsweepContext *contextP, armsweep::CollideMesh::Request  &req, armsweep::CollideMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    moveit::tools::Profiler::Clear();
    moveit::tools::Profiler::Start();
    if((req.meshNames.size() != req.poses.size()) || (req.meshNames.size() != req.allowedCollisions.size()))
    {
        moveit::tools::Profiler::Stop();
        res.duration = 0.0;
        res.errcodes.resize(1);
        res.errmsgs.resize(1);
        res.errcodes[0] = armsweep::AS_MalformedRequestLengths;
        armsweep::getStrMsg(armsweep::AS_MalformedRequestLengths, "", "", res.errmsgs[0]);
        return true;
    }

    moveit::tools::Profiler::Begin("Aux vector init");

    int maxK = req.meshNames.size();
    res.errcodes.resize(req.meshNames.size());
    res.errmsgs.resize(req.meshNames.size());
    res.inFreespace.resize(req.meshNames.size());
    res.distances.resize(req.meshNames.size());

    std::vector<std::string> meshesForCollide;
    std::vector<Eigen::Affine3d> posesForCollide;
    std::vector<int> indicesForCollide;
    std::vector<armsweep::ItemMap::iterator> iteratorsForCollide;
    StrVecVec allowedForCollide;

    meshesForCollide.clear(); meshesForCollide.reserve(maxK);
    posesForCollide.clear(); posesForCollide.reserve(maxK);
    allowedForCollide.clear(); allowedForCollide.reserve(maxK);
    indicesForCollide.clear(); indicesForCollide.reserve(maxK);
    iteratorsForCollide.clear(); iteratorsForCollide.reserve(maxK);

    std::vector<std::string> auxStrVec;

    moveit::tools::Profiler::End("Aux vector init");

    moveit::tools::Profiler::Begin("collideMesh_internal");
    for(int k = 0; k < req.meshNames.size(); k++)
    {
        armsweep::ItemMap::iterator it;
        if(collideMesh_internal(req.meshNames[k], req.poses[k], context, res.errcodes[k], res.errmsgs[k], it, auxStrVec, req.allowedCollisions[k]))
        {
            Eigen::Affine3d auxEigen;
            meshesForCollide.push_back(req.meshNames[k]);
            tf::poseMsgToEigen(req.poses[k], auxEigen);
            posesForCollide.push_back(auxEigen);
            allowedForCollide.push_back(auxStrVec);
            indicesForCollide.push_back(k);
            iteratorsForCollide.push_back(it);
        }
    }

    for(int k = 0; k < indicesForCollide.size(); k++)
    {
        res.inFreespace[indicesForCollide[k]] = 1;
        res.distances[indicesForCollide[k]] = -10.0;
    }
    moveit::tools::Profiler::End("collideMesh_internal");

    moveit::tools::Profiler::Begin("collideWithWorld");
    collideWithWorld(context, iteratorsForCollide, meshesForCollide, posesForCollide, allowedForCollide, indicesForCollide, res);
    moveit::tools::Profiler::End("collideWithWorld");
    moveit::tools::Profiler::Begin("collideWithRobot");
    collideWithRobot(context, iteratorsForCollide, meshesForCollide, posesForCollide, allowedForCollide, indicesForCollide, res);
    moveit::tools::Profiler::End("collideWithRobot");

    moveit::tools::Profiler::Stop();
    moveit::tools::Profiler::Console();
    //TODO: put the real duration here; parse it from Profiler::Status output;
    res.duration = 0.0;
    moveit::tools::Profiler::Clear();

    return true;
}

bool armsweep::visualizeMesh(armsweep::ArmsweepContext *contextP, armsweep::VisualizeMesh::Request  &req, armsweep::VisualizeMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    ros::WallDuration sleep_t(0.2);
    if(req.meshNames.size() != req.poses.size())
    {
        res.errcodes.resize(1);
        res.errmsgs.resize(1);
        res.errcodes[0] = armsweep::AS_MalformedRequestLengths;
        armsweep::getStrMsg(armsweep::AS_MalformedRequestLengths, "", "", res.errmsgs[0]);
        return true;
    }
    res.errcodes.resize(req.meshNames.size());
    res.errmsgs.resize(req.meshNames.size());
    res.instanceNames.resize(req.meshNames.size());
    std::vector<std::string> visualizedMeshNames;
    std::vector<shapes::Mesh const*> visualizedMeshes;
    std::vector<geometry_msgs::Pose> visualizedPoses;
    visualizedMeshes.clear(); visualizedMeshes.reserve(req.meshNames.size());
    visualizedMeshNames.clear(); visualizedMeshNames.reserve(req.meshNames.size());
    visualizedPoses.clear(); visualizedPoses.reserve(req.meshNames.size());
    for(int k = 0; k < req.meshNames.size(); k++)
    {
        shapes::Mesh const*meshPtr;
        if(visualizeMesh_internal(req.meshNames[k], req.poses[k], context, res.errcodes[k], res.errmsgs[k], res.instanceNames[k], meshPtr))
        {
            visualizedMeshNames.push_back(res.instanceNames[k]);
            visualizedMeshes.push_back(meshPtr);
            visualizedPoses.push_back(req.poses[k]);
        }
    }
    int maxK = visualizedMeshes.size();
    std::vector<std::string> robotLinkNames;
    robotLinkNames.clear();
    getRobotLinkNames(context, robotLinkNames);

    moveit_msgs::PlanningScene planning_scene_diff;
    geometry_msgs::TransformStamped transformStampedMsg;
    transformStampedMsg.header.frame_id = "odom_combined";
    planning_scene_diff.world.collision_objects.resize(maxK);
    planning_scene_diff.is_diff = true;
    SetEntriesInACM(visualizedMeshNames, robotLinkNames, context.robotPlanningScene);
    context.robotPlanningScene->getAllowedCollisionMatrix().getMessage(planning_scene_diff.allowed_collision_matrix);

    for(int k = 0; k < maxK; k++)
    {
        moveit_msgs::CollisionObject &add_object(planning_scene_diff.world.collision_objects[k]);
        add_object.meshes.clear();
        add_object.mesh_poses.clear();
        add_object.id = visualizedMeshNames[k];
        add_object.header.frame_id = "odom_combined";
        add_object.operation = add_object.ADD;
        shape_msgs::Mesh meshMsg;
        meshToMsg(visualizedMeshes[k], &meshMsg);
        add_object.meshes.push_back(meshMsg);
        add_object.mesh_poses.push_back(visualizedPoses[k]);
        context.collisionObjectPublisher.publish(add_object);
    }
    sleep_t.sleep();

    context.planningSceneDiffPublisher.publish(planning_scene_diff);
    sleep_t.sleep();

    return true;
}

bool armsweep::unvisualizeMesh(armsweep::ArmsweepContext *contextP, armsweep::UnvisualizeMesh::Request  &req, armsweep::UnvisualizeMesh::Response &res)
{
    armsweep::ArmsweepContext &context(*contextP);
    std::vector<std::string> unvisualizedMeshNames;
    std::vector<int> unvisualizedMeshInstanceIds;
    int maxK = req.meshNames.size();
    res.errcodes.resize(maxK);
    res.errmsgs.resize(maxK);
    unvisualizedMeshNames.reserve(maxK);
    unvisualizedMeshInstanceIds.reserve(maxK);
    for(int k = 0; k < maxK; k++)
    {
        std::string auxName;
        int auxInt;
        if(unvisualizeMesh_internal(req.meshNames[k], context, res.errcodes[k], res.errmsgs[k], auxName, auxInt))
        {
            unvisualizedMeshNames.push_back(req.meshNames[k]);
            unvisualizedMeshInstanceIds.push_back(auxInt);
        }
    }
    unvisualizer(context, unvisualizedMeshNames);
    return true;
}

