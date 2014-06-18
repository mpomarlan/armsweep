#include <armsweep/strmsgutils.h>

namespace armsweep
{

void getAllOkMsg(std::string &where)
{
    where = "All ok.";
}

void getCannotLoadMsg(std::string const&resName, std::string const&resURL, std::string &where)
{
    where = "Cannot load mesh ";
    where = where + resName + " from resource " + resURL + " :: check that file exists and that is is a mesh that assimp can load.";
}

void getNotLoadedMsg(std::string const&resName, std::string &where)
{
    where = "Mesh ";
    where = where + resName + " isn't loaded :: load mesh first before attempting queries on it.";
}

void getAlreadyLoadedMsg(std::string const&resName, std::string const&resURL, std::string &where)
{
    where = "Mesh ";
    where = where + resName + " is already loaded from resource " + resURL + " :: unload a mesh before attempting to replace it with a reload.";
}

void getMalformedRequestLengths(std::string &where)
{
    where = "Malformed request lengths: items that should have the same length do not.";
}

void getNotAnArmsweepId(std::string const&resName, std::string &where)
{
    where = "Mesh id ";
    where = where + resName + " does not look like an armsweep id. A proper armsweep mesh id is " + standardVizPrefix + "<mesh name>_<instance number> :: make sure you use the ids returned by the VisualizeMesh service.";
}

void getNotAnInstance(std::string const&resName, std::string &where)
{
    where = "There is no mesh instance corresponding to ";
    where = where + resName + " . The mesh instance name is an arsweep mesh id, and the required mesh is loaded, but the instance number is not existing. Perhaps it was already unloaded?";
}

void getFunkyMessage(std::string &where)
{
    where = "Unknown errorcode encountered :: check the function calls in the sourcecode :P";
}

void getStrMsg(ArmsweepErrcode errcode, std::string const&resName, std::string const&resURL, std::string &where)
{
    switch(errcode)
    {
        case AS_AllOk:
            getAllOkMsg(where);
            break;
        case AS_CannotLoad:
            getCannotLoadMsg(resName, resURL, where);
            break;
        case AS_NotLoaded:
            getNotLoadedMsg(resName, where);
            break;
        case AS_AlreadyLoaded:
            getAlreadyLoadedMsg(resName, resURL, where);
            break;
        case AS_MalformedRequestLengths:
            getMalformedRequestLengths(where);
            break;
        case AS_NotAnArmsweepId:
            getNotAnArmsweepId(resName, where);
            break;
        case AS_NotAnInstance:
            getNotAnInstance(resName, where);
            break;
        default:
            getFunkyMessage(where);
    }
}

std::string const standardVizPrefix("armsweepViz_");
std::string const standardTrialPrefix("armsweepTrial_");

}
