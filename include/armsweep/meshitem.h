#ifndef __ARMSWEEP_MESHITEM_H__

#define __ARMSWEEP_MESHITEM_H__

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <eigen_conversions/eigen_msg.h>
#include <shape_msgs/Mesh.h>

#include <map>
#include <string>
#include <vector>
#include <utility>

namespace armsweep
{

typedef boost::shared_ptr<shapes::Mesh const> MeshConstPtr;

class MeshItem
{
    public:
        MeshItem(armsweep::MeshConstPtr const&mesh, std::string const&resName, std::string const&resURL);
        MeshItem(MeshItem const&orig);
        MeshItem &operator=(MeshItem const&orig);
        ~MeshItem();

        MeshConstPtr mesh;
        std::string resName, resURL;
        std::vector<int> displayedInstances, internalInstances;
        int displayedLastInstance;
        int internalLastInstance;
    private:
        MeshItem();
};

typedef std::map<std::string, MeshItem> ItemMap;
typedef std::pair<std::string, MeshItem> ItemMapElement;

typedef std::vector<std::vector<std::string> > StrVecVec;

}

#endif
