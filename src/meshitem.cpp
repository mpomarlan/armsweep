#include <armsweep/armsweep.h>
#include <armsweep/meshitem.h>

armsweep::MeshItem::MeshItem(armsweep::MeshConstPtr const&mesh, std::string const&resName, std::string const&resURL):
    mesh(mesh),
    resName(resName),
    resURL(resURL),
    displayedInstances(),
    internalInstances(),
    displayedLastInstance(0),
    internalLastInstance(0)
{
}
armsweep::MeshItem::MeshItem(MeshItem const&orig):
    mesh(orig.mesh),
    resName(orig.resName),
    resURL(orig.resURL),
    displayedInstances(orig.displayedInstances),
    internalInstances(orig.internalInstances),
    displayedLastInstance(orig.displayedLastInstance),
    internalLastInstance(orig.internalLastInstance)
{
}
armsweep::MeshItem &armsweep::MeshItem::operator=(MeshItem const&orig)
{
    mesh = (orig.mesh);
    resName = orig.resName;
    resURL = orig.resURL;
    displayedInstances = orig.displayedInstances;
    internalInstances = orig.internalInstances;
    displayedLastInstance = orig.displayedLastInstance;
    internalLastInstance = orig.internalLastInstance;
    return (*this);
}

armsweep::MeshItem::MeshItem():
    mesh(new shapes::Mesh()),
    resName(""),
    resURL(""),
    displayedInstances(),
    internalInstances(),
    displayedLastInstance(0),
    internalLastInstance(0)
{
}

armsweep::MeshItem::~MeshItem()
{
}
