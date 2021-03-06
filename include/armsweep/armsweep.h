#ifndef __ARMSWEEP_ARMSWEEP_H__

#define __ARMSWEEP_ARMSWEEP_H__

#include "armsweep/LoadMesh.h"
#include "armsweep/ListMesh.h"
#include "armsweep/UnloadMesh.h"
#include "armsweep/CollideMesh.h"
#include "armsweep/VisualizeMesh.h"
#include "armsweep/UnvisualizeMesh.h"
#include "armsweep/AllowedCollisions.h"
#include "armsweep/PickTest.h"

namespace armsweep
{

enum ArmsweepErrcode{
    AS_AllOk,
    AS_CannotLoad,
    AS_NotLoaded,
    AS_AlreadyLoaded,
    AS_MalformedRequestLengths,
    AS_NotAnArmsweepId,
    AS_NotAnInstance
};

}

#endif
