#ifndef __ARMSWEEP_STRMSGUTILS_H__

#define __ARMSWEEP_STRMSGUTILS_H__

#include <string>

#include <armsweep/armsweep.h>

namespace armsweep
{

void getStrMsg(ArmsweepErrcode errcode, std::string const&resName, std::string const&resURL, std::string &where);
extern const std::string standardVizPrefix;
extern const std::string standardTrialPrefix;
}

#endif
