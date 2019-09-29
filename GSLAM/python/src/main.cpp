#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Registry.h>
#include "GSLAM_Module.h"
#include "SvarPy.h"

using namespace GSLAM;

PyObject* load(std::string pluginPath){
    return SvarPy::getModule(Registry::load(pluginPath));
}

SVAR_PYTHON_IMPL(gslam)
{
    PyEval_InitThreads();
    svar.set("load",Svar(load));
    return SvarPy::getModule(svar);
}
