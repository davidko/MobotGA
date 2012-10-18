#ifndef _MOBOT_MODEL_H_
#define _MOBOT_MODEL_H_

#include "pandaFramework.h"
#include "pandaSystem.h"
#include "mobot_specs.h"
#include "odeWorld.h"
#include "odeBody.h"
#include "odeMass.h"
#include "odeSpace.h"
#include "odeBoxGeom.h"
#include "odeCylinderGeom.h"

class MobotModel
{
  public:
    MobotModel(WindowFramework* window, PandaFramework* framework);
    ~MobotModel();
    void update();
    void build_faceplate1(OdeWorld* world, OdeSpace* space, dReal x, dReal y, dReal z, LQuaternionf rot);
    LVector3f get_position(int index);
  private:
    WindowFramework* _window;
    PandaFramework* _framework;
    OdeBody *_odeBodies[100];
    NodePath _nodePaths[100];
    int _numBodies;
};


#endif
