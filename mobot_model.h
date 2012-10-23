#ifndef _MOBOT_MODEL_H_
#define _MOBOT_MODEL_H_

#include <ode/ode.h>
#include "pandaFramework.h"
#include "pandaSystem.h"
#include "mobot_specs.h"

#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.0)

class MobotModel
{
  public:
    MobotModel(WindowFramework* window, PandaFramework* framework, dWorldID world, dSpaceID space);
    ~MobotModel();
    void update();
    dBodyID build_faceplate1(dReal x, dReal y, dReal z, LQuaternionf rot);
    dBodyID build_body1(dReal x, dReal y, dReal z, LQuaternionf rot);
    void build_center(dReal x, dReal y, dReal z, LQuaternionf rot);
    void build_mobot(dReal x, dReal y, dReal z, LQuaternionf rot);
    const dReal* get_position(int index);
  private:
    WindowFramework* _window;
    PandaFramework* _framework;
    dWorldID _world;
    dSpaceID _space;
    dBodyID _odeBodies[100];
    NodePath _nodePaths[100];
    int _numBodies;
};


#endif
