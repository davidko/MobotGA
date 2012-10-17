#ifndef _MOBOT_MODEL_H_
#define _MOBOT_MODEL_H_

#include "mobot_specs.h"
#include "odeWorld.h"
#include "odeBody.h"
#include "odeMass.h"
#include "odeSpace.h"
#include "odeBoxGeom.h"
#include "odeCylinderGeom.h"

OdeBody* build_faceplate1(OdeWorld* world, OdeSpace* space, dReal x, dReal y, dReal z, LQuaternionf rot);

#endif
