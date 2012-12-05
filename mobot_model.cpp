#include "mobot_model.h"
#include "main.h"

MobotModel::MobotModel(WindowFramework* window, PandaFramework* framework, dWorldID world, dSpaceID space)
{
  _window = window;
  _framework = framework;
  _numBodies = 0;
  _world = world;
  _space = space;
  _numBodies = 0;
  _damping = 0.00000022477;
  _maxTorque = 0.000001;
  /*
  _desiredAngles[0] = DEG2RAD(45);
  _desiredAngles[1] = DEG2RAD(45);
  _desiredAngles[2] = DEG2RAD(45);
  _desiredAngles[3] = DEG2RAD(45);
  */
  _desiredAngles[0] = DEG2RAD(0);
  _desiredAngles[1] = DEG2RAD(0);
  _desiredAngles[2] = DEG2RAD(0);
  _desiredAngles[3] = DEG2RAD(0);
  /* Set up fourier series coefficients */
  FILE *fp;
  fp = fopen("/tmp/fourier_coefs.txt", "a");
  int i, j;
  for(i = 0; i < 4; i++) {
    for(j = 0; j < 5; j++) {
      _a[i][j] = rand() % 64 + 128 - 32;
      _b[i][j] = rand() % 64 + 128 - 32;
    }
  }
  for(i = 0; i < 4; i++) {
    //printf("Joint %d\n", i+1);
    for(j = 0; j < 5; j++) {
      fprintf(fp, "%lf\n", C2V(_a[i][j]));
    }
    for(j = 0; j < 5; j++) {
      fprintf(fp, "%lf\n", C2V(_b[i][j]));
    }
  }
  fclose(fp);
}

MobotModel::~MobotModel()
{
}

void MobotModel::update()
{
  int i;
  const dReal* position;
  const dReal* quaternion;
  if(_numBodies == 0) {
    return;
  }
  for(i = 0; i < _numBodies; i++) {
    position = dBodyGetPosition(_odeBodies[i]);
    quaternion = dBodyGetQuaternion(_odeBodies[i]);
    _nodePaths[i].set_pos_quat(
        _window->get_render(),
        LVector3f(position[0], position[1], position[2]),
        LQuaternionf(
          quaternion[0], 
          quaternion[1], 
          quaternion[2], 
          quaternion[3])
        );
  }
}

void MobotModel::step(double time)
{
  /* Add damping forces */
  int i,j;
  dReal omega;
  dReal err;
  double d;
  double t;
  if(time == -1) {
    t = globalClock->get_real_time();
  } else {
    t = time;
  }

  for(i = 0; i < 4; i++) {
    if(i == 0 || i == 2) {
      d = -1;
    } else {
      d = 1;
    }
    /* Calculate the desired angles based on fourier series */
    _desiredAngles[i] = (dReal)0.5*C2V(_a[i][0]);
    for(j = 1; j < 5; j++) {
      _desiredAngles[i] += (dReal)C2V(_a[i][j]) * (dReal)sin(2*M_PI*t*j/10.0);
      _desiredAngles[i] += (dReal)C2V(_b[i][j]) * (dReal)cos(2*M_PI*t*j/10.0);
      //printf("%lf\n", C2V(a[i][j]));
    }
    if(i == 1 || i == 2) {
      if(_desiredAngles[i] > M_PI/2.0) {
        _desiredAngles[i] = M_PI/2.0;
      }
      if(_desiredAngles[i] < -M_PI/2.0) {
        _desiredAngles[i] = -M_PI/2.0;
      }
    }
    /*
    omega = dJointGetHingeAngleRate(_joints[i]);
    printf("%lf\n", omega);
    dJointAddHingeTorque(_joints[i], -omega*_damping);
    */
    /* PID control */
    err = _desiredAngles[i] - d*dJointGetHingeAngle(_joints[i]);
    //printf("%lf - %lf = %lf\n", _desiredAngles[i], dJointGetHingeAngle(_joints[i]), err);
    err *= 5;
    if(err > M_PI) err = M_PI;
    if(err < -M_PI) err = -M_PI;
    dJointSetHingeParam(_joints[i], dParamFMax, .01);
    dJointSetHingeParam(_joints[i], dParamVel, d*err/1.0);
  }
  //printf("\n");
}

dBodyID MobotModel::build_faceplate1(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  if(gEnableGraphics) {
    /* Create the nodepath */
    NodePath node = _window->load_model(_framework->get_models(), "models/box");
    node.reparent_to(_window->get_render());
    node.set_pos(-0.5, -0.5, -0.5);
    node.flatten_light();
    //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_pos(x, y, z);
    node.set_color(0.9, 0.8, 0.8, 1);
    _nodePaths[_numBodies] = node;
  }
  dBodyID body = dBodyCreate(_world);
  dMass m;
  dMassSetBox(&m, FACEPLATE_M, FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  dBodySetMass(body, &m);
  dBodySetPosition(body, x, y, z);
  /* Set up the collision geometry */
  dGeomID geom;
  /* Simple box */
  /*
  geom = dCreateBox(_space, 
      FACEPLATE_X,
      FACEPLATE_Y,
      FACEPLATE_Z);
  dGeomSetBody(geom, body);
  */

  /* Box 1 */
  geom = dCreateBox(_space, 
      FACEPLATE_X - (2.0*FACEPLATE_R),
      FACEPLATE_Y,
      FACEPLATE_Z);
  dGeomSetBody(geom, body);

  /* Box 2 */
  geom = dCreateBox(_space, 
      FACEPLATE_R,
      FACEPLATE_Y*0.95,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      -(FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 - FACEPLATE_R/2.0,
      0,
      0);
      
  /* Box 3 */
  geom = dCreateBox(_space, 
      FACEPLATE_R,
      FACEPLATE_Y*0.95,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      (FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 + FACEPLATE_R/2.0,
      0,
      0);

  /* Cylinder 4 */
  geom = dCreateCylinder(_space, FACEPLATE_R*0.8, FACEPLATE_Y*0.9);
  dGeomSetBody(geom, body);
  dQuaternion q;
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      0,
      FACEPLATE_Z/2.0 - FACEPLATE_R);

  /* Cylinder 5 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_Y*.9);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      (FACEPLATE_X/2.0) - FACEPLATE_R,
      0,
      FACEPLATE_Z/2.0 - FACEPLATE_R);

  /* Cylinder 6 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_Y*.8);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      0,
      -FACEPLATE_Z/2.0 + FACEPLATE_R);

  /* Cylinder 7 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_Y*.8);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      +(FACEPLATE_X/2.0) - FACEPLATE_R,
      0,
      -FACEPLATE_Z/2.0 + FACEPLATE_R);

  dReal quat[4];
  quat[0] = rot.get_r();
  quat[1] = rot.get_i();
  quat[2] = rot.get_j();
  quat[3] = rot.get_k();
  dBodySetQuaternion(body, quat);
  
  _odeBodies[_numBodies] = body;
  _numBodies++;
  return body;
}

dBodyID MobotModel::build_body1(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  if(gEnableGraphics) {
    /* Create the nodepath */
    NodePath node = _window->load_model(_framework->get_models(), "models/box");
    node.reparent_to(_window->get_render());
    //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_pos(-0.5, -0.5, -0.5);
    node.flatten_light();
    node.set_scale(BODY_X, BODY_Y, BODY_Z);
    node.set_pos(
        BODY_BOX1_X/2.0 - (BODY_BOX2_X-BODY_CG_OFFSET),
        (BODY_Y/2.0 - BODY_BOX1_Y), 
        0);
    node.flatten_light();
    node.set_pos(x, y, z);
    node.set_color(0.8, 0.8, 0.9, 1);
    _nodePaths[_numBodies] = node;
  }

  dBodyID body = dBodyCreate(_world);
  dMass m;
  dMassSetBox(&m, BODY_M, BODY_X, BODY_Y, BODY_Z);
  dBodySetMass(body, &m);
  dBodySetPosition(body, x, y, z);
  /* Set up the collision geometry */
  dGeomID geom;

  /* Box 1 */
  geom = dCreateBox(_space, 
      BODY_BOX1_X,
      BODY_BOX1_Y,
      BODY_BOX1_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      BODY_BOX1_X/2.0 - (BODY_BOX2_X-BODY_CG_OFFSET),
      -BODY_BOX1_Y/2.0,
      0);

  /* Box 2 */
  geom = dCreateBox(_space, 
      BODY_BOX2_X,
      BODY_BOX2_Y,
      BODY_BOX2_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      BODY_BOX2_X-BODY_CG_OFFSET,
      BODY_BOX2_Y/2.0,
      0);

  /* Box 3 */
  geom = dCreateBox(_space, 
      BODY_BOX3_X,
      BODY_BOX3_Y,
      BODY_BOX3_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      (BODY_X - BODY_BOX3_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET),
      BODY_BOX3_Y/2.0,
      0);

  /* Cylinder 1 */
  geom = dCreateCylinder(_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  dGeomSetBody(geom, body);
  dQuaternion q;
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      -(BODY_CYL1_X/2.0 - BODY_CG_OFFSET),
      BODY_BOX2_Y,
      0);

  /* Cylinder 2 */
  geom = dCreateCylinder(_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      BODY_X - BODY_CYL2_X/2.0 - (BODY_BOX2_X - BODY_CG_OFFSET),
      BODY_BOX3_Y,
      0);

  dReal quat[4];
  quat[0] = rot.get_r();
  quat[1] = rot.get_i();
  quat[2] = rot.get_j();
  quat[3] = rot.get_k();
  dBodySetQuaternion(body, quat);
  
  _odeBodies[_numBodies] = body;
  _numBodies++;
  return body;
}

dBodyID MobotModel::build_body2(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  if(gEnableGraphics) {
    /* Create the nodepath */
    NodePath node = _window->load_model(_framework->get_models(), "models/box");
    node.reparent_to(_window->get_render());
    //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_pos(-0.5, -0.5, -0.5);
    node.flatten_light();
    node.set_scale(BODY_X, BODY_Y, BODY_Z);
    node.set_pos(
        BODY_BOX1_X/2.0 - (BODY_BOX2_X-BODY_CG_OFFSET),
        -(BODY_Y/2.0 - BODY_BOX1_Y), 
        0);
    node.flatten_light();
    node.set_pos(x, y, z);
    node.set_color(0.9, 0.9, 0.8, 1);
    _nodePaths[_numBodies] = node;
  }

  dBodyID body = dBodyCreate(_world);
  dMass m;
  dMassSetBox(&m, BODY_M, BODY_X, BODY_Y, BODY_Z);
  dBodySetMass(body, &m);
  dBodySetPosition(body, x, y, z);
  /* Set up the collision geometry */
  dGeomID geom;

  /* Box 1 */
  geom = dCreateBox(_space, 
      BODY_BOX1_X,
      BODY_BOX1_Y,
      BODY_BOX1_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      BODY_BOX1_X/2.0 - (BODY_BOX2_X-BODY_CG_OFFSET),
      BODY_BOX1_Y/2.0,
      0);

  /* Box 2 */
  geom = dCreateBox(_space, 
      BODY_BOX2_X,
      BODY_BOX2_Y,
      BODY_BOX2_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      BODY_BOX2_X-BODY_CG_OFFSET,
      -BODY_BOX2_Y/2.0,
      0);

  /* Box 3 */
  geom = dCreateBox(_space, 
      BODY_BOX3_X,
      BODY_BOX3_Y,
      BODY_BOX3_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      (BODY_X - BODY_BOX3_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET),
      -BODY_BOX3_Y/2.0,
      0);

  /* Cylinder 1 */
  geom = dCreateCylinder(_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  dGeomSetBody(geom, body);
  dQuaternion q;
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      -(BODY_CYL1_X/2.0 - BODY_CG_OFFSET),
      -BODY_BOX2_Y,
      0);

  /* Cylinder 2 */
  geom = dCreateCylinder(_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      BODY_X - BODY_CYL2_X/2.0 - (BODY_BOX2_X - BODY_CG_OFFSET),
      -BODY_BOX3_Y,
      0);

  dReal quat[4];
  quat[0] = rot.get_r();
  quat[1] = rot.get_i();
  quat[2] = rot.get_j();
  quat[3] = rot.get_k();
  dBodySetQuaternion(body, quat);
  
  _odeBodies[_numBodies] = body;
  _numBodies++;
  return body;
}

dBodyID MobotModel::build_center(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  if(gEnableGraphics) {
    /* Create the nodepath */
    NodePath node = _window->load_model(_framework->get_models(), "models/box");
    node.reparent_to(_window->get_render());
    //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_pos(-0.5, -0.5, -0.5);
    node.flatten_light();
    node.set_scale(CENTER_X, CENTER_Y, CENTER_Z);
    node.set_pos(x, y, z);
    node.set_color(0.9, 0.8, 0.9, 1);
    _nodePaths[_numBodies] = node;
  }

  dBodyID body = dBodyCreate(_world);
  dMass m;
  dMassSetBox(&m, CENTER_M, CENTER_X, CENTER_Y, CENTER_Z);
  dBodySetMass(body, &m);
  dBodySetPosition(body, x, y, z);

  /* Set up collision geometry */
  dGeomID geom;

  /* Box 1 */
  geom = dCreateBox(_space, 
      CENTER_X,
      CENTER_Y - (CENTER_R)*2.0,
      CENTER_Z);
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      0,
      0,
      0);

  /* Cyl 1 */
  geom = dCreateCylinder(_space,
      CENTER_R*0.8,
      CENTER_X);
  dGeomSetBody(geom, body);
  dQuaternion q;
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      0,
      -(CENTER_Y/2.0),
      0);

  /* Cyl 1 */
  geom = dCreateCylinder(_space,
      CENTER_R*0.8,
      CENTER_X);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 0, 1, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom, 
      0,
      (CENTER_Y/2.0),
      0);

  _odeBodies[_numBodies] = body;
  _numBodies++;

  return body;
}

dBodyID MobotModel::build_big_faceplate(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  if(gEnableGraphics) {
    /* Create the nodepath */
    NodePath node = _window->load_model(_framework->get_models(), "models/box");
    node.reparent_to(_window->get_render());
    node.set_pos(-0.5, -0.5, -0.5);
    node.flatten_light();
    //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
    node.set_scale(FACEPLATE_X, FACEPLATE_COMPOUND_Y, FACEPLATE_Z);
    node.set_pos(x, y, z);
    node.set_color(0.8, 0.9, 0.9, 1);
    _nodePaths[_numBodies] = node;
  }

  dBodyID body = dBodyCreate(_world);
  dMass m;
  dMassSetBox(&m, FACEPLATE_M*3.0, FACEPLATE_X, FACEPLATE_COMPOUND_Y, FACEPLATE_Z);
  dBodySetMass(body, &m);
  dBodySetPosition(body, x, y, z);
  /* Set up the collision geometry */
  dGeomID geom;
  /* Simple box */
  /*
  geom = dCreateBox(_space, 
      FACEPLATE_X,
      FACEPLATE_Y,
      FACEPLATE_Z);
  dGeomSetBody(geom, body);
  */

  /* Box 1 */
  geom = dCreateBox(_space, 
      FACEPLATE_X - (2.0*FACEPLATE_R),
      FACEPLATE_COMPOUND_Y,
      FACEPLATE_Z);
  dGeomSetBody(geom, body);

  /* Box 2 */
  geom = dCreateBox(_space, 
      FACEPLATE_R,
      FACEPLATE_COMPOUND_Y*0.95,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      -(FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 - FACEPLATE_R/2.0,
      0,
      0);
      
  /* Box 3 */
  geom = dCreateBox(_space, 
      FACEPLATE_R,
      FACEPLATE_COMPOUND_Y*0.95,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  dGeomSetBody(geom, body);
  dGeomSetOffsetPosition(geom, 
      (FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 + FACEPLATE_R/2.0,
      0,
      0);

  /* Cylinder 4 */
  geom = dCreateCylinder(_space, FACEPLATE_R*0.8, FACEPLATE_COMPOUND_Y*0.9);
  dGeomSetBody(geom, body);
  dQuaternion q;
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      0,
      FACEPLATE_Z/2.0 - FACEPLATE_R);

  /* Cylinder 5 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_COMPOUND_Y*.9);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      (FACEPLATE_X/2.0) - FACEPLATE_R,
      0,
      FACEPLATE_Z/2.0 - FACEPLATE_R);

  /* Cylinder 6 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_COMPOUND_Y*.8);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      0,
      -FACEPLATE_Z/2.0 + FACEPLATE_R);

  /* Cylinder 7 */
  geom = dCreateCylinder(_space, FACEPLATE_R*.8, FACEPLATE_COMPOUND_Y*.8);
  dGeomSetBody(geom, body);
  dQFromAxisAndAngle(q, 1, 0, 0, DEG2RAD(90));
  dGeomSetOffsetQuaternion(geom, q);
  dGeomSetOffsetPosition(geom,
      +(FACEPLATE_X/2.0) - FACEPLATE_R,
      0,
      -FACEPLATE_Z/2.0 + FACEPLATE_R);

  dReal quat[4];
  quat[0] = rot.get_r();
  quat[1] = rot.get_i();
  quat[2] = rot.get_j();
  quat[3] = rot.get_k();
  dBodySetQuaternion(body, quat);
  
  _odeBodies[_numBodies] = body;
  _numBodies++;
  return body;
}

void MobotModel::build_mobot(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  dBodyID faceplate, body, center, body2, faceplate2; 
  faceplate = build_faceplate1(x, y, z, LQuaternionf(1, 0, 0, 0));
  body = build_body1(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  center = build_center(
      x+CENTER_X/2.0,
      y+FACEPLATE_Y/2.0+BODY_BOX1_Y+CENTER_Y/2.0,
      z,
      LQuaternionf(1, 0, 0, 0));
  body2 = build_body2(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_Y/2.0+BODY_BOX1_Y+CENTER_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  faceplate2 = build_faceplate1(
      x,
      y+FACEPLATE_Y + BODY_Y*2.0,
      z,
      LQuaternionf(1, 0, 0, 0));

  /* Attach faceplate to body */
  dJointID joint;
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, faceplate, body);
  dJointSetHingeAnchor(joint,
      x,
      y+(FACEPLATE_Y/2.0),
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[0] = joint;

  /* Attach center to body */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body, center);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + BODY_BOX2_Y,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[1] = joint;

  /* Attach center to body2 */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, center, body2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + CENTER_Y - CENTER_R,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[2] = joint;

  /* Attach last faceplate */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body2, faceplate2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_Y*2.0,
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[3] = joint;
}

const dReal* MobotModel::get_position(int index)
{
  return dBodyGetPosition(_odeBodies[index]);
}

/* Coordinates specify the CG of the first faceplate. Mobot chain will be built
 * laying along Y axis. */
void MobotModel::build_mobot_chain_head(dReal x, dReal y, dReal z)
{
  dBodyID faceplate, body, center, body2, faceplate2; 
  faceplate = build_faceplate1(x, y, z, LQuaternionf(1, 0, 0, 0));
  body = build_body1(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  center = build_center(
      x+CENTER_X/2.0,
      y+FACEPLATE_Y/2.0+BODY_BOX1_Y+CENTER_Y/2.0,
      z,
      LQuaternionf(1, 0, 0, 0));
  body2 = build_body2(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_Y/2.0+BODY_BOX1_Y+CENTER_Y,
      z,
      LQuaternionf(1, 0, 0, 0));

  /* Attach faceplate to body */
  dJointID joint;
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, faceplate, body);
  dJointSetHingeAnchor(joint,
      x,
      y+(FACEPLATE_Y/2.0),
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[0] = joint;

  /* Attach center to body */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body, center);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + BODY_BOX2_Y,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[1] = joint;

  /* Attach center to body2 */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, center, body2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + CENTER_Y - CENTER_R,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[2] = joint;
}

/* Coordinates specify CG of compound faceplate */
void MobotModel::build_mobot_chain_body(dReal x, dReal y, dReal z)
{
  dBodyID faceplate, body, center, body2; 
  faceplate = build_big_faceplate(x, y, z, LQuaternionf(1, 0, 0, 0));
  body = build_body1(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_BOX1_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  center = build_center(
      x+CENTER_X/2.0,
      y+FACEPLATE_COMPOUND_Y/2.0+BODY_BOX1_Y+CENTER_Y/2.0,
      z,
      LQuaternionf(1, 0, 0, 0));
  body2 = build_body2(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_COMPOUND_Y/2.0+BODY_BOX1_Y+CENTER_Y,
      z,
      LQuaternionf(1, 0, 0, 0));

  /* Attach faceplate to body */
  dJointID joint;
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, faceplate, body);
  dJointSetHingeAnchor(joint,
      x,
      y+(FACEPLATE_Y/2.0),
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[0] = joint;

  /* Attach center to body */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body, center);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + BODY_BOX2_Y,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[1] = joint;

  /* Attach center to body2 */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, center, body2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_Y/2.0 + BODY_BOX1_Y + CENTER_Y - CENTER_R,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[2] = joint;

}

/* Coordinates specify CG of compound faceplate */
void MobotModel::build_mobot_chain_tail(dReal x, dReal y, dReal z)
{
  dBodyID faceplate, body, center, body2, faceplate2; 
  faceplate = build_big_faceplate(x, y, z, LQuaternionf(1, 0, 0, 0));
  body = build_body1(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_BOX1_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  center = build_center(
      x+CENTER_X/2.0,
      y+FACEPLATE_COMPOUND_Y/2.0+BODY_BOX1_Y+CENTER_Y/2.0,
      z,
      LQuaternionf(1, 0, 0, 0));
  body2 = build_body2(
      x-((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      y+FACEPLATE_COMPOUND_Y/2.0+BODY_BOX1_Y+CENTER_Y,
      z,
      LQuaternionf(1, 0, 0, 0));
  faceplate2 = build_faceplate1(
      x,
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_Y*2.0 + FACEPLATE_Y/2.0,
      z,
      LQuaternionf(1, 0, 0, 0));

  /* Attach faceplate to body */
  dJointID joint;
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, faceplate, body);
  dJointSetHingeAnchor(joint,
      x,
      y+(FACEPLATE_COMPOUND_Y/2.0),
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[0] = joint;

  /* Attach center to body */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body, center);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_BOX1_Y + BODY_BOX2_Y,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[1] = joint;

  /* Attach center to body2 */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, center, body2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_BOX1_Y + CENTER_Y - CENTER_R,
      z);
  dJointSetHingeAxis(joint, 1, 0, 0);
  _joints[2] = joint;

  /* Attach last faceplate to body2 */
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, body2, faceplate2);
  dJointSetHingeAnchor(joint,
      x,
      y+FACEPLATE_COMPOUND_Y/2.0 + BODY_Y*2.0,
      z);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[3] = joint;
}

/* Attach a body or tail mobot to the end of the current mobot. The current
 * mobot should not be a tail or complete mobot... The mobots should also
 * already be properly aligned */
void MobotModel::attach_mobot(MobotModel* mobot)
{
  /* First get the position of the other mobot's faceplate. */
  const dReal* pos;
  pos = dBodyGetPosition(mobot->_odeBodies[0]);
  /* Build a hinge joint inside of that faceplate. Technically, it should
   * reside between the faceplate and the body, but because that location is
   * colinear with the axis of rotation, it won't matter if we put it inside
   * the faceplate. */
  dJointID joint;
  joint = dJointCreateHinge(_world, 0);
  dJointAttach(joint, _odeBodies[3], mobot->_odeBodies[0]);
  dJointSetHingeAnchor(joint, pos[0], pos[1], pos[2]);
  dJointSetHingeAxis(joint, 0, 1, 0);
  _joints[3] = joint;
}

void MobotModel::moveTo(dReal a1, dReal a2, dReal a3, dReal a4)
{
  _desiredAngles[0] = a1;
  _desiredAngles[1] = a2;
  _desiredAngles[2] = a3;
  _desiredAngles[3] = a4;
}

void MobotModel::getJointAngles(dReal *angles)
{
  double d;
  int i;
  for(i = 0; i < 4; i++) {
    if(i == 0 || i == 2) {
      d = -1;
    } else {
      d = 1;
    }
    angles[i] = d*dJointGetHingeAngle(_joints[i]);
  }
}

MobotChain::MobotChain(
    WindowFramework* window,
    PandaFramework* framework,
    dWorldID world,
    dSpaceID space,
    int num_modules)
{
  if(num_modules < 1) {
    return;
  }
  _window = window;
  _framework = framework;
  _world = world;
  _space = space;
  _numMobots = num_modules;
  if(num_modules == 1) {
    /* Just create one mobot */
    _mobots[0] = new MobotModel(_window, _framework, _world, _space);
    _mobots[0]->build_mobot(0, 0, 0, LQuaternionf(1, 0, 0, 0));
    return;
  }
  /* If more than one, build a head module first */
  _mobots[0] = new MobotModel(_window, _framework, _world, _space);
  _mobots[0]->build_mobot_chain_head(0, 0, 0);
  /* For each of the intermediate ones, build a body */
  int i;
  for(i = 1; i < _numMobots-1; i++) {
    _mobots[i] = new MobotModel(_window, _framework, _world, _space);
    _mobots[i]->build_mobot_chain_body(
        0,
        FACEPLATE_Y/2.0 + BODY_Y*2.0 + FACEPLATE_COMPOUND_Y/2.0 + 
        (i-1)*(FACEPLATE_COMPOUND_Y + BODY_Y*2.0),
        0);
    _mobots[i-1]->attach_mobot(_mobots[i]);
  }
  /* Build last tail mobot */
  _mobots[i] = new MobotModel(_window, _framework, _world, _space);
  _mobots[i]->build_mobot_chain_tail(
      0,
      FACEPLATE_Y/2.0 + BODY_Y*2.0 + FACEPLATE_COMPOUND_Y/2.0 + 
      (i-1)*(FACEPLATE_COMPOUND_Y + BODY_Y*2.0),
      0);
  _mobots[i-1]->attach_mobot(_mobots[i]);
}

MobotModel* MobotChain::mobot(int index)
{
  return _mobots[index];
}

void MobotChain::step(double time)
{
  int i;
  dReal angles[4];
  for(i = 0; i < _numMobots; i++) {
    _mobots[i]->step(time);
  }

#if 0
  // print angle data for first mobot
  printf("%lf ", globalClock->get_real_time() );
  _mobots[0]->getJointAngles(angles);
  for(i = 0; i < 4; i++) {
    printf("%lf ", angles[i]);
  }
  printf("\n");
#endif
  // Print position data of center mobot
  const dReal* pos = _mobots[_numMobots/2]->get_position(3);
  for(i = 0; i < 3; i++) {
    printf("%lf ", pos[i]);
  }
  printf("\n");
}

void MobotChain::update()
{
  int i;
  for(i = 0; i < _numMobots; i++) {
    _mobots[i]->update();
  }
}
