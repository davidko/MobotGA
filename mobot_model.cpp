#include "mobot_model.h"

MobotModel::MobotModel(WindowFramework* window, PandaFramework* framework, dWorldID world, dSpaceID space)
{
  _window = window;
  _framework = framework;
  _numBodies = 0;
  _world = world;
  _space = space;
  _numBodies = 0;
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

void MobotModel::build_faceplate1(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  /* Create the nodepath */
  NodePath node = _window->load_model(_framework->get_models(), "models/box");
  node.reparent_to(_window->get_render());
  node.set_pos(-0.5, -0.5, -0.5);
  node.flatten_light();
  //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
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
  _nodePaths[_numBodies] = node;
  _numBodies++;
}

void MobotModel::build_body1(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  /* Create the nodepath */
  NodePath node = _window->load_model(_framework->get_models(), "models/box");
  node.reparent_to(_window->get_render());
  //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  node.set_pos(-0.5, -0.5, -0.5);
  node.flatten_light();
  node.set_scale(BODY_X, BODY_Y, BODY_Z);

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

#if 0
  /* Box 2 */
  geom = new OdeBoxGeom(*_space, 
      BODY_BOX2_X,
      BODY_BOX2_Y,
      BODY_BOX2_Z);
  geom->set_collide_bits(0xFF & (~BODY1_CAT));
  geom->set_category_bits(BODY1_CAT);
  geom->set_body(*body);
  geom->set_offset_position(
      BODY_BOX2_X-BODY_CG_OFFSET,
      BODY_BOX2_Y/2.0,
      0);

  /* Box 3 */
  geom = new OdeBoxGeom(*_space, 
      BODY_BOX3_X,
      BODY_BOX3_Y,
      BODY_BOX3_Z);
  geom->set_collide_bits(0xFF & (~BODY1_CAT));
  geom->set_category_bits(BODY1_CAT);
  geom->set_body(*body);
  geom->set_offset_position(
      (BODY_X - BODY_BOX3_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET),
      BODY_BOX3_Y/2.0,
      0);

  /* Cylinder 1 */
  geom = new OdeCylinderGeom(*_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  LQuaternionf q;
  q.set_from_axis_angle(90, LVector3f(0, 1, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      -(BODY_CYL1_X/2.0 - BODY_CG_OFFSET),
      BODY_BOX2_Y,
      0);

  /* Cylinder 2 */
  geom = new OdeCylinderGeom(*_space, BODY_CYL1_R, BODY_CYL1_X*.99);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  q.set_from_axis_angle(90, LVector3f(0, 1, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      BODY_X - BODY_CYL2_X/2.0 - (BODY_BOX2_X - BODY_CG_OFFSET),
      BODY_BOX3_Y,
      0);

  _odeBodies[_numBodies] = body;
  _nodePaths[_numBodies] = node;
  _numBodies++;
#endif
}

void MobotModel::build_center(dReal x, dReal y, dReal z, LQuaternionf rot)
{
#if 0
  /* Create the nodepath */
  NodePath node = _window->load_model(_framework->get_models(), "models/box");
  node.reparent_to(_window->get_render());
  //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  node.set_pos(-0.5, -0.5, -0.5);
  node.flatten_light();
  node.set_scale(CENTER_X, CENTER_Y, CENTER_Z);
  OdeBody *body = new OdeBody(*_world);
  OdeMass M = OdeMass();
  M.set_box(CENTER_M, CENTER_X, CENTER_Y, CENTER_Z);
  body->set_mass(M);
  body->set_position(x, y, z);
  body->set_quaternion(rot);

  /* Set up collision geometry */
  OdeGeom* geom;

  /* Box 1 */
  geom = new OdeBoxGeom(*_space, 
      CENTER_X,
      CENTER_Y - (CENTER_R)*2.0,
      CENTER_Z);
  geom->set_collide_bits(0xFF & (~CENTER_CAT));
  geom->set_category_bits(CENTER_CAT);
  geom->set_body(*body);
  geom->set_offset_position(
      0,
      0,
      0);

  /* Cyl 1 */
  geom = new OdeCylinderGeom(*_space,
      CENTER_R,
      CENTER_X);
  geom->set_collide_bits(0xFF & (~CENTER_CAT));
  geom->set_category_bits(CENTER_CAT);
  geom->set_body(*body);
  LQuaternionf q;
  q.set_from_axis_angle(90, LVector3f(0, 1, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      0,
      -(CENTER_Y/2.0),
      0);

  /* Cyl 1 */
  geom = new OdeCylinderGeom(*_space,
      CENTER_R,
      CENTER_X);
  geom->set_collide_bits(0xFF & (~CENTER_CAT));
  geom->set_category_bits(CENTER_CAT);
  geom->set_body(*body);
  q.set_from_axis_angle(90, LVector3f(0, 1, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      0,
      (CENTER_Y/2.0),
      0);

  _odeBodies[_numBodies] = body;
  _nodePaths[_numBodies] = node;
  _numBodies++;
#endif
}

void MobotModel::build_mobot(dReal x, dReal y, dReal z, LQuaternionf rot)
{
  build_faceplate1(0, 0, 0, LQuaternionf(1, 0, 0, 0));
  build_body1(
      -((BODY_X/2.0) - (BODY_BOX2_X-BODY_CG_OFFSET)),
      FACEPLATE_Y/2.0 + BODY_BOX1_Y + DELTA,
      0,
      LQuaternionf(1, 0, 0, 0));
  OdeHingeJoint* joint = new OdeHingeJoint(*_world);
  joint->attach_bodies(*_odeBodies[0], *_odeBodies[1]);
  joint->set_anchor(
      0,
      (FACEPLATE_Y/2.0) + DELTA/2.0,
      0);
  joint->set_axis(0, 1, 0);
}

const dReal* MobotModel::get_position(int index)
{
  return dBodyGetPosition(_odeBodies[index]);
}
