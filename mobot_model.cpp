#include "mobot_model.h"

MobotModel::MobotModel(WindowFramework* window, PandaFramework* framework)
{
  _window = window;
  _framework = framework;
  _numBodies = 0;
}

MobotModel::~MobotModel()
{
}

void MobotModel::update()
{
  int i;
  for(i = 0; i < _numBodies; i++) {
    _nodePaths[i].set_pos_quat(
        _window->get_render(),
        _odeBodies[i]->get_position(),
        _odeBodies[i]->get_quaternion()
        );
  }
}

void MobotModel::build_faceplate1(OdeWorld* world, OdeSpace* space, dReal x, dReal y, dReal z, LQuaternionf rot)
{
  /* Create the nodepath */
  NodePath node = _window->load_model(_framework->get_models(), "models/box");
  node.reparent_to(_window->get_render());
  //node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  node.set_scale(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  OdeBody *body = new OdeBody(*world);
  OdeMass M = OdeMass();
  M.set_box(FACEPLATE_M, FACEPLATE_X, FACEPLATE_Y, FACEPLATE_Z);
  body->set_mass(M);
  body->set_position(x, y, z);
  body->set_quaternion(rot);
  
  /* Set up the collision geometry */
  OdeGeom* geom;
  /* Box 1 */
  geom = new OdeBoxGeom(*space, 
      FACEPLATE_X - (2.0*FACEPLATE_R),
      FACEPLATE_Y,
      FACEPLATE_Z);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);

  /* Box 2 */
  geom = new OdeBoxGeom(*space, 
      FACEPLATE_R,
      FACEPLATE_Y,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  geom->set_offset_position(
      -(FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 - FACEPLATE_R/2.0,
      0,
      0);
      
  /* Box 3 */
  geom = new OdeBoxGeom(*space, 
      FACEPLATE_R,
      FACEPLATE_Y,
      FACEPLATE_Z - (2.0*FACEPLATE_R));
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  geom->set_offset_position(
      (FACEPLATE_X - (2.0*FACEPLATE_R))/2.0 + FACEPLATE_R/2.0,
      0,
      0);

  /* Cylinder 4 */
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*.99);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  LQuaternionf q;
  q.set_from_axis_angle(90, LVector3f(1, 0, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      FACEPLATE_Z/2.0 - FACEPLATE_R,
      0);

  /* Cylinder 5 */
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*.99);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  q.set_from_axis_angle(90, LVector3f(1, 0, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      (FACEPLATE_X/2.0) - FACEPLATE_R,
      FACEPLATE_Z/2.0 - FACEPLATE_R,
      0);

  /* Cylinder 6 */
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*.99);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  q.set_from_axis_angle(90, LVector3f(1, 0, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      -(FACEPLATE_X/2.0) + FACEPLATE_R,
      -FACEPLATE_Z/2.0 + FACEPLATE_R,
      0);

  /* Cylinder 7 */
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*1.01);
  geom->set_collide_bits(0xFF & (~FACEPLATE1_CAT));
  geom->set_category_bits(FACEPLATE1_CAT);
  geom->set_body(*body);
  q.set_from_axis_angle(90, LVector3f(1, 0, 0));
  geom->set_offset_quaternion(q);
  geom->set_offset_position(
      +(FACEPLATE_X/2.0) - FACEPLATE_R,
      -FACEPLATE_Z/2.0 + FACEPLATE_R,
      0);

  _odeBodies[_numBodies] = body;
  _nodePaths[_numBodies] = node;
  _numBodies++;
}

LVector3f MobotModel::get_position(int index)
{
  return _odeBodies[index]->get_position();
}
