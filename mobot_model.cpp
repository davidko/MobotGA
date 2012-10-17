#include "mobot_model.h"

OdeBody* build_faceplate1(OdeWorld* world, OdeSpace* space, dReal x, dReal y, dReal z, LQuaternionf rot)
{
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
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*1.01);
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
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*1.01);
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
  geom = new OdeCylinderGeom(*space, FACEPLATE_R*1.01, FACEPLATE_Y*1.01);
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

  return body;
}
