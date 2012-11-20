#ifndef _MOBOT_SPECS_H_
#define _MOBOT_SPECS_H_

/* Notes */
/* 
 * Motor 2 angle should be negated 
 * */

#define FACEPLATE_Y .00845 
#define FACEPLATE_Z .0763
#define FACEPLATE_X FACEPLATE_Z
#define FACEPLATE_COMPOUND_Y (FACEPLATE_Y*3.0)
// Corner radius
#define FACEPLATE_R .0254
// Collision Category
#define FACEPLATE1_CAT 0x01
#define FACEPLATE2_CAT 0x02

#define CENTER_Y .10255
#define CENTER_Z .0509
#define CENTER_X .0319
// Corner Radius
#define CENTER_R (CENTER_Z/2.0)
// Collision Category
#define CENTER_CAT 0x04

#define BODY_Z      .0510
#define BODY_Y      .08565
#define BODY_X      .0768
#define BODY_CG_OFFSET 0.0122

#define BODY_BOX1_X BODY_X
#define BODY_BOX1_Y .0339
#define BODY_BOX1_Z BODY_Z

#define BODY_BOX2_X .0366
#define BODY_BOX2_Y (BODY_Y - BODY_BOX1_Y - BODY_CYL2_Y)
#define BODY_BOX2_Z BODY_Z

#define BODY_BOX3_X BODY_CYL2_X
#define BODY_BOX3_Y BODY_BOX2_Y
#define BODY_BOX3_Z BODY_Z

#define BODY_CYL1_X BODY_BOX2_X
#define BODY_CYL1_Y (BODY_Z/2.0)
#define BODY_CYL1_Z BODY_Z
#define BODY_CYL1_R (BODY_Z/2.0)

#define BODY_CYL2_X .0068
#define BODY_CYL2_Y BODY_CYL1_Y
#define BODY_CYL2_Z BODY_Z
#define BODY_CYL2_R (BODY_Z/2.0)

#define BODY1_CAT 0x08
#define BODY2_CAT 0x10

/* Masses, in kg */
#define MOBOT_M 0.510291
#define FACEPLATE_M (MOBOT_M*0.05)
#define BODY_M (MOBOT_M*0.3)
#define CENTER_M (MOBOT_M*0.3)

#define GROUND_CAT 0x20

#define DELTA 0.001

#endif
