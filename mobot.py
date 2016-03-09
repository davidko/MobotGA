#!/usr/bin/env python

class SimObject():
    def __init__(self):
        pass

class MobotModel(SimObject):
    FACEPLATE_Y = .00845 
    FACEPLATE_Z = .0763
    FACEPLATE_X = FACEPLATE_Z
    FACEPLATE_COMPOUND_Y = (FACEPLATE_Y*3.0)
    # Corner radius
    FACEPLATE_R = .0254
    # Collision Category
    FACEPLATE1_CAT = 0x01
    FACEPLATE2_CAT = 0x02

    CENTER_Y = .10255
    CENTER_Z = .0509
    CENTER_X = .0319
    # Corner Radius
    CENTER_R = (CENTER_Z/2.0)
    # Collision Category
    CENTER_CAT = 0x04

    BODY_Z     = .0510
    BODY_Y     = .08565
    BODY_X     = .0768
    BODY_CG_OFFSET = 0.0122

    BODY_BOX1_X = BODY_X
    BODY_BOX1_Y = .0339
    BODY_BOX1_Z = BODY_Z

    BODY_BOX2_X = .0366
    BODY_BOX2_Y = (BODY_Y - BODY_BOX1_Y - BODY_CYL2_Y)
    BODY_BOX2_Z = BODY_Z

    BODY_BOX3_X = BODY_CYL2_X
    BODY_BOX3_Y = BODY_BOX2_Y
    BODY_BOX3_Z = BODY_Z

    BODY_CYL1_X = BODY_BOX2_X
    BODY_CYL1_Y = (BODY_Z/2.0)
    BODY_CYL1_Z = BODY_Z
    BODY_CYL1_R = (BODY_Z/2.0)

    BODY_CYL2_X = .0068
    BODY_CYL2_Y = BODY_CYL1_Y
    BODY_CYL2_Z = BODY_Z
    BODY_CYL2_R = (BODY_Z/2.0)

    BODY1_CAT = 0x08
    BODY2_CAT = 0x10

    #/* Masses, in kg */
    MOBOT_M = 0.510291
    FACEPLATE_M = (MOBOT_M*0.05)
    BODY_M = (MOBOT_M*0.3)
    CENTER_M = (MOBOT_M*0.3)

    GROUND_CAT = 0x20

    DELTA = 0.001

    def __init__(self, x, y, z, rot):
        """
        x, y, and z are type (double), coordinates of the Mobot. rot is a
        Quaternion representing the rotation of the Mobot.
        """

class Faceplate(SimObject):
    def __init__(self, x, y, z, rot):
