#pragma once
namespace monolithic_pr2_planner {
    const int GRAPH_STATE_SIZE = 12;
    class GraphStateElement {
        public:
            enum {OBJ_X, 
                  OBJ_Y, 
                  OBJ_Z, 
                  OBJ_ROLL, 
                  OBJ_PITCH, 
                  OBJ_YAW,
                  R_FA, 
                  L_FA, 
                  BASE_X, 
                  BASE_Y, 
                  BASE_Z, 
                  BASE_THETA};
    };
    class BodyDOF {
        public:
            enum {X, Y, Z, THETA};
    };

    class Joints { 
        public:
            enum {
                SHOULDER_PAN,
                SHOULDER_LIFT,
                UPPER_ARM_ROLL,
                ELBOW_FLEX,
                FOREARM_ROLL,
                WRIST_FLEX,
                WRIST_ROLL
            };
    };

    class ArmSide {
        public:
            enum {
                LEFT,
                RIGHT
            };
    };

    class ObjectPose {
        public:
            enum { X, Y, Z, ROLL, PITCH, YAW };
    };

    class Tolerances {
        public:
            enum { XYZ, ROLL, PITCH, YAW };
    };

    class MPrim_Types {
        public:
            enum { BASE, ARM, ARM_ADAPTIVE, BASE_ADAPTIVE, TORSO };
    };

    class PlanningModes {
        public:
            enum { BASE_ONLY, 
                   RIGHT_ARM, 
                   LEFT_ARM, 
                   DUAL_ARM, 
                   RIGHT_ARM_MOBILE, 
                   LEFT_ARM_MOBILE, 
                   DUAL_ARM_MOBILE };
    };
}

