#pragma once
namespace monolithic_pr2_planner {
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
            enum { BASE, ARM, ADAPTIVE };
    };
}

