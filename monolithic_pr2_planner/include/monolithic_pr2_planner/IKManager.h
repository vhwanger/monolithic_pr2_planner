#pragma once
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ObjectPose.h>

namespace monolithic_pr2_planner {
    class ContArmState;

    class IKManager {
        public:
            bool getArmAnglesFromObjectPose(ObjectPose obj_pose, ContArmState* arm_state);

        private:
            static sbpl_arm_planner::SBPLArmModel* m_arm_model_left;
            static sbpl_arm_planner::SBPLArmModel* m_arm_model_right;
    };
}
