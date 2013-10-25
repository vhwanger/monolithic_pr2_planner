#pragma once
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;
    class ContArmState;
    class ArmModel {
        public:
            ArmModel();
            bool getArmAnglesFromObjectPose(DiscObjectState obj_pose, boost::shared_ptr<ContArmState> arm_state);
            bool getArmAnglesFromObjectPose(ContObjectState obj_pose, boost::shared_ptr<ContArmState> arm_state);
            
            SBPLArmModelPtr getRightArmModel(){ return m_r_arm; };
            SBPLArmModelPtr getLeftArmModel(){ return m_l_arm; };

        private:
            SBPLArmModelPtr m_l_arm;
            SBPLArmModelPtr m_r_arm;
    };
    typedef boost::shared_ptr<ArmModel> ArmModelPtr;
}
