#pragma once
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class ContArmState;
    class ArmModel {
        public:
            ArmModel(HardwareDescriptionFiles robot_model);
            bool getArmAnglesFromObjectPose(DiscObjectState obj_pose, boost::shared_ptr<ContArmState> arm_state);
            bool getArmAnglesFromObjectPose(ContObjectState obj_pose, boost::shared_ptr<ContArmState> arm_state);
            
            boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> getRightArmModel(){ return m_r_arm; };
            boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> getLeftArmModel(){ return m_l_arm; };

        private:
            boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> m_l_arm;
            boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> m_r_arm;
    };
    typedef boost::shared_ptr<ArmModel> ArmModelPtr;
}
