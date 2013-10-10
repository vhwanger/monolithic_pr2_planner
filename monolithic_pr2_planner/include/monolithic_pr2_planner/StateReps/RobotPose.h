#pragma once
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <geometry_msgs/Pose.h>

namespace monolithic_pr2_planner {
    class RobotPose {
        private:
            DiscBaseState base_state;
            ContArmState left_arm;
            ContArmState right_arm;
    };
}
