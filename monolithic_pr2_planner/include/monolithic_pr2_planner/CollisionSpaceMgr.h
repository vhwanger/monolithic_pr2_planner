#pragma once
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <Eigen/Core>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;
    class CollisionSpaceMgr : OccupancyGridUser {
        public:
            CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                              SBPLArmModelPtr left_arm);
            bool isValid(DiscObjectState& obj_state);
            bool isValid(RobotPose& robot_pose);
            bool isValidMotion(GraphStatePtr source_state, GraphStatePtr successor);

            void updateMap(const arm_navigation_msgs::CollisionMap& map);
            //bool readMapFromEigen(Eigen::Vector3d points);

        private:
            boost::shared_ptr<pr2_collision_checker::PR2CollisionSpace> m_cspace;
    };
    typedef boost::shared_ptr<CollisionSpaceMgr> CSpaceMgrPtr;
}
