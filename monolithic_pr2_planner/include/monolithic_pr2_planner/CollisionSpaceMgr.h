#pragma once
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <Eigen/Core>
#include <memory>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;
    class CollisionSpaceMgr : OccupancyGridUser {
        public:
            CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                              SBPLArmModelPtr left_arm);
            bool isValid(DiscObjectState& obj_state);
            bool isValid(RobotPose& robot_pose);
            bool isValidMotion(const GraphState& source_state, 
                               const MotionPrimitivePtr& mprim,
                               std::unique_ptr<GraphState>& successor);

            void updateMap(const arm_navigation_msgs::CollisionMap& map);
            //bool readMapFromEigen(Eigen::Vector3d points);

        private:
            // only need to check arms-arms, arms-world, arms-body
            bool isValidAfterArmMotion(std::unique_ptr<GraphState>& successor,
                                        const MotionPrimitivePtr& mprim) const;

            // only need to check base, both arms against world (not against each other),
            // torso, head
            bool isValidAfterBaseMotion(std::unique_ptr<GraphState>& successor,
                                        const MotionPrimitivePtr& mprim) const;
            bool isValidAfterAnyMotion(std::unique_ptr<GraphState>& successor,
                                        const MotionPrimitivePtr& mprim) const;
            boost::shared_ptr<pr2_collision_checker::PR2CollisionSpace> m_cspace;
    };
    typedef boost::shared_ptr<CollisionSpaceMgr> CSpaceMgrPtr;
}
