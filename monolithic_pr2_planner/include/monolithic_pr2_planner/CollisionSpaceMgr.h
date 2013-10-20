#pragma once
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <Eigen/Core>

namespace monolithic_pr2_planner {
    class CollisionSpaceMgr {
        public:
            CollisionSpaceMgr(const CollisionSpaceParams& params, ArmModelPtr arm_model);
            bool isBodyWorldColliding(RobotPose& robot_pose);
            bool isArmsWorldColliding(RobotPose& robot_pose);
            bool isBodyArmsColliding(RobotPose& robot_pose);
            bool isArmArmColliding(RobotPose& robot_pose);
            bool isAnythingColliding(RobotPose& robot_pose);

            bool isValid(DiscObjectState& obj_state);
            bool isValid(RobotPose& robot_pose);

            void updateMap(const arm_navigation_msgs::CollisionMap& map);
            //bool readMapFromEigen(Eigen::Vector3d points);

        private:
            boost::shared_ptr<pr2_collision_checker::PR2CollisionSpace> m_cspace;
            boost::shared_ptr<sbpl_arm_planner::OccupancyGrid> m_grid;
            boost::shared_ptr<ArmModel> m_arm_model;
    };
    typedef boost::shared_ptr<CollisionSpaceMgr> CSpaceMgrPtr;
}
