#pragma once
#include <arm_navigation_msgs/CollisionMap.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace monolithic_pr2_planner_node {
    typedef tf::MessageFilter<arm_navigation_msgs::CollisionMap> CollisionMapMsgFilter;
    class CollisionSpaceInterface {
        public:
            CollisionSpaceInterface(monolithic_pr2_planner::CSpaceMgrPtr);
            bool bindCollisionSpaceToTopic(std::string topic_name, tf::TransformListener& tf, std::string ref_frame);
            bool initCollisionMapFromFile(std::string filename);
        private:
            std::string m_ref_frame;
            void mapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);
            message_filters::Subscriber<arm_navigation_msgs::CollisionMap> m_collision_map_subscriber;
            boost::shared_ptr<CollisionMapMsgFilter> m_collision_map_filter;
            monolithic_pr2_planner::CSpaceMgrPtr m_cspace_mgr;
            ros::NodeHandle m_nodehandle;
            ros::Publisher m_cmap_pub;
    };
}
