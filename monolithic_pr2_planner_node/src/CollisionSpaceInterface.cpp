#include <monolithic_pr2_planner_node/CollisionSpaceInterface.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;

CollisionSpaceInterface::CollisionSpaceInterface(CSpaceMgrPtr cspace_mgr):m_cspace_mgr(cspace_mgr){
}

bool CollisionSpaceInterface::bindCollisionSpaceToTopic(string topic_name, tf::TransformListener& tf,
        string target_frame){
    m_collision_map_subscriber.subscribe(m_nodehandle, topic_name, 1);
    ROS_INFO("binding collision space to topic %s and transforming to %s!", topic_name.c_str(), target_frame.c_str());
    m_ref_frame = target_frame;
    m_collision_map_filter = boost::shared_ptr<CollisionMapMsgFilter>(new CollisionMapMsgFilter(
            m_collision_map_subscriber, tf, target_frame, 1));
    m_collision_map_filter->registerCallback(boost::bind(&CollisionSpaceInterface::mapCallback, this, _1));
    return true;
}

void CollisionSpaceInterface::mapCallback(const arm_navigation_msgs::CollisionMapConstPtr &map){
    ROS_INFO("map callback!");
    if(map->header.frame_id.compare(m_ref_frame) != 0)
    {
        ROS_WARN("collision_map_occ is in %s not in %s", map->header.frame_id.c_str(), m_ref_frame.c_str());
        ROS_DEBUG("the collision map has %i cubic obstacles", int(map->boxes.size()));
    }
    m_cspace_mgr->updateMap(*map);
    //setArmToMapTransform(map_frame_);
    return;
}
