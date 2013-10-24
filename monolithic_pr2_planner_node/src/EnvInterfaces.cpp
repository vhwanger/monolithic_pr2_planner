#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

EnvInterfaces::EnvInterfaces(Environment& env) : 
    m_nodehandle("~"), 
    m_env(boost::make_shared<Environment>(env)),
    m_collision_space_interface(env.getCollisionSpace()){
        getParams();
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<std::string>("reference_frame", m_params.ref_frame, std::string("map"));
}

void EnvInterfaces::bindPlanPathToEnv(){
    m_nodehandle.advertiseService("/sbpl_planning/plan_path", 
            &EnvInterfaces::planPathCallback,
            this);
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res){
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->obj_goal_pose = req.goal;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = ArmStateFactory::createArmState(req.larm_start, ArmSide::LEFT);
    search_request->right_arm_start = ArmStateFactory::createArmState(req.rarm_start, ArmSide::RIGHT);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = KDL::Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                              req.rarm_object.pose.orientation.y, 
                                              req.rarm_object.pose.orientation.z, 
                                              req.rarm_object.pose.orientation.w);
    larm_offset.M = KDL::Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                              req.larm_object.pose.orientation.y, 
                                              req.larm_object.pose.orientation.z, 
                                              req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;

    m_env->plan(search_request);
    return true;
}

bool EnvInterfaces::bindCollisionSpaceToTopic(std::string topic_name){
    m_collision_space_interface.bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}
