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
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.
EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env) : 
    m_env(env), m_collision_space_interface(env->getCollisionSpace()){
        getParams();
    bool forward_search = true;
    m_planner.reset(new ARAPlanner(m_env.get(), forward_search));
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<std::string>("reference_frame", m_params.ref_frame, 
                                    std::string("map"));
}

void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res){
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->obj_goal= req.goal;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    bool retVal = m_env->configureRequest(search_request, start_id, goal_id);

    m_planner->set_initialsolution_eps(search_request->initial_epsilon);
    bool return_first_soln = true;
    m_planner->set_search_mode(return_first_soln);
    m_planner->set_start(start_id);
    ROS_INFO("setting goal id to %d", goal_id);
    m_planner->set_goal(goal_id);
    m_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    m_planner->replan(60.0, &soln, &soln_cost);

    return retVal;

}

bool EnvInterfaces::bindCollisionSpaceToTopic(std::string topic_name){
    m_collision_space_interface.bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

