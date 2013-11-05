#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "testPlanningRequest");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path");
    monolithic_pr2_planner_node::GetMobileArmPlan srv;

    std::vector<double> right_arm_start(7,0), left_arm_start(7,0), body_start(4,0);

    right_arm_start[0] = 0.052858395281043;
    right_arm_start[1] = 0.075369128335531;
    right_arm_start[2] = 0.569623788333581;
    right_arm_start[3] = -0.54373199879478;
    right_arm_start[4] = -22.4372417947492;
    right_arm_start[5] = -1.86517790099345;
    right_arm_start[6] = 8.571527760711906;



    body_start[0] = 4;
    body_start[1] = 3;
    body_start[2] = .1;
    body_start[3] = 0;

    srv.request.rarm_start = right_arm_start;
    srv.request.larm_start = left_arm_start;
    srv.request.body_start = body_start;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 1;
    pose.pose.position.z = .5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped rarm_offset;
    rarm_offset.pose.position.x = 0;
    rarm_offset.pose.position.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.x = 0;
    rarm_offset.pose.orientation.y = 0;
    rarm_offset.pose.orientation.z = 0;
    rarm_offset.pose.orientation.w = 1;
    srv.request.rarm_object = rarm_offset;

    srv.request.goal = pose;
    srv.request.initial_eps = 10;
    srv.request.final_eps = 9;
    srv.request.dec_eps = .1;
    srv.request.xyz_tolerance = .1;
    srv.request.roll_tolerance = .1;
    srv.request.pitch_tolerance = .1;
    srv.request.yaw_tolerance = .1;
    if (client.call(srv))
    {
        ROS_INFO("called service");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}
