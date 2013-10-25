#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace monolithic_pr2_planner;


ParameterCatalog::ParameterCatalog() : m_nodehandle("~") {
}

void ParameterCatalog::fetch(){
    ROS_INFO("fetching parameters");
    setMotionPrimitiveFiles();
    setOccupancyGridParams(m_occupancy_grid_params);
    setLeftArmParams(m_left_arm_params);
    setRightArmParams(m_right_arm_params);
    
}

void ParameterCatalog::setMotionPrimitiveFiles(){
    setFileNameFromParamServer("planner/motion_primitive_file", 
            &m_motion_primitive_files.arm_motion_primitive_file);
    setFileNameFromParamServer("planner/base_motion_primitive_file", 
            &m_motion_primitive_files.base_motion_primitive_file);
}

void ParameterCatalog::setLeftArmParams(ArmDescriptionParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    setFileNameFromParamServer("planner/left_arm_description_file", 
            &params.arm_file);
    std::string robot_urdf_param;
    if(!m_nodehandle.searchParam("robot_description",robot_urdf_param)){
        ROS_ERROR("Can't find description on param server (/robot_description not set). Exiting");
    } else {
        m_nodehandle.param<std::string>(robot_urdf_param, params.robot_description_string, 
                                        "robot_description");
    }
}

void ParameterCatalog::setRightArmParams(ArmDescriptionParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    setFileNameFromParamServer("planner/right_arm_description_file", 
            &params.arm_file);
    std::string robot_urdf_param;
    if(!m_nodehandle.searchParam("robot_description",robot_urdf_param)){
        ROS_ERROR("Can't find description on param server (/robot_description not set). Exiting");
    } else {
        m_nodehandle.param<std::string>(robot_urdf_param, params.robot_description_string, 
                                        "robot_description");
    }
}

void ParameterCatalog::setOccupancyGridParams(OccupancyGridParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    m_nodehandle.param("collision_space/reference_frame", params.reference_frame, 
                            std::string("base_link"));
    m_nodehandle.param("collision_space/occupancy_grid/origin_x", params.origin.x,-0.6);
    m_nodehandle.param("collision_space/occupancy_grid/origin_y", params.origin.y,-1.15);
    m_nodehandle.param("collision_space/occupancy_grid/origin_z", params.origin.z,-0.05);
    m_nodehandle.param("collision_space/occupancy_grid/size_x", params.origin.x,1.6);
    m_nodehandle.param("collision_space/occupancy_grid/size_y", params.origin.y,1.8);
    m_nodehandle.param("collision_space/occupancy_grid/size_z", params.origin.z,1.4);
}

// currently just hard code these...maybe someone will want to retrieve them
// from param server at some point.
void ParameterCatalog::setRobotResolutionParams(RobotResolutionParams& params){
    params.obj_xyz_resolution = 0.02;
    params.obj_rpy_resolution = 2*M_PI/180;
    params.arm_free_angle_resolution = 3*M_PI/180;
    params.base_theta_resolution = 22.5*M_PI/180;
}

bool ParameterCatalog::setFileNameFromParamServer(const std::string param_name, 
                                                  std::string* parameter){
    std::string filename;
    m_nodehandle.param<std::string>(param_name, filename, "");
    path input_path(filename.c_str());
    if (exists(input_path)){
       *parameter = filename;
        ROS_INFO("Pulling in data from %s", filename.c_str());
    } else {
       *parameter = filename;
        ROS_ERROR("Failed to find file '%s' to load in parameters for %s", filename.c_str(),
                                                                           param_name.c_str());
        return false;
    }
    return true;
}
