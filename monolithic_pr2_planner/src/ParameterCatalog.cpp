#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace monolithic_pr2_planner;


ParameterCatalog::ParameterCatalog() : m_nodehandle("~") {
}

void ParameterCatalog::fetch(){
    setMotionPrimitiveFiles();
    setRobotHardwareDescriptionFiles();
}

void ParameterCatalog::setMotionPrimitiveFiles(){
    ROS_INFO("setting motion primitive file from param server");
    setFileNameFromParamServer("planner/motion_primitive_file", 
            &m_motion_primitive_files.arm_motion_primitive_file);
    setFileNameFromParamServer("planner/base_motion_primitive_file", 
            &m_motion_primitive_files.base_motion_primitive_file);
}

void ParameterCatalog::setRobotHardwareDescriptionFiles(){
    ROS_INFO("setting robot hardware description files from param server");
    setFileNameFromParamServer("planner/left_arm_description_file", 
            &m_arm_description_files.left_arm_description_file);
    setFileNameFromParamServer("planner/right_arm_description_file", 
            &m_arm_description_files.right_arm_description_file);
}

void ParameterCatalog::setOccupancyGridParams(){
    m_nodehandle.param("collision_space/resolution",
                            m_collision_space_params.env_resolution, 0.02);
    m_nodehandle.param("collision_space/reference_frame",
                            m_collision_space_params.reference_frame, 
                            std::string("base_link"));
    m_nodehandle.param("collision_space/occupancy_grid/origin_x",
                            m_collision_space_params.origin.x,-0.6);
    m_nodehandle.param("collision_space/occupancy_grid/origin_y",
                            m_collision_space_params.origin.y,-1.15);
    m_nodehandle.param("collision_space/occupancy_grid/origin_z",
                            m_collision_space_params.origin.z,-0.05);
    m_nodehandle.param("collision_space/occupancy_grid/size_x",
                            m_collision_space_params.origin.x,1.6);
    m_nodehandle.param("collision_space/occupancy_grid/size_y",
                            m_collision_space_params.origin.y,1.8);
    m_nodehandle.param("collision_space/occupancy_grid/size_z",
                            m_collision_space_params.origin.z,1.4);
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
        ROS_ERROR("Failed to find file '%s' to load in parameters for %s", filename.c_str(),
                                                                           param_name.c_str());
        return false;
    }
    return true;
}
