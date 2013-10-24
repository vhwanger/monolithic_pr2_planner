#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

/*
 * The parameter catalog is used to grab all parameters used by the planner and
 * organize them into various structs. 
 *
 * In an effort to keep things compartmentalized, each module has its own struct
 * defining what parameters it needs. A lot of the modules use the same
 * parameters, so there's going to be some duplicates. If you're changing these
 * structs, don't try and merge them all together - it's easier to be very clear
 * about what parameters each module takes.
 *
 * The highest level function is the fetch() function, which just calls all
 * parameter getter functions. This catalog is built when the Environment object
 * is first created. 
 */
namespace monolithic_pr2_planner {

    struct Point3D {
        double x;
        double y;
        double z;
    };

    class StateSpaceParameters {
        // add functions to load in values
        private:
            std::map<std::string, std::pair<float, float> > bounds;
            std::map<std::string, float> resolutions;
    };

    // probably should be moved elsewhere
    typedef struct {
        float epsilon;
        float allocated_planning_time;
        std::string planning_joint;
    } PlannerSearchParams;

    typedef struct {
        std::string arm_motion_primitive_file;
        std::string base_motion_primitive_file;
    } MotionPrimitiveFiles;

    typedef struct {
        std::string l_arm_file;
        std::string r_arm_file;
        std::string robot_description_string;
        double env_resolution;
    } HardwareDescriptionFiles;

    typedef struct {
        double env_resolution;
        std::string reference_frame;
        Point3D origin; 
        Point3D max_point;
    } OccupancyGridParams;

    typedef struct {
        double obj_xyz_resolution;
        double obj_rpy_resolution;
        double arm_free_angle_resolution;
        double base_theta_resolution;
    } RobotResolutionParams;

    class ParameterCatalog {
        public:
            ParameterCatalog();
            void fetch();

            void setMotionPrimitiveFiles();
            void setHardwareDescriptionFiles(HardwareDescriptionFiles& params);
            void setOccupancyGridParams(OccupancyGridParams& params);
            void setRobotResolutionParams(RobotResolutionParams& params);

            RobotResolutionParams m_robot_resolution_params;
            HardwareDescriptionFiles m_hardware_description_files;
            MotionPrimitiveFiles m_motion_primitive_files;
            OccupancyGridParams m_occupancy_grid_params;

        private:
            ros::NodeHandle m_nodehandle;
            bool setFileNameFromParamServer(const std::string param_name, 
                                            std::string* parameter);
        
    };
}