#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

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
        std::string left_arm_description_file;
        std::string right_arm_description_file;
    } RobotHardwareDescriptionFiles;

    typedef struct {
        double env_resolution;
        std::string reference_frame;
        Point3D origin; 
        Point3D max_point;
    } CollisionSpaceParams;

    class ParameterCatalog {
        public:
            ParameterCatalog();
            void fetch();
            void setMotionPrimitiveFiles();
            void setRobotHardwareDescriptionFiles();
            void setOccupancyGridParams();

            RobotHardwareDescriptionFiles m_arm_description_files;
            MotionPrimitiveFiles m_motion_primitive_files;
            CollisionSpaceParams m_collision_space_params;

        private:
            ros::NodeHandle m_nodehandle;
            bool setFileNameFromParamServer(const std::string param_name, 
                                            std::string* parameter);
        
    };
}
