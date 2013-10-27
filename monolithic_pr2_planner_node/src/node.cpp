#include <monolithic_pr2_planner_node/node.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <log4cxx/logger.h>
#include <ros/console.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;

Node::Node() : m_env_interface(m_env){
    m_env_interface.bindPlanPathToEnv("/sbpl_planning/plan_path");
    m_env_interface.bindCollisionSpaceToTopic("collision_map_out");
}


void changeLoggerLevel(std::string name, std::string level)
{
    std::string logger_name = name;
    log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);
    // Set the logger for this package to output all statements
    if (level.compare("debug") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    else if (level.compare("warn") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Warn]);
    else if (level.compare("info") == 0)
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
    else if (level.compare("fatal") == 0){
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Fatal]);
    }
}


void setLoggersFromParamServer(ros::NodeHandle nh){
    std::string level;
    nh.param<std::string>("/monolithic_pr2_planner_node/debug/logging/configuration", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(CONFIG), level);

    nh.param<std::string>("/monolithic_pr2_planner_node/debug/logging/initialization", 
                          level, "info");
    changeLoggerLevel(std::string("ros.monolithic_pr2_planner") + 
                                  std::string(".") + 
                                  std::string(INIT), level);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "monolithic_pr2_planner_node");
    ros::NodeHandle nh("~");
    setLoggersFromParamServer(nh);
    Node node;

    ros::spin();
}
