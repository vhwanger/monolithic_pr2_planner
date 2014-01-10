#include <monolithic_pr2_planner_node/CollisionSpaceInterface.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost::filesystem;
using namespace boost;

CollisionSpaceInterface::CollisionSpaceInterface(CSpaceMgrPtr cspace_mgr, HeuristicMgrPtr heur_mgr):
    m_cspace_mgr(cspace_mgr),
    m_heur_mgr(heur_mgr) {
    m_cmap_pub = m_nodehandle.advertise<arm_navigation_msgs::CollisionMap>("environment", 1);
    m_pcl_pub = m_nodehandle.advertise<sensor_msgs::PointCloud2>("pcl_environment", 1);
}

bool CollisionSpaceInterface::getVoxelsFromMesh(std::string resource, 
        geometry_msgs::Pose pose, 
        std::vector<std::vector<double> > &voxels){
    std::vector<int32_t> triangles;
    std::vector<geometry_msgs::Point> vertices, scaled_vertices;

    // get STL version of file
    std::string mesh_type = leatherman::getExtension(resource);
    std::string stl_resource = resource;
    if(!mesh_type.empty()){
        if(mesh_type.compare("dae") == 0)
            stl_resource = leatherman::replaceExtension(resource, "stl");
        ROS_WARN("Collada file found. Will try to use the STL version instead. (%s)", 
                 resource.c_str());
        ROS_WARN("STL filename: %s", stl_resource.c_str());
    }

    // get triangles
    geometry_msgs::Vector3 temp_scale;
    temp_scale.x = 1.0;
    temp_scale.y = 1.0;
    temp_scale.z = 1.0;
    if(!leatherman::getMeshComponentsFromResource(stl_resource, temp_scale, 
                                                  triangles, vertices))
        return false;

    // scale the vertices
    double scale = leatherman::getColladaFileScale(resource);
    leatherman::scaleVertices(vertices, scale, scale, scale, scaled_vertices);
    ROS_INFO("Scaled the vertices uniformly by a factor of %0.3f.", scale);

    // voxelize!
    sbpl::Voxelizer::voxelizeMesh(scaled_vertices, triangles, pose, 0.02, voxels, false);

    return true;
}


bool CollisionSpaceInterface::bindCollisionSpaceToTopic(string topic_name, 
                                                        tf::TransformListener& tf,
                                                        string target_frame){
    m_collision_map_subscriber.subscribe(m_nodehandle, topic_name, 1);
    ROS_DEBUG_NAMED(INIT_LOG, "binding collision space to topic %s "
                              "and transforming to %s!", 
                              topic_name.c_str(), target_frame.c_str());
    m_ref_frame = target_frame;
    m_collision_map_filter = make_shared<CollisionMapMsgFilter>(m_collision_map_subscriber, 
                                                                tf, target_frame, 1);
    m_collision_map_filter->registerCallback(boost::bind(
            &CollisionSpaceInterface::mapCallback, this, _1));
    return true;
}

void CollisionSpaceInterface::mapCallback(
        const arm_navigation_msgs::CollisionMapConstPtr &map){
    ROS_DEBUG_NAMED(INIT_LOG, "map callback!");
    if(map->header.frame_id.compare(m_ref_frame) != 0)
    {
        // TODO: fix this warning
        //ROS_WARN_NAMED(INIT_LOG, "collision_map_occ is in %s not in %s", 
        //               map->header.frame_id.c_str(), m_ref_frame.c_str());
        ROS_DEBUG_NAMED(INIT_LOG,"the collision map has %i cubic obstacles", 
                        int(map->boxes.size()));
    }
    m_cspace_mgr->updateMap(*map);
    ROS_DEBUG_NAMED(INIT_LOG, "publishing map");
    m_cmap_pub.publish(*map);
    //setArmToMapTransform(map_frame_);
    return;
}

vector<Eigen::Vector3d> CollisionSpaceInterface::getVoxelsFromFile(std::string filename,
                                                                   int& mapsize_x, 
                                                                   int& mapsize_y){
    path p(filename);
    if (!exists(p)){
        ROS_ERROR("Couldn't find %s to load in the collision map!", 
                  filename.c_str());
    }

    ROS_DEBUG_NAMED(CONFIG_LOG, "Reading stl file %s",filename.c_str());

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    std::vector<std::vector<double> > voxels;
    getVoxelsFromMesh(filename, pose, voxels);

    std::vector<Eigen::Vector3d> points;
    double maxx = -1;
    double maxy = -1;
    double maxz = -1;
    double minx = 100000;
    double miny = 100000;
    double minz = 100000;
    for(unsigned int i=0; i<voxels.size(); i++){
        if(voxels[i][0]>maxx)
            maxx = voxels[i][0];
        if(voxels[i][1]>maxy)
            maxy = voxels[i][1];
        if(voxels[i][2]>maxz)
            maxz = voxels[i][2];
        if(voxels[i][0]<minx)
            minx = voxels[i][0];
        if(voxels[i][1]<miny)
            miny = voxels[i][1];
        if(voxels[i][2]<minz)
            minz = voxels[i][2];
    }
    for(unsigned int i=0; i<voxels.size(); i++){
        //TODO: this offset should be read in from somewhere!
        Eigen::Vector3d p(voxels[i][0]-minx,
                voxels[i][1]-miny,
                voxels[i][2]-minz-0.32);
        points.push_back(p);
    }

    mapsize_x = maxx-minx;
    mapsize_y = maxy-miny;

    ROS_INFO_NAMED(CONFIG_LOG, "bounds (%f, %f, %f) to (%f, %f, %f)",
            minx,miny,minz,maxx,maxy,maxz);

    //make a point cloud for visualization
    ROS_DEBUG_NAMED(CSPACE_LOG, "visualizing point cloud of size %lu", points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclCloud->points.resize(points.size());
    for(unsigned int i=0; i<points.size(); i++){
        pclCloud->points[i].x = points[i][0];
        pclCloud->points[i].y = points[i][1];
        pclCloud->points[i].z = points[i][2];
    }
    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg (*pclCloud, pc);
    pc.header.frame_id = "/map";
    pc.header.stamp = ros::Time::now();
    m_pcl_pub.publish(pc);
    sleep(1);

    return points;
}

void CollisionSpaceInterface::loadMap(std::string filename){
    int mapsize_x, mapsize_y;
    VoxelList voxels = getVoxelsFromFile(filename, mapsize_x, mapsize_y);
    m_cspace_mgr->loadMap(voxels);

}

void CollisionSpaceInterface::update3DHeuristicMaps(){
    m_heur_mgr->update3DHeuristicMaps();
}

void CollisionSpaceInterface::update2DHeuristicMaps(std::vector<signed char>& data){
    m_heur_mgr->update2DHeuristicMaps(data);
}