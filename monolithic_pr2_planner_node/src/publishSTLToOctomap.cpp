#include <sbpl_geometry_utils/Voxelizer.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <leatherman/utils.h>
#include <leatherman/file.h>
#include <leatherman/binvox.h>
#include <ros/ros.h>

using namespace boost::filesystem;
using namespace std;

bool getVoxelsFromMesh(std::string resource, 
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
        ROS_DEBUG("Collada file found. Will try to use the STL version instead. (%s)", 
                 resource.c_str());
        ROS_DEBUG("STL filename: %s", stl_resource.c_str());
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
    ROS_DEBUG("Scaled the vertices uniformly by a factor of %0.3f.", scale);

    // voxelize!
    sbpl::Voxelizer::voxelizeMesh(scaled_vertices, triangles, pose, 0.02, voxels, false);

    return true;
}

vector<Eigen::Vector3d> getVoxelsFromFile(std::string filename){
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_environment", 1);
    path p(filename);
    if (!exists(p)){
        ROS_ERROR("Couldn't find %s to load in the collision map!", 
                  filename.c_str());
    }

    ROS_DEBUG("Reading stl file %s",filename.c_str());

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

    //mapsize_x = maxx-minx;
    //mapsize_y = maxy-miny;

    ROS_DEBUG("bounds (%f, %f, %f) to (%f, %f, %f)",
            minx,miny,minz,maxx,maxy,maxz);

    //make a point cloud for visualization
    ROS_INFO("visualizing point cloud of size %lu", points.size());
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
    sleep(1);
    pcl_pub.publish(pc);

    return points;
}

bool isValidPath(std::string filename){
    path input_path(filename.c_str());
    if (exists(input_path)){
        ROS_DEBUG("Pulling in data from %s", filename.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to find file '%s'", filename.c_str());
        return false;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stl_to_octomap");
    if (argc != 2){
        ROS_ERROR("missing filename for map!");
        exit(1);
    }
    //if (isValidPath(string(argv[1]))){
        getVoxelsFromFile(string(argv[1]));
    //}
    return 0;
}
