#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "points_downsampler.h"

#define MAX_MEASUREMENT_RANGE 120.0

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static std::string POINTS_TOPIC;
static double measurement_range = MAX_MEASUREMENT_RANGE;

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_publish");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_map_without_voxel", 1);
    ros::Publisher filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_map", 1);

    ROS_INFO("\033[1;32m---->\033[0m pcl_publish started.");
    
    sensor_msgs::PointCloud2 outputMap;
    pcl::PointCloud<pcl::PointXYZ> MapCloud;

    pcl::io::loadPCDFile ("/home/pranavkdas/janak_WS/src/relocalisation/data/kitti2.pcd", MapCloud);
    pcl::toROSMsg(MapCloud, outputMap);

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ> MapCopy = MapCloud;

    if(measurement_range != MAX_MEASUREMENT_RANGE){
        MapCopy = removePointsByRange(MapCopy, 0, measurement_range);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(MapCopy));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    sensor_msgs::PointCloud2 filtered_msg;

    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (voxel_leaf_size >= 0.1)
    {
        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);
        pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
    }
    else
    {
        pcl::toROSMsg(*scan_ptr, filtered_msg);
    }

    filtered_msg.header.frame_id = "/camera_init";
    ////////////////////////////////////////////////////////////////////////////////////////////    

    

    outputMap.header.frame_id = "/camera_init";

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        pcl_pub_map.publish(outputMap);
        filtered_points_pub.publish(filtered_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ros::spin();

    return 0;
}




