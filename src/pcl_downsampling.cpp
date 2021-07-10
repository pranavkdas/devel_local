#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <relocalisation/updated_coord.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

void find_position(pcl::PointCloud<pcl::PointXYZ> &input) 
    {   
        // To find current position

        Eigen::Vector4f centroid;
    
        pcl::compute3DCentroid(input, centroid);

        std::cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
    }

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_scan = nh.subscribe("/laser_cloud_sharp", 10, &cloudHandler::cloudCB_scan, this);
        // pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled_scan", 1);
    }

    void cloudCB_scan(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        // pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        // sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);
        find_position(cloud);

        // pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        // voxelSampler.setInputCloud(cloud.makeShared());
        // voxelSampler.setLeafSize(2.0, 2.0, 2.0);
        // voxelSampler.filter(cloud_downsampled);

        // pcl::toROSMsg(cloud_downsampled, output);
        // pcl_pub_scan.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_targ;
    ros::Publisher pcl_pub_scan;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_downsampling");

    cloudHandler handler;

    ros::spin();

    return 0;
}

