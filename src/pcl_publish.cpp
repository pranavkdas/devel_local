#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>




main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_publish");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_map", 1);
    // ros::Publisher pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_scan", 1);
    ROS_INFO("\033[1;32m---->\033[0m pcl_publish started.");
    sensor_msgs::PointCloud2 outputMap;
    sensor_msgs::PointCloud2 outputScan;
    pcl::PointCloud<pcl::PointXYZ> MapCloud;
    pcl::PointCloud<pcl::PointXYZ> ScanCloud;

    // pcl::io::loadPCDFile ("/home/pranavkdas/janak_WS/src/relocalisation/data/area.pcd", ScanCloud);
    pcl::io::loadPCDFile ("/home/pranavkdas/janak_WS/src/relocalisation/data/kitti2.pcd", MapCloud);

    pcl::toROSMsg(MapCloud, outputMap);
    // pcl::toROSMsg(ScanCloud, outputScan);

    outputMap.header.frame_id = "/camera_init";
    // outputScan.header.frame_id = "/camera_init";

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        pcl_pub_map.publish(outputMap);
        // pcl_pub_scan.publish(outputScan);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

