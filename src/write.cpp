#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

class write_cloud
{
	public:
		pcl::PointCloud<pcl::PointXYZ> final_cloud;

		void edgesCB(const sensor_msgs::PointCloud2 &input);
		// void surfaceCB(const sensor_msgs::PointCloud2 &input);
};

void write_cloud::edgesCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
    final_cloud = final_cloud + cloud;
}

// void write_cloud::surfaceCB(const sensor_msgs::PointCloud2 &input)
// {
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::fromROSMsg(input, cloud);
//     final_cloud = final_cloud + cloud;
// }

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;

    write_cloud L;

    // ros::Rate loop_rate(100);

    ros::Subscriber edges_sub = nh.subscribe("/laser_cloud_less_sharp", 10, &write_cloud::edgesCB, &L);
    // ros::Subscriber surface_sub = nh.subscribe("/laser_cloud_less_flat", 10, &write_cloud::surfaceCB, &L);

    while(ros::ok){
    	
    	
    	pcl::PointCloud<pcl::PointXYZ> cloud_to_save = L.final_cloud;
    	if(cloud_to_save.points.size()>0){
    		std::cout << "saving" << std::endl;
    		pcl::io::savePCDFileASCII ("map_kitti_30sec.pcd", cloud_to_save);
    	}

    	ros::spin();
        // loop_rate.sleep();
    }

    // ros::spin();
    

    return 0;
}

