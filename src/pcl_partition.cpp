#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <relocalisation/updated_coord.h>


class updater
{
public:
	float a=0;
	float b=-1;
	float c=0;

    pcl::PointCloud<pcl::PointXYZ> cropped_map;
    pcl::PointCloud<pcl::PointXYZ> cropped_scan;

	void coordsCB(const relocalisation::updated_coord &input);
	void cloudCB(const sensor_msgs::PointCloud2 &input);
    void cloudFULLCB(const sensor_msgs::PointCloud2 &input);
};

	void updater::coordsCB(const relocalisation::updated_coord &input){
        a = input.x;
        b = input.y;
        c = input.z;
    }

    void updater::cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = a;
        center_point.y = b;
        center_point.z = c; 

        float radius = 40;
        std::vector<int> radiusIdx;
        std::vector<float> radiusSQDist;
        if (octree.radiusSearch (center_point, radius, radiusIdx, radiusSQDist) > 0)
        {
            for (size_t i = 0; i < radiusIdx.size (); ++i)
            {
                cloud_partitioned.points.push_back(cloud.points[radiusIdx[i]]);
            }
        }

        cropped_map = cloud_partitioned;
        
    }

    void updater::cloudFULLCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = 0;
        center_point.y = 0;
        center_point.z = 0;

        float radius = 40;
        std::vector<int> radiusIdx;
        std::vector<float> radiusSQDist;
        if (octree.radiusSearch (center_point, radius, radiusIdx, radiusSQDist) > 0)
        {
            for (size_t i = 0; i < radiusIdx.size (); ++i)
            {
                cloud_partitioned.points.push_back(cloud.points[radiusIdx[i]]);
            }
        }

        pcl::PointCloud<pcl::PointXYZ> final_cloud = cloud_partitioned;

        for (int i = 0; i < cloud_partitioned.points.size(); i++) {

            final_cloud.points[i].x = cloud_partitioned.points[i].y;
            final_cloud.points[i].y = cloud_partitioned.points[i].z;
            final_cloud.points[i].z = cloud_partitioned.points[i].x;
        }

        cropped_scan = final_cloud;
    }

int main(int argc, char **argv){

    ros::init(argc, argv, "final");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033 [0m pcl_partition started");

    updater L;

    ros::Subscriber pcl_sub = nh.subscribe("pcl_map", 10, &updater::cloudCB,&L);
    ros::Subscriber coords_sub = nh.subscribe("updated_coord", 10, &updater::coordsCB,&L);
    ros::Subscriber pcl_sub_main = nh.subscribe("full_cloud_projected", 10, &updater::cloudFULLCB,&L);

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_target", 1);
    ros::Publisher pcl_pub_main = nh.advertise<sensor_msgs::PointCloud2>("main_cloud", 1);

    // ros::spin();
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {	

        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(L.cropped_map, output);

        output.header.frame_id = "/new_init";
        pcl_pub.publish(output);

        pcl::toROSMsg(L.cropped_scan, output);

        output.header.frame_id = "/camera_init";
        pcl_pub_main.publish(output);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}