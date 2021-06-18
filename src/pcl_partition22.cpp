#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <relocalisation/updated_coord.h>

float a=0;
float b=-1;
float c=0;
int p = 0;
class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_map", 10, &cloudHandler::cloudCB, this);
        coords_sub = nh.subscribe("updated_coord", 10, &cloudHandler::coordsCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_target", 1);
        pcl_pub_main = nh.advertise<sensor_msgs::PointCloud2>("main_cloud", 1);
        pcl_sub_main = nh.subscribe("full_cloud_projected", 10, &cloudHandler::cloudFULLCB, this);
    }

    void coordsCB(const relocalisation::updated_coord &input){
        a = input.x;
        b = input.y;
        c = input.z;
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);
        
        sensor_msgs::PointCloud2 output;

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = a;
        center_point.y = b;
        center_point.z = c; // Just for testing.
        // center_point.z = 50;
        // std::cout << center_point.x << "\n";
        

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

        // for (size_t i = 0; i < cloud_partitioned.points.size (); ++i)
        // {
        //     cloud_partitioned.points[i].x = cloud_partitioned.points[i].x -center_point.x;
        //     cloud_partitioned.points[i].y = cloud_partitioned.points[i].y -center_point.y;
        //     cloud_partitioned.points[i].z = cloud_partitioned.points[i].z -center_point.z;
        //     // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        // }


        pcl::toROSMsg(cloud_partitioned, output);

        output.header.frame_id = "/new_init";
        pcl_pub.publish(output);
        // std::cout << "area" << "\n";
    }

    void cloudFULLCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);
        
        sensor_msgs::PointCloud2 output;

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = 0;
        center_point.y = 0;
        center_point.z = 0; // Just for testing.
        // center_point.z = 50;

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
        // for (size_t i = 0; i < cloud_partitioned.points.size (); ++i)
        // {
        //     cloud_partitioned.points[i].x = cloud_partitioned.points[i].x -center_point.x;
        //     cloud_partitioned.points[i].y = cloud_partitioned.points[i].y -center_point.y;
        //     cloud_partitioned.points[i].z = cloud_partitioned.points[i].z -center_point.z;
        //     // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        // }


        pcl::toROSMsg(final_cloud, output);

        output.header.frame_id = "/camera_init";
        pcl_pub_main.publish(output);
        // std::cout << "cloud" << "\n";
    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Subscriber coords_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub_main;
    ros::Subscriber pcl_sub_main;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_partition");

    cloudHandler handler;

    ros::spin();

    return 0;
}