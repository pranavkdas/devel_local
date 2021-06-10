#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

//main(int argc, char **argv)
//{
    //ros::init (argc, argv, "pcl_read");

    

class pcl_reader
{    
public:
    pcl_reader(){
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher pcl_pub_2 = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output2", 1);
    ros::Subscriber pcl_sub = nh.subscribe("pcl_publish_output", 1, &pcl_reader::cloud_CB, this);
    
    }

    //CALLBACK FUNCTION FOR PCL_SUB
    void cloud_CB(const sensor_msgs::PointCloud2 &input){
        pcl::PointCloud<pcl::PointXYZ> cloud1;
        pcl::fromROSMsg(input, cloud1);

        sensor_msgs::PointCloud2 read_output;
        sensor_msgs::PointCloud2 read_output2;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        //pcl::io::loadPCDFile ("/home/janak/pcd_new_kitti/1317379226.699959039.pcd", cloud);

        pcl::toROSMsg(cloud1, read_output);
        read_output.header.frame_id = "/camera_init";

        pcl::io::loadPCDFile ("/home/janak/pcd_new_kitti/map/new_kitti.pcd", cloud);

        pcl::toROSMsg(cloud, read_output2);
        read_output2.header.frame_id = "/camera_init";

        pcl_pub.publish(read_output);
        pcl_pub_2.publish(read_output2);


    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub_2;
};

//}

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_read");
    pcl_reader reader;
    ros::spin();

    return 0;
}


