#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <relocalisation/updated_coord.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"
#include <visualization_msgs/Marker.h>
#include <relocalisation/updated_coord.h>
#include <std_msgs/Float64.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "points_downsampler.h"
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

class listener
{
	public:
	    pcl::PointCloud<pcl::PointXYZ> map;
	    pcl::PointCloud<pcl::PointXYZ> scan;
        sensor_msgs::PointCloud2 filtered_msg;

	    float p,q,r; // translation from icp
	    float pose[7];

	    pcl::PointCloud<pcl::PointXYZ> traj;
	    
	    void mapCB(const sensor_msgs::PointCloud2 &input);
	    void scanCB(const sensor_msgs::PointCloud2 &input);
	    void imuCB(const nav_msgs::Odometry &input);
        
        pcl::PointCloud<pcl::PointXYZ> do_icp(pcl::PointCloud<pcl::PointXYZ> &reference_area ,pcl::PointCloud<pcl::PointXYZ> &incoming_scan);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud )
};

	void listener::mapCB(const sensor_msgs::PointCloud2 &input)
	{
	    pcl::fromROSMsg(input, map);
	}

	void listener::scanCB(const sensor_msgs::PointCloud2 &input)
	{  
        pcl::fromROSMsg(input, scan);
	}


	void listener::imuCB(const nav_msgs::Odometry &input)
	{

        // pose[0] = laserOdometry.header.stamp = cloudHeader.stamp;
        pose[0] = input.pose.pose.orientation.x;
        pose[1] = input.pose.pose.orientation.y;
        pose[2] = input.pose.pose.orientation.z;
        pose[3] = input.pose.pose.orientation.w;
        pose[4] = input.pose.pose.position.x;
        pose[5] = input.pose.pose.position.y;
        pose[6] = input.pose.pose.position.z;

        std::cout << "IMU : " << pose[4] << " " << pose[5] <<" " << pose[6] << std::endl;
	}


    pcl::PointCloud<pcl::PointXYZ> pre_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
        // pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        //     std::cerr << "failed to load " << target_pcd << std::endl;
        //     return 0;
        // }
        // if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        //     std::cerr << "failed to load " << source_pcd << std::endl;
        //     return 0;
        // }

        // downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

        voxelgrid.setInputCloud(target_cloud);
        voxelgrid.filter(*downsampled);
        *target_cloud = *downsampled;

        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;

        std::cout << "--- pclomp::NDT ---" << std::endl;

        int num_threads = omp_get_max_threads();

        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_omp->setResolution(1.0);

        std::cout << "--- DIRECT7" << ", " << num_threads << " threads ---" << std::endl;
        ndt_omp->setNumThreads(num_threads);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

        ndt_omp->setInputTarget(target_cloud);
        ndt_omp->setInputSource(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

        auto t1 = ros::WallTime::now();
        ndt_omp->align(*aligned);
        auto t2 = ros::WallTime::now();
        Eigen::Matrix4f transformation = ndt_omp->getFinalTransformation();

        std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
        std::cout << "fitness: " << ndt_omp->getFitnessScore() << std::endl;
        std::cout << "transform: " << transformation << std::endl << std::endl;
        

        return *aligned;

    }
	          
    visualization_msgs::Marker make_marker(float a,float b,float c)
	{   
		// Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/camera_init";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = a;
        marker.pose.position.y = b;
        marker.pose.position.z = c;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0);

        return marker;
	}

	pcl::PointCloud<pcl::PointXYZ> extract_trim_area(pcl::PointCloud<pcl::PointXYZ> &input,float a,float b,float c)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (input.makeShared());
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
                cloud_partitioned.points.push_back(input.points[radiusIdx[i]]);
            }
        }

        return cloud_partitioned;
        
    }

    pcl::PointCloud<pcl::PointXYZ> rotate_pointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_partitioned)
    {
    	pcl::PointCloud<pcl::PointXYZ> final_cloud = cloud_partitioned;

        for (int i = 0; i < cloud_partitioned.points.size(); i++) {

            final_cloud.points[i].x = cloud_partitioned.points[i].y;
            final_cloud.points[i].y = cloud_partitioned.points[i].z;
            final_cloud.points[i].z = cloud_partitioned.points[i].x;
        }

        return final_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ> translate_pointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_partitioned, float a,float b,float c)
    {
    	pcl::PointCloud<pcl::PointXYZ> final_cloud = cloud_partitioned;

        for (int i = 0; i < cloud_partitioned.points.size(); i++) {

            final_cloud.points[i].x -= a;
            final_cloud.points[i].y -= b;
            final_cloud.points[i].z -= c;
        }

        return final_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ> trajectory_pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, float a,float b,float c)
    {
    	pcl::PointCloud<pcl::PointXYZ> final_cloud = cloud;
    	pcl::PointXYZ new_point;

    	new_point.x = a;
    	new_point.y = b;
    	new_point.z = c;

        final_cloud.points.push_back(new_point);

        return final_cloud;
    }

    
    relocalisation::updated_coord find_position(float a, float b, float c) 
    {   

        relocalisation::updated_coord new_msg;
        new_msg.x = -a;
        new_msg.y = -b;
        new_msg.z = -c;

        return new_msg;
    }

    relocalisation::updated_coord find_position2(pcl::PointCloud<pcl::PointXYZ> &input) 
    {   
        // To find current position

        Eigen::Vector4f centroid;
    
        pcl::compute3DCentroid(input, centroid);

        relocalisation::updated_coord new_msg;
        new_msg.x = centroid[0];
        new_msg.y = centroid[1];
        new_msg.z = centroid[2];

        return new_msg;
        // coords_pub.publish(new_msg);
    }

int main(int argc, char **argv){

    ros::init(argc, argv, "final");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033[0m Major workback starting.");
    ros::Time::init();
    listener L;

    ros::Subscriber pcl_sub_map = nh.subscribe("/pcl_map_without_voxel", 10, &listener::mapCB, &L);
    // ros::Subscriber pcl_sub_map = nh.subscribe("/pcl_map", 10, &listener::mapCB, &L);
    ros::Subscriber pcl_sub_scan = nh.subscribe("/full_cloud_projected", 10, &listener::scanCB, &L);
    ros::Subscriber imu_sub = nh.subscribe("/laser_odom_to_init", 10 , &listener::imuCB, &L);

    // ros::Publisher pub_position = nh.advertise<relocalisation::updated_coord>("final_point",1);
    ros::Publisher pub_test = nh.advertise<sensor_msgs::PointCloud2>("check",1);
    ros::Publisher pub_trajectory = nh.advertise<sensor_msgs::PointCloud2>("trajectory",1);
    ros::Publisher pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_scan", 1);

    ros::Rate loop_rate(10);

    // Initial co-ordinates
    float coord_x = 0;
	float coord_y = 0;
	float coord_z = 0;

	for(int i=0;i<7;i++){
		L.pose[i] = 0;
	}
    float yo =0;

    while(ros::ok())
    {	
    	std::cout << "loop_start" << std::endl;

        // filtered_points_pub.publish(L.filtered_msg);

    	relocalisation::updated_coord temp;
	    temp.x = coord_x;
	    temp.y = coord_y;
	    temp.z = coord_z;

	    pcl::PointCloud<pcl::PointXYZ> rotated_scan = rotate_pointcloud(L.scan);

        pcl::PointCloud<pcl::PointXYZ> select_area = extract_trim_area(L.map,temp.x,temp.y,temp.z);

        pcl::PointCloud<pcl::PointXYZ> scan = extract_trim_area(rotated_scan,0,0,0);

	    pcl::PointCloud<pcl::PointXYZ> area = translate_pointcloud(select_area,temp.x,temp.y,temp.z);

        sensor_msgs::PointCloud2 laserCloudOutMsg;

        pcl::toROSMsg(select_area, laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "/camera_init";
        pub_test.publish(laserCloudOutMsg);
        

        if (area.points.size()>0 && scan.points.size()>0 )
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr area_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            *area_ptr = area;
            *scan_ptr = scan;

         //    Eigen::Matrix4f tf = pre_icp( scan_ptr,area_ptr);

	        // relocalisation::updated_coord position = find_position(tf(0,3), tf(1,3), tf(2,3));

            pcl::PointCloud<pcl::PointXYZ> tf = pre_icp( scan_ptr,area_ptr);

            relocalisation::updated_coord position = find_position2(tf);

            // std::cout <<tf(0,3) << " " << tf(1,3) <<" " << tf(2,3) << endl;
	        // std::cout << position.x << " " << temp.x << endl;
	        // std::cout << position.y << " " << temp.y << endl;
	        // std::cout << position.z << " " << temp.z << endl;

	        // std::cout << position.x << " "<< position.y <<" "<< position.z << std::endl;


	        coord_x = position.x + temp.x; 
	        coord_y = position.y;
	        coord_z = position.z + temp.z;

	        // pcl::PointCloud<pcl::PointXYZ> trajectory = trajectory_pcl(L.traj,coord_x,coord_y,coord_z);

	        // sensor_msgs::PointCloud2 laserCloudOutMsg;
	        // pcl::toROSMsg(trajectory, laserCloudOutMsg);
	        // laserCloudOutMsg.header.frame_id = "/camera_init";
	        // pub_trajectory.publish(laserCloudOutMsg);

	        visualization_msgs::Marker current_position = make_marker(coord_x,coord_y,coord_z);
	        marker_pub.publish(current_position);

	        std::cout << "COORDS : " << coord_x << " "<< coord_y <<" "<< coord_z << std::endl;
           
	    }
        

		ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}