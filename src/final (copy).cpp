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

typedef PointMatcher<float> PM;

class listener
{
	public:
	    pcl::PointCloud<pcl::PointXYZ> map;
	    pcl::PointCloud<pcl::PointXYZ> scan;
	    
	    void mapCB(const sensor_msgs::PointCloud2 &input);
	    void scanCB(const sensor_msgs::PointCloud2 &input);
};

	void listener::mapCB(const sensor_msgs::PointCloud2 &input)
	{
	    pcl::fromROSMsg(input, map);
	}

	void listener::scanCB(const sensor_msgs::PointCloud2 &input)
	{
	    pcl::fromROSMsg(input, scan);
	}

	pcl::PointCloud<pcl::PointXYZ> do_icp(pcl::PointCloud<pcl::PointXYZ> &reference_area ,pcl::PointCloud<pcl::PointXYZ> &incoming_scan)
	{
		
		sensor_msgs::PointCloud2 ref_area;
        pcl::toROSMsg(reference_area,ref_area);

        sensor_msgs::PointCloud2 scan_in;
        pcl::toROSMsg(incoming_scan,scan_in);

        PM::ICP icp;
        
        PM::DataPoints object= PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ref_area, false);
        
        PM::DataPoints scene= PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan_in, false);

        PointMatcherSupport::Parametrizable::Parameters params;
        std::string name;
        
        // Uncomment for console outputs
        // setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

        // Prepare reading filters
        name = "MinDistDataPointsFilter";
        params["minDist"] = "1.0";
        std::shared_ptr<PM::DataPointsFilter> minDist_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        name = "RandomSamplingDataPointsFilter";
        params["prob"] = "0.05";
        std::shared_ptr<PM::DataPointsFilter> rand_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare reference filters
        name = "MinDistDataPointsFilter";
        params["minDist"] = "1.0";
        std::shared_ptr<PM::DataPointsFilter> minDist_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        name = "RandomSamplingDataPointsFilter";
        params["prob"] = "0.05";
        std::shared_ptr<PM::DataPointsFilter> rand_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare matching function
        name = "KDTreeMatcher";
        params["knn"] = "1";
        params["epsilon"] = "3.16";
        std::shared_ptr<PM::Matcher> kdtree =
            PM::get().MatcherRegistrar.create(name, params);
        params.clear();

        // Prepare outlier filters
        name = "TrimmedDistOutlierFilter";
        params["ratio"] = "0.75";
        std::shared_ptr<PM::OutlierFilter> trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
        params.clear();

        // Prepare error minimization
        name = "PointToPointErrorMinimizer";
        std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
            PM::get().ErrorMinimizerRegistrar.create(name);

        // Prepare transformation checker filters
        name = "CounterTransformationChecker";
        params["maxIterationCount"] = "150";
        std::shared_ptr<PM::TransformationChecker> maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        name = "DifferentialTransformationChecker";
        params["minDiffRotErr"] = "0.001";
        params["minDiffTransErr"] = "0.01";
        params["smoothLength"] = "4";
        std::shared_ptr<PM::TransformationChecker> diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        // Prepare inspector
        std::shared_ptr<PM::Inspector> nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");

        //  name = "VTKFileInspector";
        //  params["dumpDataLinks"] = "1";
        //  params["dumpReading"] = "1";
        //  params["dumpReference"] = "1";
        //  std::shared_ptr<PM::Inspector> vtkInspect =
        //      PM::get().InspectorRegistrar.create(name, params);
        //  params.clear();

        // Prepare transformation
        std::shared_ptr<PM::Transformation> rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");
        
        // Build ICP solution
        icp.readingDataPointsFilters.push_back(minDist_read);
        icp.readingDataPointsFilters.push_back(rand_read);

        icp.referenceDataPointsFilters.push_back(minDist_ref);
        icp.referenceDataPointsFilters.push_back(rand_ref);

        icp.matcher = kdtree;
        
        icp.outlierFilters.push_back(trim);
        
        icp.errorMinimizer = pointToPoint;

        icp.transformationCheckers.push_back(maxIter);
        icp.transformationCheckers.push_back(diff);
        
        // toggle to write vtk files per iteration
        icp.inspector = nullInspect;
        //icp.inspector = vtkInspect;

        icp.transformations.push_back(rigidTrans);

        
        PM::TransformationParameters T = icp(object, scene);

        // std::cout << "Transformation Matrix = \n" << T << std::endl;

        PM::DataPoints transformed_object(object);
        icp.transformations.apply(transformed_object, T);

        sensor_msgs::PointCloud2 transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformed_object, "/camera_init", ros::Time::now());
    	
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud_pcl;

    	pcl::fromROSMsg(transformed_pcd, transformed_cloud_pcl);

    	return transformed_cloud_pcl;

	}

	relocalisation::updated_coord find_position(pcl::PointCloud<pcl::PointXYZ> &input) 
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
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

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

        // for (size_t i = 0; i < cloud_partitioned.points.size (); ++i)
        // {
        //     cloud_partitioned.points[i].x = cloud_partitioned.points[i].x -center_point.x;
        //     cloud_partitioned.points[i].y = cloud_partitioned.points[i].y -center_point.y;
        //     cloud_partitioned.points[i].z = cloud_partitioned.points[i].z -center_point.z;
        //     // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        // }
        // pcl::toROSMsg(cloud_partitioned, output);
        // output.header.frame_id = "/new_init";
        
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

//////////////////////////////////////
// Do allocation and reset tomorrow //
//////////////////////////////////////
    
    // void allocateMemory(){

    //     laserCloudIn.reset(new pcl::PointCloud<PointType>());

    //     fullCloud.reset(new pcl::PointCloud<PointType>());
    //     fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    //     groundCloud.reset(new pcl::PointCloud<PointType>());
    //     segmentedCloud.reset(new pcl::PointCloud<PointType>());
    //     segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    //     outlierCloud.reset(new pcl::PointCloud<PointType>());

    //     fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    //     fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

    //     segMsg.startRingIndex.assign(N_SCAN, 0);
    //     segMsg.endRingIndex.assign(N_SCAN, 0);

    //     segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
    //     segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
    //     segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

    //     std::pair<int8_t, int8_t> neighbor;
    //     neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    //     neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    //     neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    //     neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

    //     allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    //     allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    //     queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    //     queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    // }


int main(int argc, char **argv){

    ros::init(argc, argv, "final");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033 [Major workback starting.");

    listener L;

    ros::Subscriber pcl_sub_map = nh.subscribe("pcl_map", 10, &listener::mapCB, &L);
    ros::Subscriber pcl_sub_scan = nh.subscribe("full_cloud_projected", 10, &listener::scanCB, &L);

    // ros::Publisher pub_position = nh.advertise<relocalisation::updated_coord>("final_point",1);
    // ros::Publisher pub_test = nh.advertise<sensor_msgs::PointCloud2>("check",1);
    ros::Publisher pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);


    ros::Rate loop_rate(10);

    // Initial co-ordinates
    float coord_x = 0;
	float coord_y = 0;
	float coord_z = 0;

    while(ros::ok())
    {	
    	std::cout << "loop_start" << std::endl;
	    pcl::PointCloud<pcl::PointXYZ> rotated_scan = rotate_pointcloud(L.scan);

        pcl::PointCloud<pcl::PointXYZ> area = extract_trim_area(L.map,coord_x,coord_y,coord_z);
        pcl::PointCloud<pcl::PointXYZ> scan = extract_trim_area(rotated_scan,0,0,0);

        if (area.points.size()>0 && scan.points.size()>0 )
        {
	        pcl::PointCloud<pcl::PointXYZ> aligned_scan = do_icp(area,scan);

	        sensor_msgs::PointCloud2 transformed_pcd;
	        pcl::toROSMsg(aligned_scan, transformed_pcd);
	        transformed_pcd.header.frame_id = "/camera_init";
        	pcl_pub_aligned.publish(transformed_pcd);

	        relocalisation::updated_coord position = find_position(aligned_scan);

	        coord_x = position.x;
	        coord_y = position.y;
	        coord_z = position.z;

	        visualization_msgs::Marker current_position = make_marker(coord_x,coord_y,coord_z);
	        marker_pub.publish(current_position);

	        std::cout << position.x << " "<< position.y <<" "<< position.z << std::endl;
	    }

		ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}