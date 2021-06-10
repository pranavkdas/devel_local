#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <unordered_map>
#include <array>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <array>
#include <fstream>
#include <iomanip>  // std::setprecision()

#define N 6

// sensor_msgs::NavSatFix var_gps;
// std::vector<std::array<long double, N>> vect;

long double a,b,c;
long double x,y,z;


class gpsHandler
{
public:
    gpsHandler()
    {
        gps_sub = nh.subscribe("/kitti/oxts/gps/fix", 10, &gpsHandler::gpsCB, this);
        odom_sub = nh.subscribe("/integrated_to_init", 10, &gpsHandler::odomCB, this);
        // coords_sub = nh.subscribe("/kitti/oxts/gps/fix", 10, &gpsHandler::gpsCB, this);
        // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_partitioned", 1);
    }

    // A small error can creep up because we are directly subscribing to the raw gps data and x,y,z data needs to come through lego loam
    // This causes some delay in between the first few GPS and xyz msgs, after which they come one after the other. 
    // GPS GPS GPS XYZ GPS XYZ GPS XYZ ...
    void gpsCB(const sensor_msgs::NavSatFixConstPtr& msg)
    {   
        a = msg->latitude;
        b = msg->longitude;
        c = msg->altitude;
        // std::cout << "GPS" <<" "<< a <<" " <<  b<<" " << c << "\n";  
    }
    void print(std::vector<std::array<long double, N> > v)
    {
  
    // Displaying the vector of arrays
    // ranged for loop is supported
    for (std::array<long double, N> i : v) {
        for (auto x : i)
            cout << x << " ";
        cout << endl;
    }
}
    void odomCB(const nav_msgs::OdometryConstPtr& msg)
    {   
        x=0;
        y=0;
        z=0;
        x= msg->pose.pose.position.x;
        y= msg->pose.pose.position.y;
        z= msg->pose.pose.position.z;

        // vect.push_back({a,b,c,x,y,z});

        // gpsHandler::print(vect);
        std::cout << setprecision(13);
        std::cout << "VECTOR" <<" "<< a << " " << b << " " << c << " " << x << " " << y << " " << z<<"\n";
        std::ofstream outfile;

        outfile.open("/home/pranavkdas/janak_WS/src/relocalisation/data/gps_list.txt", std::ios_base::app); // append instead of overwrite
        

        outfile << std::fixed << setprecision(13) << endl;
        outfile << a << " " << b << " " << c << " " << x << " " << y << " " << z; 
    }
    // void odomCB(
    //     keep 6 global variables; latitude,longitude,altitude,x,y,z position
    //     during every instance, let the call back functions update these values.
    //     Let a separate function take all of them together and store them as a vector of vector made of long double
    //     )
    // After this save this dictionary and process it so that;
    // a dictionary is made for latitudes and longitudes comprising of their first 4decimal points as key, 
    // and the latitudes and longitudes with full expansion is stored as a list in values.
    // Once this is done you can find the incoming latitude and longitude which is most closer to the current one, and look for 
    // x,y position from a similar dictionary in a similar manner.
    

protected:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Subscriber odom_sub;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "gps_dict");

    gpsHandler handler;

    ros::spin();

    return 0;
}

