#include "ros/ros.h"
#include "damst/damst.hpp"
#include "sensor_msgs/LaserScan.h"
#include "roboskel_msgs/LaserScans.h"

roboskel_msgs::LaserScans l;
ros::Subscriber sub;


void laserCallback(const sensor_msgs::LaserScan& ls) {
    if (l.scans.size() < 3) {
        l.scans.push_back(ls);
    }
    else{
        sub.shutdown();
        damst::DensityAwareMST mst;
        mst.opt(l, 3);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_damst2");
    ros::NodeHandle n;
    sub = n.subscribe("/scan", 1, laserCallback);
    ros::spin();
    return 0;
}
