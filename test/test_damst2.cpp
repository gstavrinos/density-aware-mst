#include "ros/ros.h"
#include "damst/damst.hpp"
#include "sensor_msgs/LaserScan.h"
#include "roboskel_msgs/LaserScans.h"
#include "roboskel_msgs/ClusteredLaserScans.h"

roboskel_msgs::LaserScans l;
ros::Subscriber sub;
ros::Publisher pub;

void laserCallback(const sensor_msgs::LaserScan& ls) {
    if (l.scans.size() < 1) {
        l.scans.push_back(ls);
    }
    else{
        l.scans.push_back(ls);
        damst::DensityAwareMST mst;
        roboskel_msgs::ClusteredLaserScans cls;
        cls = mst.opt(l, 3);
        sensor_msgs::LaserScan s = ls;
        // s.header = ls.header;
        // s.ranges = ls.ranges;
        s.intensities.clear();
        for (auto i:cls.cluster_map[0].cluster) {
            s.intensities.push_back(i);
        }
        pub.publish(s);
        l.scans.clear();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_damst2");
    ros::NodeHandle n;
    sub = n.subscribe("/scan", 1, laserCallback);
    pub = n.advertise<sensor_msgs::LaserScan>("testtt", 1);
    ros::spin();
    return 0;
}
