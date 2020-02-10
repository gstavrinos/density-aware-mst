#include <cmath>
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
    else {
        l.scans.erase(l.scans.begin());
        l.scans.push_back(ls);
        l.header = ls.header;
        damst::DensityAwareMST mst;
        roboskel_msgs::ClusteredLaserScans s;
        s = mst.opt(l, 3);
        roboskel_msgs::LaserScans msg(l);
        for (size_t i=0;i<s.scans.size();i++) {
            msg.scans[i].intensities.clear();
            for (size_t j=0;j<s.scans[i].ranges.size();j++) {
                msg.scans[i].intensities.push_back(s.cluster_map[i].cluster[j]);
            }
        }
        sensor_msgs::LaserScan test;
        // Just get the first as a demo
        test = msg.scans[0];
        pub.publish(test);
        l.scans.clear();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_damst2");
    ros::NodeHandle n;
    sub = n.subscribe("/scan_filtered", 1, laserCallback);
    pub = n.advertise<sensor_msgs::LaserScan>("/damst_test2/scan", 1);
    ros::spin();
    return 0;
}
