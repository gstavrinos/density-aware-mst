#pragma once

// General
#include <algorithm>
#include <iostream>
#include <fstream>

// Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <roboskel_msgs/LaserScans.h>
#include "roboskel_msgs/ClusteredLaserScans.h"
#include "roboskel_msgs/LaserScanCluster.h"

namespace damst {

class DensityAwareMST{

    public:
        using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double>>;
        using EdgeDesc = boost::graph_traits<Graph>::edge_descriptor;
        using EdgeIter = boost::graph_traits<Graph>::edge_iterator;
        using Edge = std::pair<unsigned, unsigned>;
        std::vector<Edge> edges;
        std::vector<double> weights;

        DensityAwareMST(){}

        ~DensityAwareMST(){}

        roboskel_msgs::ClusteredLaserScans opt(const roboskel_msgs::LaserScans&, const unsigned);
        void visualizeResultTree() const;
        void createDottyGraph() const;
        void printResultTree();

    private:
        Graph* graph;
        std::vector<EdgeDesc> result;
        size_t num_nodes;

        size_t generateTree(const roboskel_msgs::LaserScans&, const unsigned);
        double dist(const roboskel_msgs::LaserScans*, const size_t, const size_t, const size_t, const size_t) const;
        unsigned numberOfEdges() const;
        void updateGraphBasedOnResult(const roboskel_msgs::LaserScans& ls);
        double score(const std::vector<double>) const;

};

}
