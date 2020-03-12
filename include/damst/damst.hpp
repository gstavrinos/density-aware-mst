#pragma once

// General
#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>

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
        using VertexDesc = boost::graph_traits<Graph>::vertex_descriptor;
        using DegreeSize = boost::graph_traits<Graph>::degree_size_type;
        using EdgeIter = boost::graph_traits<Graph>::edge_iterator;
        using Edge = std::pair<unsigned, unsigned>;
        std::vector<Edge> edges;
        std::vector<double> weights;
        std::vector<std::pair<double,double>> vertices;

        DensityAwareMST(){}

        ~DensityAwareMST(){}

        roboskel_msgs::ClusteredLaserScans opt(const roboskel_msgs::LaserScans&, const unsigned);
        std::pair<std::vector<int>, int> opt(const std::vector<std::pair<double, double>>, const bool=false);
        std::pair<std::vector<int>, int> opt2(const std::vector<std::pair<double, double>>, const bool=false);
        void visualizeResultTree() const;
        void createDottyGraph() const;
        void printResultTree();

    private:
        std::shared_ptr<Graph> graph;
        std::vector<EdgeDesc> result;
        size_t num_nodes;

        size_t generateTree(const roboskel_msgs::LaserScans&, const unsigned);
        void generateTree(const std::vector<std::pair<double, double>>);
        double dist(const roboskel_msgs::LaserScans&, const size_t, const size_t, const size_t, const size_t) const;
        double dist(const std::pair<double, double>, const std::pair<double, double>) const;
        void dbgSave(const std::vector<Edge>);
        void dbgSave(const std::vector<bool>);
        bool stopt();
        unsigned numberOfEdges() const;
        void updateGraphBasedOnResult(const roboskel_msgs::LaserScans& ls);
        void updateGraphBasedOnResult(const std::vector<std::pair<double, double>>);
        double score(const std::vector<double>) const;
        double score(const std::vector<Edge>, const std::vector<double>, const Graph) const;
        double score(const std::vector<std::pair<double,double>>, const std::vector<Edge>, const std::vector<double>, const Graph) const;

};

}
