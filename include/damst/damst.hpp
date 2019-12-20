#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <roboskel_msgs/LaserScans.h>

namespace damst {

class DensityAwareMST{

    public:
        using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double>>;
        using EdgeDesc = boost::graph_traits<Graph>::edge_descriptor;
        using Edge = std::pair<unsigned int, unsigned int>;
        // For laserscans, we use two "edges" to describe an edge
        // the first edge (pair) consists of the indeces of the first laserscan and point
        // and the second edge (pair) consists of the indeces of the second laserscan and point
        using LaserScanEdge = std::pair<Edge, Edge>;
        std::vector<Edge> edges;
        std::vector<double> weights;

        DensityAwareMST(){}
        // TODO a proper constructor (maybe)
        // DensityAwareMST(std::vector, std::vector, unsigned int);

        ~DensityAwareMST(){}

        std::vector<EdgeDesc> generateTree(const roboskel_msgs::LaserScans&);
        std::vector<EdgeDesc> generateTree(const unsigned int);
        const std::vector<EdgeDesc>& getResult() const;
        void visualizeReultTree() const;
        void createDottyGraph() const;
        void printResultTree();

    private:
        Graph* graph;
        std::vector<LaserScanEdge> ls_edges;
        std::vector<EdgeDesc> result;
        unsigned int numberOfEdges() const;

};

}
