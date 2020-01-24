#pragma once

// General
#include <algorithm>
#include <iostream>
#include <fstream>

// Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <roboskel_msgs/LaserScans.h>

// Pagmo
#include "damst/problem_def.hpp"
#include <pagmo/problem.hpp>
#include <pagmo/algorithm.hpp>
#include <pagmo/population.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/algorithms/simulated_annealing.hpp>

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
        // TODO a proper constructor (maybe)
        // DensityAwareMST(std::vector, std::vector, unsigned int);

        ~DensityAwareMST(){}

        std::vector<EdgeDesc> generateTree(const unsigned);
        const std::vector<EdgeDesc>& getResult() const;
        void visualizeResultTree() const;
        void createDottyGraph() const;
        void printResultTree();
        std::vector<int> opt(const roboskel_msgs::LaserScans&, const unsigned);

    private:
        Graph* graph;
        std::vector<EdgeDesc> result;
        size_t num_nodes;

        std::vector<EdgeDesc> generateTree(const roboskel_msgs::LaserScans&, const unsigned);
        double dist(const roboskel_msgs::LaserScans*, const uint8_t, const uint8_t, const uint8_t, const uint8_t) const;
        unsigned numberOfEdges() const;
        void updateGraphBasedOnResult();
        double score(const Graph*) const;
        double score(const std::vector<double>) const;

};

}
