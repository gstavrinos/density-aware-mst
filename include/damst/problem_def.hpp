#pragma once

// General
#include <string>

// Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

// Pagmo
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>

namespace damst {

class ProblemDefinition {

    public:
        using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double>>;
        using EdgeDesc = boost::graph_traits<Graph>::edge_descriptor;
        using EdgeIter = boost::graph_traits<Graph>::edge_iterator;
        using Edge = std::pair<unsigned, unsigned>;

        ProblemDefinition() {}
        ProblemDefinition(Graph*);

        pagmo::vector_double fitness(const pagmo::vector_double &) const;
        std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;
        std::string get_name() const;

        double score(const Graph*) const;

    private:
        Graph* graph;
        unsigned num_edges;
        std::pair<pagmo::vector_double, pagmo::vector_double> bounds;

};

}

