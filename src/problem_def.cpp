#include "damst/problem_def.hpp"

namespace damst {

    ProblemDefinition::ProblemDefinition(Graph* g, unsigned ss){
        graph = g;
        num_edges = boost::num_edges(*g);
        pagmo::vector_double lb, ub;
        for (unsigned i=0; i<ss; i++) {
            lb.push_back(-1);
            ub.push_back(ss);
        }
        bounds = std::pair<pagmo::vector_double, pagmo::vector_double>(lb, ub);
    }

    pagmo::vector_double ProblemDefinition::fitness(const pagmo::vector_double &v) const {
        // TODO this is a placeholder
        // I have to create subgraphs and pass them to the score function of the damst class
        return {v[0]*2};
    }

    std::pair<pagmo::vector_double, pagmo::vector_double> ProblemDefinition::get_bounds() const {
        return bounds;
    }

    std::string ProblemDefinition::get_name() const {
        return "Density Aware MST";
    }

    double ProblemDefinition::score(const Graph* g) const {
        unsigned num_nodes = 0;
        double tot_weight = 0.0;
        boost::property_map < Graph, boost::edge_weight_t >::type weight = boost::get(boost::edge_weight, *graph);
        ProblemDefinition::EdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::edges(*g); ei != ei_end; ei++) {
            num_nodes++;
            tot_weight += weight[*ei];
        }
        return num_nodes / (1000 * tot_weight);
    }

}

