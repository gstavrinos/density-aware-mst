#include "damst/problem_def.hpp"

namespace damst {

    ProblemDefinition::ProblemDefinition(Graph* g, unsigned ss){
        graph = g;
        num_edges = boost::num_edges(g);
        pagmo::vector_double lb, ub;
        for (unsigned i=0; i<ss; i++) {
            lb.push_back(-1);
            ub.push_back(ss);
        }
        bounds = std::pair(lb, ub);
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

}

