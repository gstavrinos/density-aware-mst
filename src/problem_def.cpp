#include "damst/problem_def.hpp"

namespace damst {

    ProblemDefinition::ProblemDefinition(Graph* g){
        graph = g;
        num_edges = boost::num_edges(*g);
        pagmo::vector_double lb, ub;
        for (unsigned i=0; i<num_edges; i++) {
            lb.push_back(-1);
            ub.push_back(num_edges-1);
        }
        bounds = std::pair<pagmo::vector_double, pagmo::vector_double>(lb, ub);
    }

    pagmo::vector_double ProblemDefinition::fitness(const pagmo::vector_double &v) const {
        Graph gr = Graph(*graph);
        EdgeIter ei, ei_end, next;
        boost::tie(ei, ei_end) = boost::edges(*graph);
        double f = 0.0;

        for (auto i:v) {
            // TODO
            // no checks here, I think it is always safe
            // but I need to check
            next = ei;
            std::advance(next, (int)i);
            remove_edge(boost::source(*next, *graph), boost::target(*next, *graph), gr);
        }
        // TODO use connected_components here
        std::vector<int> component (boost::num_vertices (gr));
        size_t num_components = boost::connected_components(gr, &component[0]);

        for (size_t i=0; i < num_components; i++) {
            std::cout << i << std::endl;
            std::cout << component[i] << std::endl;
            std::cout << "---" << std::endl;
        }

        std::vector<std::vector<Edge>> sub_graph_edges;

        return {f};
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

