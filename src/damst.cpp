#include "damst/damst.hpp"

namespace damst {

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const unsigned num_nodes) {
    std::cout << &edges[0] << std::endl;
    graph = new Graph(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    return result;
}

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const roboskel_msgs::LaserScans& ls, const unsigned nn) {
    // TODO the tree apparently is not correctly connected
    // becauseconnected_components does not return 1! fml!
    size_t ss = ls.scans.size();
    num_nodes = 0;
    for (unsigned i=0; i<ss; i++) {
        size_t rs = ls.scans[i].ranges.size(); 
        for (unsigned j=0; j<rs; j++) {
            if (j+1 < rs) {
                num_nodes++;
                edges.push_back(Edge(i*rs+j, i*rs+j+1));
                weights.push_back(dist(&ls, i, j, i, j+1));
            }
            if (i+1 < ss) {
                for (unsigned n=j-nn; n<=j+nn; n++) {
                    // I am checking with the current
                    // laserscan and not with the next one
                    // to avoid a call to .size(). I should
                    // probably be fine, since all laserscans
                    // are generated from the same driver.
                    // BUT: (TODO) If I ever encounter crashes,
                    // this is the place to look for first.
                    if (n >= 0 and n < rs) {
                        num_nodes++;
                        edges.push_back(Edge(i*rs+j, (i+1)*rs+n));
                        weights.push_back(dist(&ls, i, j, i+1, n));
                    }
                }
            }
        }
    }
    graph = new Graph(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    // updateGraphBasedOnResult();
    return result;
}

void DensityAwareMST::updateGraphBasedOnResult() {
    EdgeIter ei, ei_end, next;
    boost::tie(ei, ei_end) = boost::edges(*graph);
    // for (unsigned i=0; i < edges.size(); i++) {
        for (next=ei; ei != ei_end; ei=next) {
            next++;
            if (std::find(result.begin(), result.end(), *ei) == result.end()) {
                remove_edge(*ei, *graph);
            }
        }
    // }
}

std::vector<unsigned> DensityAwareMST::opt() const {
    // pagmo::problem prob(ProblemDefinition(graph));
    // pagmo::algorithm algo(pagmo::simulated_annealing());
    // pagmo::archipelago archi(16u, algo, prob, 20u);
    // archi.evolve(10);
    // archi.wait_check();
    // for (const auto &isl : archi) {
        // std::cout << isl.get_population().champion_f()[0] << std::endl;
    // }
    // pagmo::population pop(prob, *graph);
    // pop = algo.evolve(pop);
    // algo.evolve();
    // std::cout << pop << std::endl;

    std::vector<size_t> v;

    v.push_back(0);
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);

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

    std::vector<int> component2 (boost::num_vertices (*graph));
    size_t num_components2 = boost::connected_components(*graph, &component2[0]);
    for (size_t i=0; i < num_components; i++) {
        std::cout << i << std::endl;
        std::cout << component[i] << std::endl;
        std::cout << component2[i] << std::endl;
        std::cout << "---" << std::endl;
    }

    std::vector<std::vector<Edge>> sub_graph_edges;

    std::cout << num_components << std::endl;
    std::cout << num_components2 << std::endl;

    std::vector<unsigned> v_;
    return v_;
}

double DensityAwareMST::score(const Graph* g) const {
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

double DensityAwareMST::dist(const roboskel_msgs::LaserScans* ls, const uint8_t i1, const uint8_t j1, const uint8_t i2, const uint8_t j2) const {
    double r1 = ls->scans[i1].ranges[j1];
    double r2 = ls->scans[i2].ranges[j2];
    double theta1 = ls->scans[i1].angle_min + j1 * ls->scans[i1].angle_increment;
    double theta2 = ls->scans[i2].angle_min + j2 * ls->scans[i2].angle_increment;
    return sqrt(r1*r1 + r2*r2 - 2*r1*r2*(cos(theta1-theta2)));
}

// Seeing dots instead of edge labels?
// sudo apt install xfonts-100dpi
// and reboot
void DensityAwareMST::createDottyGraph() const {
    std::ofstream fout("damst_result.dot");
    fout << "Graph A {\n"
    << " rankdir=LR\n"
    << " size=\"5,5\"\n"
    << " ratio=\"filled\"\n"
    << " edge[style=\"bold\"]\n" << " node[shape=\"circle\"]\n";
    EdgeIter eiter, eiter_end;
    for (boost::tie(eiter, eiter_end) = boost::edges(*graph); eiter != eiter_end; eiter++) {
        std::cout << result[0] << std::endl;
        std::cout << *eiter << std::endl;
        fout << boost::source(*eiter, *graph) << " -- " << boost::target(*eiter, *graph);
        if (std::find(result.begin(), result.end(), *eiter) != result.end()){
            fout << "[color=\"black\", label=\"" << boost::get(boost::edge_weight, *graph, *eiter) << "\"];\n";
        }
        else{
            fout << "[color=\"gray\", label=\"" << boost::get(boost::edge_weight, *graph, *eiter) << "\"];\n";
        }
    }
    fout << "}\n";
}

unsigned DensityAwareMST::numberOfEdges() const {
    return edges.size();
}

void DensityAwareMST::printResultTree(){
    boost::property_map < Graph, boost::edge_weight_t >::type weight = boost::get(boost::edge_weight, *graph);
    for (std::vector < damst::DensityAwareMST::EdgeDesc >::iterator ei = result.begin(); ei != result.end(); ei++) {
        std::cout << boost::source(*ei, *graph) << " 🠜🠞 " << boost::target(*ei, *graph) << " with weight: " << weight[*ei] << std::endl;
    }
}

const std::vector<DensityAwareMST::EdgeDesc>& DensityAwareMST::getResult() const {
    return result;
}

void DensityAwareMST::visualizeResultTree() const {
    DensityAwareMST::createDottyGraph();
    std::system("dotty damst_result.dot &");
}

}

