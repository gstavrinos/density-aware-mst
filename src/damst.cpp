#include "damst/damst.hpp"

namespace damst {

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const unsigned num_nodes) {
    std::cout << &edges[0] << std::endl;
    graph = new Graph(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    return result;
}

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const roboskel_msgs::LaserScans& ls, const unsigned nn) {
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
                        // num_nodes++;
                        edges.push_back(Edge(i*rs+j, (i+1)*rs+n));
                        weights.push_back(dist(&ls, i, j, i+1, n));
                    }
                }
            }
        }
    }
    graph = new Graph(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    updateGraphBasedOnResult();
    return result;
}

void DensityAwareMST::updateGraphBasedOnResult() {
    EdgeIter ei, ei_end, next;
    boost::tie(ei, ei_end) = boost::edges(*graph);
    for (next=ei; ei != ei_end; ei=next) {
        next++;
        if (std::find(result.begin(), result.end(), *ei) == result.end()) {
            for (size_t i=0; i < edges.size(); i++) {
                if( edges[i].first == boost::source(*next, *graph) and edges[i].second == boost::target(*next, *graph)) {
                    edges.erase(edges.begin()+i);
                    weights.erase(weights.begin()+i);
                    break;
                }
            }
            remove_edge(*ei, *graph);
        }
    }
}

std::vector<int> DensityAwareMST::opt(const roboskel_msgs::LaserScans& ls, const unsigned nn) {
    generateTree(ls, nn);

    // TODO sort edges based on weigths instead of looking for the next biggest each time
    bool optimizing = true;
    bool thinking_of_stopping = false;
    double last_maxw = 0.0;
    unsigned max_no_improv = 20;
    unsigned num_no_improv = 0;

    std::vector<Edge> edges_to_remove;

    double best_score = 0.0;

    while (optimizing) {
        double maxw = 0;
        size_t lmi = 0;
        Graph *gr = new Graph(*graph);
        // Find biggest (in length) edge
        for (size_t i=0;i<edges.size();i++){
            if (weights[i] >= maxw){
                maxw = weights[i];
                lmi = i;
            }
        }
        std::cout << "MAXW = " << maxw << std::endl;
        std::cout << "LAST_MAXW = " << last_maxw << std::endl;
        if (thinking_of_stopping and maxw < last_maxw) {
            optimizing = false;
            continue;
        }
        // Cut the edge
        remove_edge(edges[lmi].first, edges[lmi].second, *gr);
        // And cut all the other edges that we know they generate a better score
        for (auto edge:edges_to_remove) {
            remove_edge(edge.first, edge.second, *gr);
        }
        // Find the subgraphs generated by cutting that edge
        std::vector<int> component (boost::num_vertices (*gr));
        size_t num_components = boost::connected_components(*gr, &component[0]);
        std::vector<std::vector<double>> subweights(num_components);
        std::vector<size_t> subnum_nodes(num_components);
        for (size_t i=0; i < component.size(); i++) {
            for (size_t j=0;j<edges.size();j++) {
                if (edges[j].first == i or edges[j].second == i) {
                    subweights[component[i]].push_back(weights[j]);
                }
            }
        }
        double s = 0.0;
        for (size_t i=0;i<num_components;i++) {
            s += score(subweights[i]);
        }
        std::cout << "Score = " << s << std::endl;
        std::cout << "BestScore = " << best_score << std::endl;
        std::cout << "NumComponents = " << num_components << std::endl;
        if (s >= best_score) {
            num_no_improv = s==best_score ? num_no_improv+1 : 0;
            std::cout << "num_no_improv = " << num_no_improv << std::endl;
            if (num_no_improv >= max_no_improv) {
                optimizing = false;
            }
            best_score = s;
            last_maxw = maxw;
            edges_to_remove.push_back(edges[lmi]);
            thinking_of_stopping = false;
        }
        else {
            thinking_of_stopping = true;
        }
        // Remove this edge and its weight, so we don't have to search for it again
        edges.erase(edges.begin()+lmi);
        weights.erase(weights.begin()+lmi);
    }

    for (auto edge:edges_to_remove) {
        remove_edge(edge.first, edge.second, *graph);
    }
    std::vector<int> component (boost::num_vertices (*graph));
    size_t num_components = boost::connected_components(*graph, &component[0]);

    return component;
}

double DensityAwareMST::score(const std::vector<double> w) const {
    if (w.size() > 0) {
        double tot_weight = 0.0;
        for (auto i:w) {
            tot_weight += i;
        }
        std::cout << "TOTWEIGHT = " << tot_weight << std::endl;
        return w.size() / (1 * tot_weight);
    }
    else {
        return 0.0;
    }
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

