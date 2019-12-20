#include "damst/damst.hpp"

namespace damst {

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const unsigned int num_nodes){
    std::cout << &edges[0] << std::endl;
    graph = new Graph(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    return result;
}

std::vector<DensityAwareMST::EdgeDesc> DensityAwareMST::generateTree(const roboskel_msgs::LaserScans& ls){
    size_t ss = ls.scans.size();
    size_t num_nodes = 0;
    for (unsigned int i=0; i<ss; i++) {
        size_t rs = ls.scans[i].ranges.size(); 
        for (unsigned int j=0; j<rs; j++) {
            num_nodes++;
            if (j+1 < rs) {
                ls_edges.push_back(LaserScanEdge(Edge(i,j),Edge(i,j+1)));
                // TODO weight, based on polar distance
                //
            }
        }
    }

}

// Seeing dots instead of edge labels?
// sudo apt install xfonts-100dpi
// and reboot
void DensityAwareMST::createDottyGraph() const{
    std::ofstream fout("damst_result.dot");
    fout << "Graph A {\n"
    << " rankdir=LR\n"
    << " size=\"5,5\"\n"
    << " ratio=\"filled\"\n"
    << " edge[style=\"bold\"]\n" << " node[shape=\"circle\"]\n";
    boost::graph_traits<Graph>::edge_iterator eiter, eiter_end;
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

unsigned int DensityAwareMST::numberOfEdges() const{
    return edges.size();
}

void DensityAwareMST::printResultTree(){
    boost::property_map < Graph, boost::edge_weight_t >::type weight = boost::get(boost::edge_weight, *graph);
    for (std::vector < damst::DensityAwareMST::EdgeDesc >::iterator ei = result.begin(); ei != result.end(); ei++){
        std::cout << boost::source(*ei, *graph) << " 🠜🠞 " << boost::target(*ei, *graph) << " with weight: " << weight[*ei] << std::endl;
    }
}

const std::vector<DensityAwareMST::EdgeDesc>& DensityAwareMST::getResult() const{
    return result;
}

void DensityAwareMST::visualizeReultTree() const{
    DensityAwareMST::createDottyGraph();
    std::system("dotty damst_result.dot &");
}

}

