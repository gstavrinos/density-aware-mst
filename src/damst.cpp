#include "damst/damst.hpp"

namespace damst {

size_t DensityAwareMST::generateTree(const roboskel_msgs::LaserScans& ls, const unsigned nn) {
    const size_t ss = ls.scans.size();
    num_nodes = 0;
    for (unsigned i=0; i<ss; i++) {
        const size_t rs = ls.scans[i].ranges.size(); 
        for (unsigned j=0; j<rs; j++) {
            if (std::isfinite(ls.scans[i].ranges[j])) {
                for (size_t k=j+1;k<rs;k++) {
                    if (std::isfinite(ls.scans[i].ranges[k])) {
                        edges.push_back(Edge(i*rs+j, i*rs+k));
                        weights.push_back(dist(ls, i, j, i, k));
                    }
                }
            }
            else{
            std::cout << ls.scans[i].ranges[j];
            std::cin >> num_nodes;
            }
            num_nodes++;

            if (i+1 < ss) {
                for (unsigned n=j-nn; n<=j+nn; n++) {
                    // I am checking with the current
                    // laserscan and not with the next one
                    // to avoid a call to .size(). I should
                    // probably be fine, since all laserscans
                    // are generated from the same driver.
                    // BUT: (TODO) If I ever encounter crashes,
                    // this is the place to look for first.
                    if (n >= 0 and n < rs and std::isfinite(ls.scans[i+1].ranges[n])) {
                        edges.push_back(Edge(i*rs+j, (i+1)*rs+n));
                        weights.push_back(dist(ls, i, j, i+1, n));
                    }
                }
            }
        }
    }

    graph = std::make_shared<Graph>(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    updateGraphBasedOnResult(ls);
    return ss;
}

void DensityAwareMST::generateTree(const std::vector<std::pair<double,double>> points) {
    edges.clear();
    weights.clear();
    for (size_t i=0; i<points.size(); i++) {
        for (size_t j=i+1; j<points.size(); j++) {
            edges.push_back(Edge(i,j));
            weights.push_back(dist(points[i], points[j]));
        }
    }
    graph = std::make_shared<Graph>(&edges[0], &edges[0]+numberOfEdges(), &weights[0], points.size());
    boost::kruskal_minimum_spanning_tree(*graph, std::back_inserter(result));
    updateGraphBasedOnResult(points);
}

void DensityAwareMST::updateGraphBasedOnResult(const std::vector<std::pair<double, double>> points) {
    edges.clear();
    weights.clear();
    for (size_t i=0;i<result.size();i++) {
        int s = boost::source(result[i], *graph);
        int t = boost::target(result[i], *graph);
        edges.push_back(Edge(s, t));
        // TODO optimize this by accessing it through the graph instead of calculating it again
        weights.push_back(dist(points[s], points[t]));
    }
    graph = std::make_shared<Graph>(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
}

void DensityAwareMST::updateGraphBasedOnResult(const roboskel_msgs::LaserScans& ls) {
    edges.clear();
    weights.clear();
    for (size_t i=0;i<result.size();i++) {
        int s = boost::source(result[i], *graph);
        int t = boost::target(result[i], *graph);
        edges.push_back(Edge(s, t));
        int laserscan_s = s/ls.scans[0].ranges.size();
        int laserscan_t = t/ls.scans[0].ranges.size();
        int i_s = s%ls.scans[0].ranges.size();
        int i_t = t%ls.scans[0].ranges.size();
        std::cout << boost::source(result[i], *graph) << std::endl;
        std::cout << boost::target(result[i], *graph) << std::endl;
        std::cout << laserscan_s << std::endl;
        std::cout << laserscan_t << std::endl;
        std::cout << i_s << std::endl;
        std::cout << i_t << std::endl;
        std::cout << "-----" << std::endl;
        // TODO optimize this by accessing it through the graph instead of calculating it again
        weights.push_back(dist(ls, laserscan_s, i_s, laserscan_t, i_t));
    }
    graph = std::make_shared<Graph>(&edges[0], &edges[0]+numberOfEdges(), &weights[0], num_nodes);
}

roboskel_msgs::ClusteredLaserScans DensityAwareMST::opt(const roboskel_msgs::LaserScans& ls, const unsigned n) {
    generateTree(ls, n);

    // TODO sort edges based on weigths instead of looking for the next biggest each time
    bool optimizing = true;
    bool thinking_of_stopping = false;
    double last_maxw = 0.0;
    unsigned max_no_improv = 20;
    unsigned num_no_improv = 0;

    std::vector<Edge> edges_to_remove;

    double best_score = 0.0;

    int cnt = 0;

    while (optimizing or cnt < 3) {
        cnt++;
        double maxw = 0;
        size_t lmi = 0;
        std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
        // Find biggest (in length) edge
        for (size_t i=0;i<edges.size();i++){
            if (weights[i] >= maxw){
                maxw = weights[i];
                lmi = i;
            }
        }
        std::cout <<"maxw:";
        std::cout << maxw << std::endl;

        if (thinking_of_stopping and maxw < last_maxw) {
            optimizing = false;
            continue;
        }
        // And cut all the other edges that we know they generate a better score
        for (auto edge:edges_to_remove) {
            remove_edge(edge.first, edge.second, *gr);
        }
        // Cut the edge
        remove_edge(edges[lmi].first, edges[lmi].second, *gr);
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

    std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
    for (auto edge:edges_to_remove) {
        remove_edge(edge.first, edge.second, *gr);
    }

    // Custom message to store each laserscan's points equivalent cluster
    // For example, msg.cluster_map[0][1] stores the cluster 
    // of the second point of the first laserscan.
    roboskel_msgs::ClusteredLaserScans msg;
    msg.header= ls.header;
    msg.scans = ls.scans;

    std::vector<int> component (boost::num_vertices (*gr));
    msg.num_clusters = boost::connected_components(*gr, &component[0]);

    std::cout << msg.num_clusters << std::endl;

    std::vector<unsigned> tmp;
    const size_t s = component.size();

    const size_t all_points = ls.scans.size()*ls.scans[0].ranges.size();
    // break_point is the size of the laserscans
    // assuming that they all have the same size, as they should
    const size_t break_point = s/ls.scans.size();

    for (size_t i=0;i<s;i++) {
        tmp.push_back(component[i]);
        if ((i+1) % break_point == 0) {
            roboskel_msgs::LaserScanCluster tmpmsg;
            tmpmsg.cluster = tmp;
            msg.cluster_map.push_back(tmpmsg);
            tmp.clear();
        }
    }
    return msg;
}

std::pair<std::vector<int>, int> DensityAwareMST::opt(const std::vector<std::pair<double, double>> points) {

    std::cout << "Generating tree..." << std::endl;
    generateTree(points);
    std::cout << "Optimizing..." << std::endl;
    // TODO sort edges based on weigths instead of looking for the next biggest each time
    bool optimizing = true;
    bool thinking_of_stopping = false;
    double last_maxw = 0.0;
    unsigned max_no_improv = 20;
    unsigned num_no_improv = 0;

    std::vector<Edge> edges_to_remove;

    double best_score = 0.0;

    int cnt = 0;

    while (optimizing or cnt < 3) {
        cnt++;
        double maxw = 0;
        size_t lmi = 0;
        std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
        // Find biggest (in length) edge
        for (size_t i=0;i<edges.size();i++){
            if (weights[i] >= maxw){
                maxw = weights[i];
                lmi = i;
            }
        }
        std::cout <<"maxw:";
        std::cout << maxw << std::endl;

        if (thinking_of_stopping and maxw < last_maxw) {
            optimizing = false;
            continue;
        }
        // And cut all the other edges that we know they generate a better score
        for (auto edge:edges_to_remove) {
            remove_edge(edge.first, edge.second, *gr);
        }
        // Cut the edge
        remove_edge(edges[lmi].first, edges[lmi].second, *gr);
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

    std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
    for (auto edge:edges_to_remove) {
        remove_edge(edge.first, edge.second, *gr);
    }

    std::vector<int> component (boost::num_vertices (*gr));
    int num_clusters = boost::connected_components(*gr, &component[0]);

    return std::pair<std::vector<int>, int>(component, num_clusters);
}

double DensityAwareMST::score(const std::vector<double> w) const {
    double tot_weight = 0.0;
    for (auto i:w) {
        tot_weight += i;
    }
    std::cout << "TOTWEIGHT = " << tot_weight << std::endl;
    if (tot_weight > 0) { 
        return pow(w.size(),2) / (1 * tot_weight);
    }
    else {
        return 0.0;
    }
}

double DensityAwareMST::dist(const roboskel_msgs::LaserScans& ls, const size_t i1, const size_t j1, const size_t i2, const size_t j2) const {
    double r1 = ls.scans[i1].ranges[j1];
    double r2 = ls.scans[i2].ranges[j2];
    if (std::isfinite(r1) and std::isfinite(r2)) {
        double theta1 = ls.scans[i1].angle_min + j1 * ls.scans[i1].angle_increment;
        double theta2 = ls.scans[i2].angle_min + j2 * ls.scans[i2].angle_increment;
        return sqrt(r1*r1 + r2*r2 - 2*r1*r2*(cos(theta1-theta2)));
    }
    else {
        return 0.01;
    }
}

double DensityAwareMST::dist(const std::pair<double, double> p1, const std::pair<double, double> p2) const {
    return sqrt(pow(p1.first - p2.first, 2) + pow(p2.second - p2.second, 2));
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

void DensityAwareMST::visualizeResultTree() const {
    DensityAwareMST::createDottyGraph();
    std::system("dotty damst_result.dot &");
}

}

