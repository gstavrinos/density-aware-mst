#include "damst/damst.hpp"

namespace damst {

int partition(std::vector<DensityAwareMST::Edge>& edges, std::vector<double>& weights, size_t start, size_t end) {
    double pivot = weights[end];

    size_t pivi = start;
    DensityAwareMST::Edge e;
    double w;
    for(size_t i=start; i<end; i++) {
        if(weights[i] <= pivot) {
            w = weights[i];
            e = edges[i];
            weights[i] = weights[pivi];
            edges[i] = edges[pivi];
            weights[pivi] = w;
            edges[pivi] = e;
            pivi++;
        }
    }

    w = weights[end];
    e = edges[end];
    weights[end] = weights[pivi];
    edges[end] = edges[pivi];
    weights[pivi] = w;
    edges[pivi] = e;

    return pivi;
}

int partition(std::vector<double>& weights, int start, int end) {
    double pivot = weights[end];

    size_t pivi = start;
    double w;
    for(size_t i=start; i<end; i++) {
        if(weights[i] <= pivot) {
            w = weights[i];
            weights[i] = weights[pivi];
            weights[pivi] = w;
            pivi++;
        }
    }

    w = weights[end];
    weights[end] = weights[pivi];
    weights[pivi] = w;

    return pivi;
}

void quicksort(std::vector<DensityAwareMST::Edge>& edges, std::vector<double>& weights, int start, int end) {
    if(start<end) {
        int pivi = partition(edges, weights, start, end);
        quicksort(edges, weights, start, pivi-1);
        quicksort(edges, weights, pivi+1, end);
    }
}

void quicksort(std::vector<double>& weights, int start, int end) {
    if(start<end) {
        size_t pivi = partition(weights, start, end);
        quicksort(weights, start, pivi-1);
        quicksort(weights, pivi+1, end);
    }
}

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
    vertices = std::vector<std::pair<double,double>>(points);
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
        double s_entropy = 0;
        double t_entropy = 0;
        int s = boost::source(result[i], *graph);
        int t = boost::target(result[i], *graph);
        for (auto p:points) {
            double p1 = pow(dist(p, points[s]),1);
            double p2 = pow(dist(p, points[t]),1);
            if (p1 > 0) {
                s_entropy += 1/p1;
            }
            if (p2 > 0) {
                t_entropy += 1/p2;
            }
        }
        edges.push_back(Edge(s, t));
        // TODO optimize this by accessing it through the graph instead of calculating it again
        // weights.push_back(dist(points[s], points[t]));
        // weights.push_back(pow(dist(points[s],points[t]),2)*abs(s_entropy - t_entropy));
        weights.push_back(pow(dist(points[s],points[t]),2)*pow(s_entropy - t_entropy,2));
        // weights.push_back(dist(points[s],points[t])*pow(s_entropy - t_entropy,2));
    }
    graph = std::make_shared<Graph>(&edges[0], &edges[0]+numberOfEdges(), &weights[0], points.size());
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
        // std::cout <<"maxw:";
        // std::cout << maxw << std::endl;

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
        // std::cout << "Score = " << s << std::endl;
        // std::cout << "BestScore = " << best_score << std::endl;
        // std::cout << "NumComponents = " << num_components << std::endl;
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

    // std::cout << msg.num_clusters << std::endl;

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

std::pair<std::vector<int>, int> DensityAwareMST::opt(const std::vector<std::pair<double, double>> points, const bool debug) {

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
    double minw = 100;
    double maxw = 0;

    double meanw = 0;
    for (size_t i=0;i<weights.size();i++){
        meanw += weights[i];
        minw = minw>weights[i] ? weights[i] : minw;
        maxw = maxw<weights[i] ? weights[i] : maxw;
    }
    meanw /= weights.size();
    std::cout << "min=" << minw << std::endl;
    std::cout << "max=" << maxw << std::endl;
    std::cout << "mean=" << meanw << std::endl;
    meanw += 0.975*meanw;
    std::cout << "mean=" << meanw << std::endl;
    std::cout << "median=" << weights[int(weights.size()/2)] << std::endl;
    meanw = weights[int(weights.size()/2)];

    for (size_t i=0;i<edges.size();i++){
        if (weights[i] > meanw){
            edges_to_remove.push_back(edges[i]);
        }
    }


    // while (optimizing or cnt < 3) {
        // cnt++;
        // double maxw = 0;
        // size_t lmi = 0;
        // std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
        // // Find biggest (in length) edge
        // for (size_t i=0;i<edges.size();i++){
            // if (weights[i] >= maxw){
                // maxw = weights[i];
                // lmi = i;
            // }
        // }
        // // std::cout <<"maxw:";
        // // std::cout << maxw << std::endl;

        // if (thinking_of_stopping and maxw < last_maxw) {
            // optimizing = false;
            // continue;
        // <
        // // And cut all the other edges that we know they generate a better score
        // for (auto edge:edges_to_remove) {
            // remove_edge(edge.first, edge.second, *gr);
        // }
        // // Cut the edge
        // remove_edge(edges[lmi].first, edges[lmi].second, *gr);
        // // Find the subgraphs generated by cutting that edge
        // std::vector<int> component (boost::num_vertices (*gr));
        // size_t num_components = boost::connected_components(*gr, &component[0]);
        // std::vector<std::vector<double>> subweights(num_components);
        // std::vector<size_t> subnum_nodes(num_components);
        // for (size_t i=0; i < component.size(); i++) {
            // for (size_t j=0;j<edges.size();j++) {
                // if (edges[j].first == i or edges[j].second == i) {
                    // subweights[component[i]].push_back(weights[j]);
                // }
            // }
        // }
        // double s = 0.0;
        // for (size_t i=0;i<num_components;i++) {
            // s += score(subweights[i]);
        // }
        // // std::cout << "Score = " << s << std::endl;
        // // std::cout << "BestScore = " << best_score << std::endl;
        // // std::cout << "NumComponents = " << num_components << std::endl;
        // if (s >= best_score) {
            // num_no_improv = s==best_score ? num_no_improv+1 : 0;
            // if (num_no_improv >= max_no_improv) {
                // optimizing = false;
            // }
            // best_score = s;
            // last_maxw = maxw;
            // edges_to_remove.push_back(edges[lmi]);
            // thinking_of_stopping = false;
        // }
        // else {
            // thinking_of_stopping = true;
        // }
        // // Remove this edge and its weight, so we don't have to search for it again
        // edges.erase(edges.begin()+lmi);
        // weights.erase(weights.begin()+lmi);
    // }

    std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
    if (debug) {
        dbgSave(edges_to_remove);
    }

    for (auto edge:edges_to_remove) {
        remove_edge(edge.first, edge.second, *gr);
    }

    std::vector<int> component (boost::num_vertices (*gr));
    int num_clusters = boost::connected_components(*gr, &component[0]);

    return std::pair<std::vector<int>, int>(component, num_clusters);
}

std::pair<std::vector<int>, int> DensityAwareMST::opt2(const std::vector<std::pair<double, double>> points, const bool debug) {

    std::cout << "Generating tree..." << std::endl;
    generateTree(points);
    std::cout << "Sorting..." << std::endl;
    quicksort(edges, weights, 0, weights.size()-1);
    std::cout << "Optimizing..." << std::endl;

    std::vector<bool> edges_to_remove(edges.size());
    for (size_t i=0; i<edges.size(); i++) {
        edges_to_remove[i] = false;
    }

    double best_score = 0.0;

    mean = 0;

    for (auto w:weights) {
        mean += w;
    }

    mean /= weights.size();

    double sigma = 0;
    double sum = 0;

    for (auto w:weights) {
        sum += pow(w - mean,2);
    }

    sigma = sqrt(sum/weights.size());

    double threshold = mean + sigma;

    std::cout << "Mean, threshold, sigma"<< std::endl;
    std::cout << mean << std::endl;
    std::cout << threshold << std::endl;
    std::cout << sigma << std::endl;

    for (int i=edges.size()-1; i>=edges.size()-300; i--) {
    // for (int i=edges.size()-1; i>=0; i--) {
    // for (int i=0; i<=30; i++) {
        std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
        for (int j=edges.size()-1; j > i; j--) {
            if (edges_to_remove[j]) {
                remove_edge(edges[j].first, edges[j].second, *gr);
            }
        }
        remove_edge(edges[i].first, edges[i].second, *gr);

        std::vector<int> component (boost::num_vertices (*gr));
        size_t num_components = boost::connected_components(*gr, &component[0]);
        std::vector<std::vector<double>> subweights(num_components);
        std::vector<std::vector<Edge>> subedges(num_components);
        // std::vector<std::vector<std::pair<double,double>>> subvertices(num_components);
        for (size_t k=0; k < component.size(); k++) {
            for (int j=edges.size()-1; j >= 0; j--) {
                if (j >=i and (edges_to_remove[j] or j==i)) {
                    continue;
                }
                if (edges[j].first == k or edges[j].second == k) {
                    subweights[component[k]].push_back(weights[j]);
                    subedges[component[k]].push_back(edges[j]);
                    // subvertices[component[k]].push_back(points[component[k]]);
                }
            }
        }
        double s = 0.0;
        for (size_t k=0; k<num_components; k++) {
            // s += score(subvertices[k], subedges[k], subweights[k], *gr); //score(subedges[k], subweights[k], *gr);//score(subweights[k]);
            s += score(subweights[k]);
        }
        // s /=num_components;
        if (s > best_score) {
            std::cout << best_score << std::endl;
        // if (s < sigma) {
        // if (weights[i] >= threshold) {
            best_score = s;
            edges_to_remove[i] = true;
        }
        if (stopt()) {
            break;
        }
    }

    if (debug) {
        dbgSave(edges_to_remove);
    }

    std::shared_ptr<Graph> gr = std::make_shared<Graph>(*graph);
    for (size_t i=0; i<edges.size(); i++) {
        if (edges_to_remove[i]){
            remove_edge(edges[i].first, edges[i].second, *gr);
        }
    }

    std::vector<int> component (boost::num_vertices (*gr));
    int num_clusters = boost::connected_components(*gr, &component[0]);

    return std::pair<std::vector<int>, int>(component, num_clusters);
}

void DensityAwareMST::dbgSave(const std::vector<Edge> edges_to_remove) {
    std::cout << "Writing tree to file..." << std::endl;
    std::ofstream of;
    of.open("/home/gstavrinos/damst_full_tree.txt");
    EdgeIter eiter, eiter_end;
    for (boost::tie(eiter, eiter_end) = boost::edges(*graph); eiter != eiter_end; eiter++) {

    of << boost::source(*eiter, *graph) << " " << boost::target(*eiter, *graph) << std::endl;
    }
    of.close();
    of.open("/home/gstavrinos/damst_removed_edges.txt");
    for (auto edge:edges_to_remove) {
        of << edge.first << " " << edge.second << std::endl;
    }
    of.close();
}

void DensityAwareMST::dbgSave(const std::vector<bool> edges_to_remove) {
    std::cout << "Writing tree to file..." << std::endl;
    std::ofstream of;
    of.open("/home/gstavrinos/damst_full_tree.txt");
    EdgeIter eiter, eiter_end;
    for (boost::tie(eiter, eiter_end) = boost::edges(*graph); eiter != eiter_end; eiter++) {

    of << boost::source(*eiter, *graph) << " " << boost::target(*eiter, *graph) << std::endl;
    }
    of.close();
    of.open("/home/gstavrinos/damst_removed_edges.txt");
    for (size_t i=0; i<edges.size(); i++) {
        if (edges_to_remove[i]){
            of << edges[i].first << " " << edges[i].second << std::endl;
        }
    }
    of.close();
}

bool DensityAwareMST::stopt() {
    return false;
}

double DensityAwareMST::score(const std::vector<std::pair<double,double>> v, const std::vector<Edge> e, const std::vector<double> w, const Graph g) const {
    double scr = 0;
    double minx = 10000000;
    double miny = 10000000;
    double maxx = -10000000;
    double maxy = -10000000;

    for (auto v_:v) {
        minx = v_.first < minx ? v_.first : minx;
        miny = v_.second < miny ? v_.second : miny;
        maxx = v_.first > maxx ? v_.first : maxx;
        maxy = v_.second > maxy ? v_.second : maxy;
    }

    // for (int i=0; i < e.size(); i++) {
        // scr += w[i] / ((int(degree(source(e[i],g),g)) + int(degree(target(e[i],g),g)))/e.size());
    // }
    // if (scr > 0) {
        // scr = pow(e.size()+1, 1)/scr;
    // }
    return scr;
}

double DensityAwareMST::score(const std::vector<Edge> e, const std::vector<double> w, const Graph g) const {
    double tot_weight = 0;
    for (int i=0; i < e.size(); i++) {
        tot_weight += w[i] / ((int(degree(source(e[i],g),g)) + int(degree(target(e[i],g),g)))/e.size());
    }
    if (tot_weight > 0) {
        tot_weight = pow(e.size()+1, 1)/tot_weight;
    }
    return tot_weight;
}

double DensityAwareMST::score(const std::vector<double> w) const {

    double scr = 0;

    for (auto i:w) {
        scr += i;
    }

    scr = (w.size()-1);///pow(scr,2);

    // double scr = 0;
    // double sum = 0;

    // for (auto i:w) {
        // sum += pow(i - mean,2);
    // }

    // double sigma = sqrt(sum/weights.size());
    // scr = sigma;
    // scr = (w.size()+1) / scr;
    // if (w.size() > 0) {
        // std::vector<double> sw(w);
        // quicksort(sw, 0, sw.size()-1);

        // double medw = sw[sw.size()/2];

        // int node_weight = 0;
        // double tot_weight = 0;
        // for (auto i:w) {
            // tot_weight += i;
            // if (i <= sw[sw.size()-1]-medw) {
                // scr += i;
                // node_weight += 1;
            // }
            // else {
                // scr -= i;
            // }
        // }
        // return tot_weight/node_weight;
        // // return node_weight * scr / ((w.size()+1-node_weight));
        // // return node_weight*(w.size()+1) / ((w.size()+1-node_weight));
    // }
    return scr;
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
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
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

