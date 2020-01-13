#include "damst/damst.hpp"

int main() {
    damst::DensityAwareMST mst;
    mst.edges.push_back(damst::DensityAwareMST::Edge(0,1));
    mst.edges.push_back(damst::DensityAwareMST::Edge(2,1));
    mst.edges.push_back(damst::DensityAwareMST::Edge(0,2));
    mst.edges.push_back(damst::DensityAwareMST::Edge(3,1));
    mst.edges.push_back(damst::DensityAwareMST::Edge(0,3));
    mst.edges.push_back(damst::DensityAwareMST::Edge(2,3));
    mst.edges.push_back(damst::DensityAwareMST::Edge(3,1));
    mst.weights.push_back(1.0);
    mst.weights.push_back(6.0);
    mst.weights.push_back(3.0);
    mst.weights.push_back(9.0);
    mst.weights.push_back(4.0);
    mst.weights.push_back(2.0);
    mst.weights.push_back(2.0);
    std::vector<damst::DensityAwareMST::EdgeDesc> result = mst.generateTree(4);

    mst.printResultTree();

    mst.createDottyGraph();

    std::cout << "First pair of the result" << std::endl;
    std::cout << mst.getResult()[0] << std::endl;

    mst.visualizeResultTree();
    return EXIT_SUCCESS;
}
