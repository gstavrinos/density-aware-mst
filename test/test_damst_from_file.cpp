#include <chrono>
#include "damst/damst.hpp"
#include "matplotlibcpp.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "You need to provide input and output files (absolute paths and in this order)." << std::endl;
        return 1;
    }
    std::string line;
    std::ifstream f(argv[1]);
    std::vector<std::pair<double, double>> points;
    std::cout << "Constructing points vector from file..." << std::endl;
    if (f.is_open()) {
        while (std::getline(f, line)) {
            std::string c;
            std::istringstream line_(line);
            std::pair<double, double> point;
            std::getline(line_, c, '\t');
            point.first = std::atof(c.c_str());
            std::getline(line_, c, '\t');
            point.second = std::atof(c.c_str());
            points.push_back(point);
        }
    }

    damst::DensityAwareMST mst;

    std::pair<std::vector<int>, int> result;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    result = mst.opt(points);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Overall time = " << std::chrono::duration_cast<std::chrono::seconds> (end - begin).count() << " seconds." << std::endl;

    const std::vector<int> component = result.first;
    const int num_clusters = result.second;

    const size_t s = component.size();

    std::vector<double> x,y;
    std::vector<std::string> c;

    std::cout << "Saving results..." << std::endl;
    std::ofstream of;
    of.open(argv[2]);
    // First line is the number of clusters
    of << num_clusters << std::endl;
    std::string colours[8] = {"b.", "g.", "r.", "c.", "m.", "y.", "k.", "w."};
    for (size_t i=0;i<s;i++) {
        of << points[i].first << "," << points[i].second << "," << component[i] << std::endl;
        x.push_back(points[i].first);
        y.push_back(points[i].second);
        c.push_back(colours[component[i] % 8]);
        // Very hacky way to visualize the clusters
        // but scatter does not allow
        // for point colours in cpp
        // (as fas as I can tell)
        matplotlibcpp::plot(x,y,c[0]);
        x.clear();
        y.clear();
        c.clear();
    }
    of.close();

    std::cout << "Visualizing results..." << std::endl;
    // matplotlibcpp::scatter(x,y,1);
    matplotlibcpp::show();

    return 0;
}
