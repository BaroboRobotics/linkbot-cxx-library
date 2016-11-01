
#include <linkbot/linkbot.hpp>

#include <iostream>

int main(int argc, char **argv) {
    std::vector<double> xs = {1,2,3};
    std::vector<double> ys = {4,5,6};
    barobo::scatterPlot(xs, ys);

    return 0;
}
