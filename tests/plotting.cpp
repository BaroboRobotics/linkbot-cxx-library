
#include <linkbot/linkbot.hpp>

#include <iostream>

int main(int argc, char **argv) {
    std::string serial_id;
    std::cout << "Enter robot serial id: ";
    std::cin >> serial_id;

    barobo::CLinkbot linkbot {serial_id};
    std::vector<double> times[3];
    std::vector<double> angles[3];
    linkbot.recordAnglesBegin();
    linkbot.move(90, 90, 90);
    auto data = linkbot.recordAnglesEnd();
    barobo::scatterPlot(data);
    return 0;
}
