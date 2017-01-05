
#include <linkbot/linkbot.hpp>

#include <iostream>

int main(int argc, char **argv) {
    std::string serial_id;
    std::cout << "Enter robot serial id: ";
    std::cin >> serial_id;

    barobo::CLinkbot linkbot {serial_id};
    std::vector<double> times[3];
    std::vector<double> angles[3];
    linkbot.recordAnglesBegin(
        times[0],
        angles[0],
        times[1],
        angles[1],
        times[2],
        angles[2]);
    linkbot.move(90, 90, 90);
    linkbot.recordAnglesEnd();
    barobo::scatterPlot(times[0], angles[0]);

    return 0;
}
