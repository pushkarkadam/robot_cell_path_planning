#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include "robot_cell_path_planning/read_data.h"

int main() {

    std::vector<std::string> columnNames = {"x", "y", "z"};

    std::string fileName = "/home/robot1/Documents/robot_coords.csv";

    std::map<std::string, std::vector<double>> table = readViaPoints(fileName, columnNames);

    std::vector<double> eval_point_x = table["x"];

    for (double value: table["x"]) {
        std::cout << value << "\n";
        std::cout << std::endl;
    }

    return 0;
}
