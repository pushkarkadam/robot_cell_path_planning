#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include "robot_cell_path_planning/read_data.h"

// std::map<std::string, std::vector<double>> readViaPoints(std::string fileName, std::vector<std::string> columnNames);

int main() {

    std::vector<std::string> columnNames = {"x", "y", "z"};

    std::string fileName = "/home/robot1/Documents/robot_coords.csv";

    std::map<std::string, std::vector<double>> table = readViaPoints(fileName, columnNames);

    // Print collected data
    for (const std::string& name : columnNames) {
        std::cout << name << ": ";

        for (double value : table[name]) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
