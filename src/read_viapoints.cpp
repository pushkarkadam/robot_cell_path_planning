#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

std::map<std::string, std::vector<double>> readViaPoints(std::string fileName, std::vector<std::string> columnNames);

int main() {

    std::vector<std::string> columnNames = {"x", "y", "z"};

    std::map<std::string, std::vector<double>> table = readViaPoints("/home/robot1/Documents/robot_coords.csv", columnNames);

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

std::map<std::string, std::vector<double>> readViaPoints(std::string fileName, std::vector<std::string> columnNames) {
    // Reading csv file
    std::ifstream file(fileName);

    std::string line;

    std::getline(file, line);
    std::stringstream headerStream(line);

    // Map of column to list of values 
    std::map<std::string, std::vector<double>> table;

    for (const std::string& name : columnNames) {
        table[name] = std::vector<double>();
    }

    // Read the rest of the rows 
    while (std::getline(file, line)) {
        std::stringstream row(line);
        std::string cell;
        int i = 0;

        while (std::getline(row, cell, ',')) {
            table[columnNames[i]].push_back(std::stod(cell));
            i++;
        }
    }

    return table;
}