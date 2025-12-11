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

/**
* @brief Reads a CSV file of via points.
* 
* This function takes the csv file name and a vector of column names
* and stores a Map for each of the column as a vector.
* 
* @param fileName The name of the CSV file.
* @param columnNames An array of strings of the column names of the csv file.
* @return A Map of column names to an array of double.
*/
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