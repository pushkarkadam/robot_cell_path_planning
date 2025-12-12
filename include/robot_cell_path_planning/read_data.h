#ifndef READ_DATA_HPP
#define READ_DATA_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

std::map<std::string, std::vector<double>> readViaPoints(std::string fileName, std::vector<std::string> columnNames);

#endif