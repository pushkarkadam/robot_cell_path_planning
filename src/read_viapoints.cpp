#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

int main() {
    ifstream file("/home/robot1/Documents/robot_coords.csv");

    string line;

    while (getline(file, line)) {
        cout << line << endl;
    }

    return 0;
}