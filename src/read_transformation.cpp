#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

int main() {
    std::cout << "Reading Transformation." << std::endl;

    YAML::Node config = YAML::LoadFile("/home/robot1/transform2.yaml");

    if (!config["transform"]) {
        std::cerr << "Error: 'transform; key not found in YAML file." << std::endl;
        return 1;
    }

    Eigen::Matrix4d T;
    auto mat = config["transform"];

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T(i, j) = mat[i][j].as<double>();
        }
    }

    std::cout << "Loaded Transformation Matrix:\n" << T << std::endl;

    return 0;
}