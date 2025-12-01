#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

int main() {
    std::cout << "Hello YAML!" << std::endl;

    YAML::Node config = YAML::LoadFile("/home/robot1/hello.yaml");

    std::string app_name = config["name"].as<std::string>();

    std::cout << "Read Value: " << app_name << std::endl;

    return 0;
}