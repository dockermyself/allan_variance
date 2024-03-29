#include "yaml_parsers.hpp"
#include <yaml-cpp/node/parse.h>
#include <iostream>
YAML::Node loadYamlFile(const std::string &filename)
{
    if (filename.empty())
    {
        throw std::invalid_argument("Filename is empty!");
    }

    YAML::Node node;

    try
    {
        node = YAML::LoadFile(filename);
    }
    catch (...)
    {
        throw std::invalid_argument("Error reading config file: " + filename);
    }

    if (node.IsNull())
    {
        throw std::invalid_argument("Error reading config file: " + filename);
    }

    std::cout << "Successfully read config file: " << filename << std::endl;

    return node;
}
