#include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <set>
#include "AllanVarianceComputor.hpp"

int main(int argc, char **argv)
{
    std::string bags_folder = ".";
    std::string config_file;

    if (argc >= 2)
    {
        bags_folder = argv[1];
        config_file = argv[2];
        std::cout << "Bag Folder = " << bags_folder << std::endl;
        std::cout << "Config File = " << config_file << std::endl;
    }
    else
    {
        std::cout << "folder and/or config file not provided!" << std::endl;
    }

    namespace fs = boost::filesystem;
    fs::path path = fs::absolute(fs::path(bags_folder));

    std::set<std::string> bag_filenames_sorted;
    for (const auto &entry : fs::directory_iterator(path))
    {
        if (entry.path().extension() == ".bag")
        {
            bag_filenames_sorted.insert(entry.path().string());
        }
    }
    std::cout << "Bag filenames count: " << bag_filenames_sorted.size() << std::endl;

    std::clock_t start = std::clock();

    allan_variance::AllanVarianceComputor computor(config_file, bags_folder);
    std::cout << "Batch computor constructed" << std::endl;
    for (const auto &it : bag_filenames_sorted)
    {
        std::cout << "Processing rosbag " << it << std::endl;
        computor.run(it);
    }

    double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    printf("Total computation time: %f s", durationTime);
    printf("Data written to allan_variance.csv");
    return 0;
}
