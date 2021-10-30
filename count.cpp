// count.cpp
//
// This code checks the size of the map on the actual memory.

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// =============================================================================
void printSize(const std::shared_ptr<octomap::OcTree>& map)
{
  std::cout << " memory usage: "
            << std::setw(10)
            << map->memoryUsage()
            << " bytes, "
            << std::setprecision(2) << std::fixed
            << map->memoryUsage() / 1024.0
            << " KiB, "
            << map->memoryUsage() / 1024.0 / 1024.0
            << " MiB" << std::endl;
}

// =============================================================================
int main(int argc, char** argv )
{
  std::ifstream fin("settings.yaml");
  YAML::Node doc = YAML::Load(fin);

  std::vector< std::shared_ptr<octomap::OcTree> > submaps_ground_truth;
  std::cout << "======================================" << std::endl;
  std::cout << "loading files for the ground_truth:" << std::endl;
  for (int i = 0; i < doc["ground_truth"].size(); ++i)
  {
    std::string file_name = doc["ground_truth"][i].as<std::string>();
    std::cout << file_name << std::endl;
    auto map = std::make_shared<octomap::OcTree>(file_name);
    printSize(map);
  }

  for (int i = 0; i < doc["maps"].size(); ++i)
  {
    std::cout << "======================================" << std::endl;
    std::string map_name = doc["maps"][i].as<std::string>();
    std::cout << "loading files for the map: " << map_name << std::endl;
    std::vector< std::shared_ptr<octomap::OcTree> > submaps;
    for (int j = 0; j < doc[map_name].size(); ++j)
    {
      std::string file_name = doc[map_name][j].as<std::string>();
      std::cout << file_name << std::endl;
      auto map = std::make_shared<octomap::OcTree>(file_name);
      printSize(map);
    }
  }
  return 0;
}
