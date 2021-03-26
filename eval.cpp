// eval.cpp

#include <iostream>
#include <iomanip>

#include <yaml-cpp/yaml.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void insert_submap(
  const std::shared_ptr<octomap::OcTree>& submap,
  std::shared_ptr<octomap::OcTree>& dest_map)
{
  double probHit = submap->getProbHit();
  double probMiss = submap->getProbMiss();
  double resol = submap->getResolution();
  double minx, miny, minz;
  submap->getMetricMin(minx, miny, minz);
  double maxx, maxy, maxz;
  submap->getMetricMax(maxx, maxy, maxz);

  for (double x = minx - resol/2.0; x <= maxx + resol; x += resol)
  {
    for (double y = miny - resol/2.0; y <= maxy + resol; y += resol)
    {
      for (double z = minz - resol/2.0; z <= maxz + resol; z += resol)
      {
        octomap::point3d query = octomap::point3d(x, y, z);
        octomap::OcTreeNode* result = submap->search(query);
        if (result)
        {
          double occupancy = result->getOccupancy();
          octomap::point3d endpoint((float) x, (float) y, (float) z);
          if (occupancy >= probHit)
            dest_map->updateNode(endpoint, true);
          else if (occupancy <= probMiss)
            dest_map->updateNode(endpoint, false);
        }
      }
    }
  }
}

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
    std::cout << "opening: " << file_name << std::endl;
    submaps_ground_truth.push_back(
      std::make_shared<octomap::OcTree>(file_name));
  }

  std::map<std::string,
           std::vector< std::shared_ptr<octomap::OcTree> > > map2submaps;
  for (int i = 0; i < doc["maps"].size(); ++i)
  {
    std::cout << "======================================" << std::endl;
    std::string map_name = doc["maps"][i].as<std::string>();
    std::cout << "loading files for the map: " << map_name << std::endl;
    std::vector< std::shared_ptr<octomap::OcTree> > submaps;
    for (int j = 0; j < doc[map_name].size(); ++j)
    {
      std::string file_name = doc[map_name][j].as<std::string>();
      std::cout << "opening: " << file_name << std::endl;
      submaps.push_back(std::make_shared<octomap::OcTree>(file_name));
    }
    map2submaps[map_name] = std::move(submaps);
  }
  std::cout << "======================================" << std::endl;

  double probHit = submaps_ground_truth[0]->getProbHit();
  double probMiss = submaps_ground_truth[0]->getProbMiss();
  double resol = submaps_ground_truth[0]->getResolution();
  {
    std::shared_ptr<octomap::OcTree> map
      = std::make_shared<octomap::OcTree>(resol);

    map->setProbHit(probHit);
    map->setProbMiss(probMiss);
    map->setClampingThresMin(0.12);
    map->setClampingThresMax(0.97);

    std::cout << "combining submaps into a ground truth map" << std::flush;
    for (auto m: submaps_ground_truth)
    {
      std::cout << "." << std::flush;
      insert_submap(m, map);
    }
    std::cout << " done" << std::endl;
    std::cout << "saving the map..." << std::flush;
    map->writeBinary("ground_truth_combined.bt");
    std::cout << " done" << std::endl;
  }

  std::cout << "======================================" << std::endl;

  for (auto p: map2submaps)
  {
    std::string map_name = p.first;
    std::vector< std::shared_ptr<octomap::OcTree> > submaps = p.second;

    std::shared_ptr<octomap::OcTree> map
      = std::make_shared<octomap::OcTree>(resol);

    map->setProbHit(probHit);
    map->setProbMiss(probMiss);
    map->setClampingThresMin(0.12);
    map->setClampingThresMax(0.97);
    std::cout << "combining submaps into the map: " << map_name << std::endl;
    for (auto m: submaps)
    {
      std::cout << "." << std::flush;
      insert_submap(m, map);
    }
    std::cout << " done" << std::endl;
    std::cout << "saving the map..." << std::flush;
    map->writeBinary(map_name + "_combined.bt");
    std::cout << " done" << std::endl;
  }

  std::cout << "======================================" << std::endl;

  return 0;
}
