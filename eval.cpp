// eval.cpp

#include <iostream>
#include <iomanip>

#include <yaml-cpp/yaml.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node)
{
  if (node != NULL)
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  else
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

int main(int argc, char** argv )
{
  std::ifstream fin("settings.yaml");
  YAML::Node doc = YAML::Load(fin);

  std::cout << "ground_truth: " << doc["ground_truth"] << std::endl;
  for (int i = 0; i < doc["ground_truth"].size(); ++i)
    std::cout << doc["ground_truth"][i] << std::endl;
  std::cout << "maps: " << doc["maps"] << std::endl;
  for (int i = 0; i < doc["maps"].size(); ++i)
    std::cout << doc["maps"][i] << std::endl;
  std::cout << "default: " << doc["default"] << std::endl;
  for (int i = 0; i < doc["default"].size(); ++i)
    std::cout << doc["default"][i] << std::endl;
  std::cout << "mymethod: " << doc["mymethod"] << std::endl;
  for (int i = 0; i < doc["mymethod"].size(); ++i)
    std::cout << doc["mymethod"][i] << std::endl;

  octomap::OcTree *map;

  map = new octomap::OcTree(0.1);
  map->setProbHit(0.7);
  map->setProbMiss(0.4);
  map->setClampingThresMin(0.12);
  map->setClampingThresMax(0.97);

  double offset_x = 1.0;
  double offset_y = 1.0;
  double offset_z = 1.0;

  for (double x = -1.0 + offset_x; x < 1.0 + offset_x; x += 0.05f)
  {
    for (double y = -1.0 + offset_y; y < 1.0 + offset_y; y += 0.05f)
    {
      for (double z = -1.0 + offset_z; z < 1.0 + offset_z; z += 0.05f)
      {
        octomap::point3d endpoint ((float) x, (float) y, (float) z);
        map->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }
  for (double x = -1.58 + offset_x; x <= -0.4 + offset_x; x += 0.02f)
  {
    for (double y = -1.58 + offset_y; y <= -0.4 + offset_y; y += 0.02f)
    {
      for (double z = -1.58 + offset_z; z <= -0.4 + offset_z; z += 0.02f)
      {
        octomap::point3d endpoint ((float) x, (float) y, (float) z);
        map->updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  std::cout << std::endl;
  std::cout << "performing some queries:" << std::endl;

  octomap::point3d query (0., 0., 0.);
  octomap::OcTreeNode* result = map->search (query);
  print_query_info(query, result);

  query = octomap::point3d(-1.,-1.,-1.);
  result = map->search (query);
  print_query_info(query, result);

  query = octomap::point3d(1.,1.,1.);
  result = map->search (query);
  print_query_info(query, result);

  std::cout << "=== scan (fixed) ===" << std::endl;
  for (double x = -1.7; x <= 1.7; x += 0.1)
  {
    for (double y = -1.7; y <= 1.7; y += 0.1)
    {
      query = octomap::point3d(x, y, -0.5);
      result = map->search (query);
      //print_query_info(query, result);
      if (result)
      {
        std::cout << std::setw(5) << std::setprecision(2) << std::fixed
                  << result->getOccupancy();
      }
      else
      {
        std::cout << " XXXX";
      }
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  double x, y, z;
  map->getMetricSize(x, y, z);
  std::cout << "size: x = " << x << ", y = " << y << ", z = " << z
            << std::endl;
  double minx, miny, minz;
  map->getMetricMin(minx, miny, minz);
  std::cout << "min: x = " << minx << ", y = " << miny << ", z = " << minz
            << std::endl;
  double maxx, maxy, maxz;
  map->getMetricMax(maxx, maxy, maxz);
  std::cout << "max: x = " << maxx << ", y = " << maxy << ", z = " << maxz
            << std::endl;

  std::cout << "=== scan (flexible) ===" << std::endl;
  for (double x = minx; x <= maxx + 0.1; x += 0.1)
  {
    for (double y = miny; y <= maxx + 0.1; y += 0.1)
    {
      query = octomap::point3d(x, y, (maxz - minz)*0.4 + minz);
      result = map->search (query);
      //print_query_info(query, result);
      if (result)
      {
        std::cout << std::setw(5) << std::setprecision(2) << std::fixed
                  << result->getOccupancy();
      }
      else
      {
        std::cout << " XXXX";
      }
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  map->writeBinary("simple_tree.bt");
  std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
  std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
  std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl << std::endl;

  delete map;

  return 0;
}
