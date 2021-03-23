// eval.cpp

#include <iostream>
#include <iomanip>

#include <yaml-cpp/yaml.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node)
{
  if (node != NULL)
    std::cout << "occupancy probability at "
              << query << ":\t " << node->getOccupancy() << std::endl;
  else
    std::cout << "occupancy probability at "
              << query << ":\t is unknown" << std::endl;
}

int main(int argc, char** argv )
{
  std::ifstream fin("settings.yaml");
  YAML::Node doc = YAML::Load(fin);

  std::vector< std::shared_ptr<octomap::OcTree> > submaps_ground_truth;
  std::cout << "======================================" << std::endl;
  std::cout << "ground_truth:" << std::endl;
  for (int i = 0; i < doc["ground_truth"].size(); ++i)
  {
    std::string file_name = doc["ground_truth"][i].as<std::string>();
    std::cout << "opening: " << file_name << std::endl;
    submaps_ground_truth.push_back(std::make_shared<octomap::OcTree>(file_name));
  }

  std::map<std::string,
           std::vector< std::shared_ptr<octomap::OcTree> > > map2submaps;
  for (int i = 0; i < doc["maps"].size(); ++i)
  {
    std::cout << "======================================" << std::endl;
    std::string map_name = doc["maps"][i].as<std::string>();
    std::cout << "map: " << map_name << std::endl;
    for (int j = 0; j < doc[map_name].size(); ++j)
    {
      std::string file_name = doc[map_name][j].as<std::string>();
      std::cout << "opening: " << file_name << std::endl;
      std::vector< std::shared_ptr<octomap::OcTree> > submaps;
      submaps.push_back(std::make_shared<octomap::OcTree>(file_name));
      map2submaps[map_name] = std::move(submaps);
    }
  }
  std::cout << "======================================" << std::endl;

  std::cout << "combining submaps into a ground truth map" << std::endl;
  double resol = submaps_ground_truth[0]->getResolution();
  double probHit = submaps_ground_truth[0]->getProbHit();
  double probMiss = submaps_ground_truth[0]->getProbMiss();
  {
    std::shared_ptr<octomap::OcTree> map
      = std::make_shared<octomap::OcTree>(resol);

    for (auto m: submaps_ground_truth)
    {
      double minx, miny, minz;
      m->getMetricMin(minx, miny, minz);
      double maxx, maxy, maxz;
      m->getMetricMax(maxx, maxy, maxz);

      for (double x = minx; x <= maxx + resol; x += resol)
      {
        for (double y = miny; y <= maxx + resol; y += resol)
        {
          for (double z = minz; z <= maxz + resol; z += resol)
          {
            octomap::point3d query = octomap::point3d(x, y, z);
            octomap::OcTreeNode* result = map->search(query);
            if (result)
            {
              double occupancy = result->getOccupancy();
              octomap::point3d endpoint((float) x, (float) y, (float) z);
              if (occupancy == probHit)
                map->updateNode(endpoint, true);
              else if (occupancy == probMiss)
                map->updateNode(endpoint, false);
            }
          }
        }
      }
    }
  }

  std::cout << "======================================" << std::endl;

  for (auto p: map2submaps)
  {
    std::string map_name = p.first;
    std::vector< std::shared_ptr<octomap::OcTree> > submaps = p.second;
    std::cout << "combining submaps into the map: " << map_name << std::endl;

    std::shared_ptr<octomap::OcTree> map
      = std::make_shared<octomap::OcTree>(resol);

    for (auto m: submaps)
    {
      double minx, miny, minz;
      m->getMetricMin(minx, miny, minz);
      double maxx, maxy, maxz;
      m->getMetricMax(maxx, maxy, maxz);

      for (double x = minx; x <= maxx + resol; x += resol)
      {
        for (double y = miny; y <= maxx + resol; y += resol)
        {
          for (double z = minz; z <= maxz + resol; z += resol)
          {
            octomap::point3d query = octomap::point3d(x, y, z);
            octomap::OcTreeNode* result = map->search(query);
            if (result)
            {
              double occupancy = result->getOccupancy();
              octomap::point3d endpoint((float) x, (float) y, (float) z);
              if (occupancy == probHit)
                map->updateNode(endpoint, true);
              else if (occupancy == probMiss)
                map->updateNode(endpoint, false);
            }
          }
        }
      }
    }
  }

  std::cout << "======================================" << std::endl;

  // octomap::OcTree *map;
  //
  // map = new octomap::OcTree(0.1);
  // map->setProbHit(0.7);
  // map->setProbMiss(0.4);
  // map->setClampingThresMin(0.12);
  // map->setClampingThresMax(0.97);
  //
  // double offset_x = 1.0;
  // double offset_y = 1.0;
  // double offset_z = 1.0;
  //
  // for (double x = -1.0 + offset_x; x < 1.0 + offset_x; x += 0.05f)
  // {
  //   for (double y = -1.0 + offset_y; y < 1.0 + offset_y; y += 0.05f)
  //   {
  //     for (double z = -1.0 + offset_z; z < 1.0 + offset_z; z += 0.05f)
  //     {
  //       octomap::point3d endpoint ((float) x, (float) y, (float) z);
  //       map->updateNode(endpoint, true); // integrate 'occupied' measurement
  //     }
  //   }
  // }
  // for (double x = -1.58 + offset_x; x <= -0.4 + offset_x; x += 0.02f)
  // {
  //   for (double y = -1.58 + offset_y; y <= -0.4 + offset_y; y += 0.02f)
  //   {
  //     for (double z = -1.58 + offset_z; z <= -0.4 + offset_z; z += 0.02f)
  //     {
  //       octomap::point3d endpoint ((float) x, (float) y, (float) z);
  //       map->updateNode(endpoint, false);  // integrate 'free' measurement
  //     }
  //   }
  // }
  //
  // std::cout << std::endl;
  // std::cout << "performing some queries:" << std::endl;
  //
  // octomap::point3d query (0., 0., 0.);
  // octomap::OcTreeNode* result = map->search (query);
  // print_query_info(query, result);
  //
  // query = octomap::point3d(-1.,-1.,-1.);
  // result = map->search (query);
  // print_query_info(query, result);
  //
  // query = octomap::point3d(1.,1.,1.);
  // result = map->search (query);
  // print_query_info(query, result);
  //
  // std::cout << "=== scan (fixed) ===" << std::endl;
  // for (double x = -1.7; x <= 1.7; x += 0.1)
  // {
  //   for (double y = -1.7; y <= 1.7; y += 0.1)
  //   {
  //     query = octomap::point3d(x, y, -0.5);
  //     result = map->search (query);
  //     //print_query_info(query, result);
  //     if (result)
  //     {
  //       std::cout << std::setw(5) << std::setprecision(2) << std::fixed
  //                 << result->getOccupancy();
  //     }
  //     else
  //     {
  //       std::cout << " XXXX";
  //     }
  //   }
  //   std::cout << std::endl;
  // }
  //
  // std::cout << std::endl;
  //
  // double x, y, z;
  // map->getMetricSize(x, y, z);
  // std::cout << "size: x = " << x << ", y = " << y << ", z = " << z
  //           << std::endl;
  // double minx, miny, minz;
  // map->getMetricMin(minx, miny, minz);
  // std::cout << "min: x = " << minx << ", y = " << miny << ", z = " << minz
  //           << std::endl;
  // double maxx, maxy, maxz;
  // map->getMetricMax(maxx, maxy, maxz);
  // std::cout << "max: x = " << maxx << ", y = " << maxy << ", z = " << maxz
  //           << std::endl;
  //
  // std::cout << "=== scan (flexible) ===" << std::endl;
  // for (double x = minx; x <= maxx + 0.1; x += 0.1)
  // {
  //   for (double y = miny; y <= maxx + 0.1; y += 0.1)
  //   {
  //     query = octomap::point3d(x, y, (maxz - minz)*0.4 + minz);
  //     result = map->search (query);
  //     //print_query_info(query, result);
  //     if (result)
  //     {
  //       std::cout << std::setw(5) << std::setprecision(2) << std::fixed
  //                 << result->getOccupancy();
  //     }
  //     else
  //     {
  //       std::cout << " XXXX";
  //     }
  //   }
  //   std::cout << std::endl;
  // }
  //
  // std::cout << std::endl;
  //
  // map->writeBinary("simple_tree.bt");
  // std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
  // std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
  // std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl << std::endl;
  //
  // delete map;

  return 0;
}
