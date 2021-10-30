// eval.cpp

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// =============================================================================
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

// =============================================================================
void eval_map(
  const std::shared_ptr<octomap::OcTree>& map2eval,
  const std::shared_ptr<octomap::OcTree>& map_gt)
{
  double probHit = map2eval->getProbHit();
  double probMiss = map2eval->getProbMiss();
  double resol = map2eval->getResolution();
  double minx, miny, minz;
  map2eval->getMetricMin(minx, miny, minz);
  double maxx, maxy, maxz;
  map2eval->getMetricMax(maxx, maxy, maxz);

  int count_total = 0;
  int count_error = 0;
  int count_false_pos = 0;
  int count_false_neg = 0;
  int count_unknown = 0;
  for (double x = minx - resol/2.0; x <= maxx + resol; x += resol)
  {
    for (double y = miny - resol/2.0; y <= maxy + resol; y += resol)
    {
      for (double z = minz - resol/2.0; z <= maxz + resol; z += resol)
      {
        octomap::point3d query = octomap::point3d(x, y, z);
        octomap::OcTreeNode* res2check = map2eval->search(query);
        if (res2check)
        {
          ++count_total;
          octomap::OcTreeNode* res_ans = map_gt->search(query);
          if (res_ans)
          {
            double occ2check = res2check->getOccupancy();
            double occ_ans = res_ans->getOccupancy();
            if (occ2check >= probHit && occ_ans <= probMiss)
            {
              ++count_false_pos;
              ++count_error;
            }
            else if (occ2check <= probMiss && occ_ans >= probHit)
            {
              ++count_false_neg;
              ++count_error;
            }
          }
          else
          {
            ++count_unknown;
            ++count_error;
          }
        }
      }
    }
  }
  if (count_total > 0)
  {
    std::cout << "Total nodes: " << count_total << std::endl;
    std::cout << "Erroneous nodes: " << count_error << std::endl;
    std::cout << " - False positive nodes: "
              << count_false_pos << std::endl;
    std::cout << " - False negative nodes: "
              << count_false_neg << std::endl;
    std::cout << " - Unknown nodes: "
              << count_unknown << std::endl;
    std::cout << "Error rate: "
              << ((double)count_error/count_total) << std::endl;
  }
  else
  {
    std::cout << "Error: no occ/free node in the map to check." << std::endl;
  }
}

// =============================================================================
void saveAsPCD(
  const std::shared_ptr<octomap::OcTree>& map2save,
  const std::string& file_name)
{
  double probHit = map2save->getProbHit();
  double probMiss = map2save->getProbMiss();
  double resol = map2save->getResolution();
  double minx, miny, minz;
  map2save->getMetricMin(minx, miny, minz);
  double maxx, maxy, maxz;
  map2save->getMetricMax(maxx, maxy, maxz);

  std::vector< std::tuple<double, double, double> > point_list;

  for (double x = minx - resol/2.0; x <= maxx + resol; x += resol)
  {
    for (double y = miny - resol/2.0; y <= maxy + resol; y += resol)
    {
      for (double z = minz - resol/2.0; z <= maxz + resol; z += resol)
      {
        octomap::point3d query = octomap::point3d(x, y, z);
        octomap::OcTreeNode* res2check = map2save->search(query);
        if (res2check)
        {
          double occ2check = res2check->getOccupancy();
          if (occ2check >= probHit)
          {
            // add a point
            auto p = std::make_tuple(x, y, z);
            point_list.push_back(p);
          }
        }
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width    = point_list.size();
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (unsigned int i = 0; i < point_list.size(); ++i)
  {
    auto p = point_list[i];
    cloud.points[i].x = std::get<0>(p);
    cloud.points[i].y = std::get<1>(p);
    cloud.points[i].z = std::get<2>(p);
  }

  pcl::io::savePCDFileASCII (file_name, cloud);
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
  std::shared_ptr<octomap::OcTree> map_gt
    = std::make_shared<octomap::OcTree>(resol);
  {
    map_gt->setProbHit(probHit);
    map_gt->setProbMiss(probMiss);
    map_gt->setClampingThresMin(0.12);
    map_gt->setClampingThresMax(0.97);

    std::cout << "combining submaps into a ground truth map" << std::flush;
    for (auto m: submaps_ground_truth)
    {
      std::cout << "." << std::flush;
      insert_submap(m, map_gt);
    }
    std::cout << " done" << std::endl;

    std::cout << "compressing..." << std::flush;
    map_gt->toMaxLikelihood();
    map_gt->prune();
    std::cout << " done" << std::endl;

    std::cout << "saving the map..." << std::flush;
    map_gt->writeBinary("ground_truth_combined.bt");
    std::cout << " done" << std::endl;

    std::cout << "saving the map as point clouds..." << std::flush;
    saveAsPCD(map_gt, "ground_truth_combined.pcd");
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

    std::cout << "compressing..." << std::flush;
    map->toMaxLikelihood();
    map->prune();
    std::cout << " done" << std::endl;

    std::cout << "evaluating..." << std::endl;
    eval_map(map, map_gt);

    std::cout << "saving the map..." << std::flush;
    map->writeBinary(map_name + "_combined.bt");
    std::cout << " done" << std::endl;

    std::cout << "saving the map as point clouds..." << std::flush;
    saveAsPCD(map, map_name + "_combined.pcd");
    std::cout << " done" << std::endl;
  }

  std::cout << "======================================" << std::endl;

  return 0;
}
