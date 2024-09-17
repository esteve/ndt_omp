#include <autoware/ndt_omp/multigrid_pclomp/multigrid_ndt_omp.h>
#include <autoware/ndt_omp/pclomp/gicp_omp.h>
#include <autoware/ndt_omp/pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>

// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr align(
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud)
{
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  auto t1 = std::chrono::system_clock::now();
  registration->align(*aligned);
  auto t2 = std::chrono::system_clock::now();
  std::cout << "single : " << static_cast<double>((t2 - t1).count()) / 1e6 << "[msec]" << std::endl;

  for (int i = 0; i < 10; i++) {
    registration->align(*aligned);
  }
  auto t3 = std::chrono::system_clock::now();
  std::cout << "10times: " << static_cast<double>((t3 - t2).count()) / 1e6 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

  return aligned;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cout << "usage: align target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if (pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }

  // downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  source_cloud = downsampled;

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned;

  // benchmark
  std::cout << "--- pcl::GICP ---" << std::endl;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(
    new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  // cppcheck-suppress redundantAssignment
  aligned = align(gicp, target_cloud, source_cloud);

  std::cout << "--- autoware::ndt_omp::pclomp::GICP ---" << std::endl;
  autoware::ndt_omp::pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr
    gicp_omp(new autoware::ndt_omp::pclomp::GeneralizedIterativeClosestPoint<
             pcl::PointXYZ, pcl::PointXYZ>());
  // cppcheck-suppress redundantAssignment
  aligned = align(gicp_omp, target_cloud, source_cloud);

  std::cout << "--- pcl::NDT ---" << std::endl;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
    new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt->setResolution(1.0);
  // cppcheck-suppress redundantAssignment
  aligned = align(ndt, target_cloud, source_cloud);

  std::vector<int> num_threads = {1, omp_get_max_threads()};
  std::vector<std::pair<std::string, autoware::ndt_omp::pclomp::NeighborSearchMethod>>
    search_methods = {
      {"KDTREE", autoware::ndt_omp::pclomp::KDTREE},
      {"DIRECT7", autoware::ndt_omp::pclomp::DIRECT7},
      {"DIRECT1", autoware::ndt_omp::pclomp::DIRECT1}};

  autoware::ndt_omp::pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr
    ndt_omp(
      new autoware::ndt_omp::pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);

  autoware::ndt_omp::pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::
    Ptr mg_ndt_omp(new autoware::ndt_omp::pclomp::MultiGridNormalDistributionsTransform<
                   pcl::PointXYZ, pcl::PointXYZ>());
  mg_ndt_omp->setResolution(1.0);

  for (int n : num_threads) {
    for (const auto & search_method : search_methods) {
      std::cout << "--- autoware::ndt_omp::pclomp::NDT (" << search_method.first << ", " << n
                << " threads) ---" << std::endl;
      ndt_omp->setNumThreads(n);
      ndt_omp->setNeighborhoodSearchMethod(search_method.second);
      // cppcheck-suppress redundantAssignment
      aligned = align(ndt_omp, target_cloud, source_cloud);
    }

    std::cout << "--- multigrid_pclomp::NDT (" << n << " threads) ---" << std::endl;
    mg_ndt_omp->setNumThreads(n);
    // cppcheck-suppress redundantAssignment
    aligned = align(mg_ndt_omp, target_cloud, source_cloud);
  }

  // visualization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(
    target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(
    source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(
    aligned, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(aligned, aligned_handler, "aligned");
  vis.spin();

  return 0;
}
