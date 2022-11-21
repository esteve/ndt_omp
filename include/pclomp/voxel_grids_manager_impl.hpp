#include "voxel_grids_manager.h"

#ifndef PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_
#define PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_


template<typename PointTarget>
void pclomp::VoxelGridsManager<PointTarget>::VoxelGridsManager()
: target_cells_dict_()
{
}

template<typename PointTarget>
void pclomp::VoxelGridsManager<PointTarget>::addVoxelGridMap(
  const std::string map_id, const PointCloudTargetConstPtr target)
{
  TargetGrid target_grid = nullptr;
  target_cells_dict_[map_id] = target_grid;
}

template<typename PointTarget>
void pclomp::VoxelGridsManager<PointTarget>::removeVoxelGridMap(
  const std::string map_id)
{
  target_cells_dict_.erase(map_id);
}

template<typename PointTarget>
void pclomp::VoxelGridsManager<PointTarget>::createKdTree()
{
  if (!searchable_ || (voxel_centroids_->size() == 0)) return;
  kdtree_.setInputCloud (voxel_centroids_);
  return;
}

template<typename PointTarget>
int pclomp::VoxelGridsManager<PointTarget>::radiusSearch (
  const PointT & point, double radius, std::vector<LeafConstPtr> & k_leaves,
  std::vector<float> & k_sqr_distances, unsigned int max_nn = 0) const
{
  k_leaves.clear ();

  // // Check if kdtree has been built
  // if (!searchable_)
  // {
  //   PCL_WARN ("%s: Not Searchable", this->getClassName().c_str ());
  //   return 0;
  // }

  // Find neighbors within radius in the occupied voxel centroid cloud
  std::vector<int> k_indices;
  int k = kdtree_.radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);

  // Find leaves corresponding to neighbors
  k_leaves.reserve (k);
  for (std::vector<int>::iterator iter = k_indices.begin (); iter != k_indices.end (); iter++)
  {
    auto leaf = leaves_.find(voxel_centroids_leaf_indices_[*iter]);
    if (leaf == leaves_.end()) {
      std::cerr << "error : could not find the leaf corresponding to the voxel" << std::endl;
      std::cin.ignore(1);
    }
    k_leaves.push_back (&(leaf->second));
  }
  return k;
}

template<typename PointTarget>
int getNeighborhoodAtPoint(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const
{
  (void)reference_point;
  (void)neighbors;
  throw std::runtime_error;
}

template<typename PointTarget>
int getNeighborhoodAtPoint(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const
{
  (void)reference_point;
  (void)neighbors;
  throw std::runtime_error;
}


template<typename PointTarget>
int getNeighborhoodAtPoint1(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const
{
  (void)reference_point;
  (void)neighbors;
  throw std::runtime_error;
}

template<typename PointTarget>
int getNeighborhoodAtPoint7(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const
{
  (void)reference_point;
  (void)neighbors;
  throw std::runtime_error;
}

#endif // PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_
