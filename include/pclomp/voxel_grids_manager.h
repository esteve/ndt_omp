#include <pcl/pcl_macros.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <unordered_map>
#include <pcl/point_types.h>
#include "voxel_grid_covariance_omp.h"

#ifndef PCLOMP_VOXEL_GRIDS_MANAGER_H_
#define PCLOMP_VOXEL_GRIDS_MANAGER_H_

namespace pclomp
{

template<typename PointTarget> class VoxelGridsManager
{
protected:
  /** \brief Typename of searchable voxel grid containing mean and covariance. */
  typedef pclomp::VoxelGridCovariance<PointTarget> TargetGrid;
  /** \brief Typename of pointer to searchable voxel grid. */
  typedef TargetGrid* TargetGridPtr;
  /** \brief Typename of const pointer to searchable voxel grid. */
  typedef const TargetGrid* TargetGridConstPtr;
  /** \brief Typename of const pointer to searchable voxel grid leaf. */
  typedef typename TargetGrid::LeafConstPtr TargetGridLeafConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  struct LeafID {
    std::string voxel_id;
    int leaf_id;
    bool operator < (const LeafID& rhs) const {
      if (voxel_id < rhs.voxel_id) {
        return true;
      }
      if (voxel_id > rhs.voxel_id) {
        return false;
      }
      if (leaf_id < rhs.leaf_id) {
        return true;
      }
      if (leaf_id > rhs.leaf_id) {
        return false;
      }
      return false;
    }
  };

public:
  VoxelGridsManager();
  void removeVoxelGridMap(const std::string map_id);
  void addVoxelGridMap(const std::string map_id, const PointCloudTargetConstPtr target);
  void getNeighborhood(const PointTarget& reference_point, std::vector<TargetGridLeafConstPtr> &neighbors) const;
  int radiusSearch (
    const PointT & point, double radius, std::vector<LeafConstPtr> & k_leaves,
    std::vector<float> & k_sqr_distances, unsigned int max_nn = 0) const;
  int getNeighborhoodAtPoint(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const;
  int getNeighborhoodAtPoint1(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const;
  int getNeighborhoodAtPoint7(const PointT & reference_point, std::vector<LeafConstPtr> & neighbors) const;

private:
	std::map<LeafID, Leaf> leaves_;
  PointCloudPtr voxel_centroids_;
  std::vector<LeafID> voxel_centroids_leaf_indices_;

  pcl::KdTreeFLANN<PointT> kdtree_;
  std::map<std::string, TargetGrid> target_cells_dict_;

};
} // namespace pclomp

#endif // PCLOMP_VOXEL_GRIDS_MANAGER_H_
