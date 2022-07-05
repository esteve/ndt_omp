#include "voxel_grids_manager.h"

#ifndef PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_
#define PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_


// template<typename PointTarget> void
// pclomp::VoxelGridsManager<PointTarget>::VoxelGridsManager () {}

template<typename PointTarget> void
pclomp::VoxelGridsManager<PointTarget>::addVoxelGridMap(
  const std::string mapID, const PointCloudTargetConstPtr target)
{
  (void)mapID;
  (void)target;
}

template<typename PointTarget> void
pclomp::VoxelGridsManager<PointTarget>::getNeighborhood(
  const PointTarget& reference_point,
  std::vector<TargetGridLeafConstPtr> &neighbors) const
{
  (void)reference_point;
  (void)neighbors;
}

#endif // PCLOMP_VOXEL_GRIDS_MANAGER_IMPL_HPP_
