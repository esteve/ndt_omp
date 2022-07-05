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
  template<typename PointTarget>
  class VoxelGridsManager
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

        std::map<std::string, TargetGrid> target_cells_dict_;

    public:
        VoxelGridsManager();
        void removeVoxelGridMap(const std::string mapID);
        void addVoxelGridMap(const std::string mapID, const PointCloudTargetConstPtr target);
        void getNeighborhood(const PointTarget& reference_point, std::vector<TargetGridLeafConstPtr> &neighbors) const ;
  };
}

#endif // PCLOMP_VOXEL_GRIDS_MANAGER_H_
