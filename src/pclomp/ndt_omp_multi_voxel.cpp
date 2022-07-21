#include <pclomp/ndt_omp_multi_voxel.h>
#include <pclomp/ndt_omp_multi_voxel_impl.hpp>

template class pclomp::NormalDistributionsTransformMultiVoxel<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransformMultiVoxel<pcl::PointXYZI, pcl::PointXYZI>;
