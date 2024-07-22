/**
 * @file map_util.h
 * @brief MapUtil classes
 */
#ifndef JPS_MAP_UTIL_H
#define JPS_MAP_UTIL_H

#include <data_type/data_type.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap_server/OctomapServer.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <map>
using namespace Eigen;

/// The type of map data Tmap is defined as a 1D array
using Tmap = std::vector<signed char>;
/**
 * @biref The map util class for collision checking
 * @param Dim is the dimension of the workspace
 */
template <int Dim>
class MapUtil
{
 public:
  /// Simple constructor
  MapUtil() {}
  /// Get map data
  Tmap getMap() { return map_; }
  //
  bool has_map_() { return has_map; }
  /// Get resolution
  decimal_t getRes() { return res_; }
  /// Get dimensions
  Veci<Dim> getDim() { return dim_; }
  /// Get origin
  Vecf<Dim> getOrigin() { return origin_d_; }
  void setParam(ros::NodeHandle &nh)
  {
    // we only use jps3d
    if (Dim != 3)
    {
      ROS_ERROR("SEARCH MUST BE 3D!");
    }
    nh.param("/map/resolution", res_, 0.1);
    nh.param("/map/x_size", map_size(0), 100.0);
    nh.param("/map/y_size", map_size(1), 100.0);
    nh.param("/map/z_size", map_size(2), 2.0);
    nh.param("/map/origin_x", origin_d_(0), -50.0);
    nh.param("/map/origin_y", origin_d_(1), -50.0);
    nh.param("/map/origin_z", origin_d_(2), 0.0);
    nh.param("/world_frame_id", world_frame_id, std::string("world"));

    dim_(0) = map_size(0) / res_;
    dim_(1) = map_size(1) / res_;
    dim_(2) = map_size(2) / res_;
    int buffer_size = dim_(0) * dim_(1) * dim_(2);
    map_.resize(buffer_size, 0);
    point_cloud_sub_ = nh.subscribe("/global_map", 10, &MapUtil::GlobalMapBuild, this);
  }
  void GlobalMapBuild(const sensor_msgs::PointCloud2 &pointcloud_map)
  {
    if (has_map) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pointcloud_map, cloud);
    ROS_INFO("Received the point cloud");
    ROS_INFO("map_util is building the map! please wait~");

    if ((int)cloud.points.size() == 0) return;
    pcl::PointXYZ pt;
    ROS_INFO("cloud points size=%d\n", (int)cloud.points.size());
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
      pt = cloud.points[idx];
      setObs(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    has_map = true;
    ROS_WARN("Finish gridmap built");
  }
  void setObs(Eigen::Vector3d pt)
  {
    int expand_size = 0;  // inflate size
    double coord_x = pt[0];
    double coord_y = pt[1];
    double coord_z = pt[2];

    Veci<Dim> index3i = floatToInt(Vecf<Dim>(coord_x, coord_y, coord_z));
    if (isOutside(index3i)) return;
    for (int i = -expand_size; i <= expand_size; i++)
      for (int j = -expand_size; j <= expand_size; j++)
        for (int k = -expand_size; k <= expand_size; k++)
        {
          Veci<Dim> temp_index;
          temp_index(0) = index3i(0) + i;
          temp_index(1) = index3i(1) + j;
          temp_index(2) = index3i(2) + k;
          if (isOutside(temp_index)) continue;
          map_[getIndex(temp_index)] = val_occ;
        }
  }
  int toAddress(int x, int y, int z)
  {
    Vec3i pn;
    pn[0] = x;
    pn[1] = y;
    pn[2] = z;
    return getIndex(pn);
  }
  /// Get index of a cell
  int getIndex(const Veci<Dim> &pn)
  {
    return Dim == 2 ? pn(0) + dim_(0) * pn(1)
                    : pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
  }

  /// Check if the given cell is outside of the map in i-the dimension
  bool isOutsideXYZ(const Veci<Dim> &n, int i) { return n(i) < 0 || n(i) >= dim_(i); }
  /// Check if the cell is free by index
  bool isFree(int idx) { return map_[idx] == val_free; }
  /// Check if the cell is unknown by index
  bool isUnknown(int idx) { return map_[idx] == val_unknown; }
  /// Check if the cell is occupied by index
  bool isOccupied(int idx) { return map_[idx] > val_free; }
  // free 0 occ 100 unknow -1
  /// Check if the cell is outside by coordinate
  bool isOutside(const Veci<Dim> &pn)
  {
    for (int i = 0; i < Dim; i++)
      if (pn(i) < 0 || pn(i) >= dim_(i)) return true;
    return false;
  }
  /// Check if the given cell is free by coordinate
  bool isFree(const Veci<Dim> &pn)
  {
    if (isOutside(pn))
      return false;
    else
      return isFree(getIndex(pn));
  }
  /// Check if the given cell is occupied by coordinate
  bool isOccupied(const Veci<Dim> &pn)
  {
    if (isOutside(pn))
      return true;
    else
      return isOccupied(getIndex(pn));
  }
  /// Check if the given cell is unknown by coordinate
  bool isUnknown(const Veci<Dim> &pn)
  {
    if (isOutside(pn)) return false;
    return map_[getIndex(pn)] == val_unknown;
  }

  /**
   * @brief Set map
   *
   * @param ori origin position
   * @param dim number of cells in each dimension
   * @param map array of cell values
   * @param res map resolution
   */
  void setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim, const Tmap &map, decimal_t res)
  {
    map_ = map;
    dim_ = dim;
    origin_d_ = ori;
    res_ = res;
  }

  /// Print basic information about the util
  void info()
  {
    Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
    std::cout << "MapUtil Info ========================== " << std::endl;
    std::cout << "   res: [" << res_ << "]" << std::endl;
    std::cout << "   origin: [" << origin_d_.transpose() << "]" << std::endl;
    std::cout << "   range: [" << range.transpose() << "]" << std::endl;
    std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
  };

  /// Float position to discrete cell coordinate
  Veci<Dim> floatToInt(const Vecf<Dim> &pt)
  {
    Veci<Dim> pn;
    for (int i = 0; i < Dim; i++) pn(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
    return pn;
  }
  /// Discrete cell coordinate to float position
  Vecf<Dim> intToFloat(const Veci<Dim> &pn)
  {
    // return pn.template cast<decimal_t>() * res_ + origin_d_;
    return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) * res_ + origin_d_;
  }

  /// Raytrace from float point pt1 to pt2
  vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2)
  {
    Vecf<Dim> diff = pt2 - pt1;
    decimal_t k = 0.8;
    int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
    decimal_t s = 1.0 / max_diff;
    Vecf<Dim> step = diff * s;

    vec_Veci<Dim> pns;
    Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
    for (int n = 1; n < max_diff; n++)
    {
      Vecf<Dim> pt = pt1 + step * n;
      Veci<Dim> new_pn = floatToInt(pt);
      if (isOutside(new_pn)) break;
      if (new_pn != prev_pn) pns.push_back(new_pn);
      prev_pn = new_pn;
    }
    return pns;
  }

  /// Check if the ray from p1 to p2 is occluded
  bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2, int8_t val = 100)
  {
    vec_Veci<Dim> pns = rayTrace(p1, p2);
    for (const auto &pn : pns)
    {
      if (map_[getIndex(pn)] >= val) return true;
    }
    return false;
  }

  /// Get occupied voxels
  vec_Vecf<Dim> getCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Get free voxels
  vec_Vecf<Dim> getFreeCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Get unknown voxels
  vec_Vecf<Dim> getUnknownCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Dilate occupied cells
  void dilate(const vec_Veci<Dim> &dilate_neighbor)
  {
    Tmap map = map_;
    Veci<Dim> n = Veci<Dim>::Zero();
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isOccupied(getIndex(n)))
            {
              for (const auto &it : dilate_neighbor)
              {
                if (!isOutside(n + it)) map[getIndex(n + it)] = val_occ;
              }
            }
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isOccupied(getIndex(n)))
          {
            for (const auto &it : dilate_neighbor)
            {
              if (!isOutside(n + it)) map[getIndex(n + it)] = val_occ;
            }
          }
        }
      }
    }

    map_ = map;
  }

  /// Free unknown voxels
  void freeUnknown()
  {
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isUnknown(getIndex(n))) map_[getIndex(n)] = val_free;
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isUnknown(getIndex(n))) map_[getIndex(n)] = val_free;
        }
      }
    }
  }

  /// Map entity
  Tmap map_;
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  std::string world_frame_id;

 protected:
  /// Resolution
  decimal_t res_;
  /// Origin, float type
  Vecf<Dim> origin_d_;
  /// Dimension, int type
  Veci<Dim> dim_;
  Vecf<Dim> map_size;
  /// Assume occupied cell has value 100
  int8_t val_occ = 100;
  /// Assume free cell has value 0
  int8_t val_free = 0;
  /// Assume unknown cell has value -1
  int8_t val_unknown = -1;
  bool has_map = false;
  ros::Subscriber point_cloud_sub_;
};

typedef MapUtil<2> OccMapUtil;

typedef MapUtil<3> VoxelMapUtil;

#endif
