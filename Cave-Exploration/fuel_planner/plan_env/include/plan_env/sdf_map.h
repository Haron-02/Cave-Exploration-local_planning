#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <memory>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

using namespace std;

namespace cv
{
  class Mat;
}

class RayCaster;

namespace fast_planner
{
  struct MapParam;
  struct MapData;
  class MapROS;

  class SDFMap
  {
  public:
    SDFMap();
    ~SDFMap();

    enum OCCUPANCY
    {
      UNKNOWN,
      FREE,
      OCCUPIED
    };

    enum CHANGE_COLLECTION_MODE
    {
      ALL_IN_PERCEPTION_RANGE,
      ONLY_UPDATED
    };

    void initMap(ros::NodeHandle &nh);
    void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                         const Eigen::Vector3d &camera_pos);

    // double形式的position->int形式的index
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    // int形式的index->double形式的position,position取得是id的中点的转换
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    // 约束sdf中的每一个维度的id在[0, map_voxel_num_-1)
    void boundIndex(Eigen::Vector3i &id);
    // 转换成地址形式(线性存储)
    int toAddress(const Eigen::Vector3i &id);
    int toAddress(const int &x, const int &y, const int &z);
    // 地址形式转换成id
    Eigen::Vector3i AddressToIndex(const int &addr);
    Eigen::Vector3d AddressToPos(const int &addr);

    // 判断一个点是否在map(double 坐标表示)内
    bool isInMap(const Eigen::Vector3d &pos);
    bool isInMap(const Eigen::Vector3i &idx);

    // 判断一个点是否在box(double 坐标表示)内
    bool isInBox(const Eigen::Vector3i &id);
    bool isInBox(const Eigen::Vector3d &pos);
    bool isInBox(const int &addr);

    // 判断一个点是否在frontier box(double 坐标表示)内
    bool isInFrontierBox(const Eigen::Vector3i &id);
    bool isInFrontierBox(const Eigen::Vector3d &pos);

    // 重新确定box的boundary？
    void boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up);
    // 获取当前position的状态信息
    int getOccupancy(const Eigen::Vector3d &pos);
    // 获取当前index的状态信息
    int getOccupancy(const Eigen::Vector3i &id);
    // 获取当前addr的状态信息
    int getOccupancy(const int &addr);
    // 设置当前index的状态信息，设置的是occupancy_buffer_inflate_
    void setOccupied(const Eigen::Vector3d &pos, const int &occ = 1);
    // 获取当前位置的occupancy_buffer_inflate_信息
    int getInflateOccupancy(const Eigen::Vector3d &pos);
    int getInflateOccupancy(const Eigen::Vector3i &id);
    // 获取距离信息，从distance_buffer_中获取
    double getDistance(const Eigen::Vector3d &pos);
    double getDistance(const Eigen::Vector3i &id);

    void resetChangeDetection();
    void enableChangeDetection();
    void disableChangeDetection();
    std::unordered_set<int>::iterator changesBegin();
    std::unordered_set<int>::iterator changesEnd();
    double getMaxRayLength();
    bool isCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target);

    double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
    void updateESDF3d();
    void resetBuffer();
    void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

    void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
    void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
    void getFrontierBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
    void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset = false);
    double getResolution();
    int getVoxelNum();
    double getInflateRadius();

    void setChangeCollectionMode(CHANGE_COLLECTION_MODE mode);

  private:
    void clearAndInflateLocalMap();
    // 膨胀一个点
    void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
    void setCacheOccupancy(const int &adr, const int &occ);
    // 找出在map范围内沿着camera_pt到pt的方向离pt最近的点
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    unique_ptr<MapParam> mp_;
    unique_ptr<MapData> md_;
    unique_ptr<MapROS> mr_;
    unique_ptr<RayCaster> caster_;
    

    friend MapROS;

  public:
    typedef std::shared_ptr<SDFMap> Ptr;
    unique_ptr<RayCaster> collision_check_caster_;
    std::mutex sdf_mutex_;
    std::mutex change_mutex_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<pcl::PointXYZ>::Ptr occupancy_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> occupancy_cloud_stack_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_;
    
    int occupancy_cloud_count_;
  };

  struct MapParam
  {
    // map properties
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;
    Eigen::Vector3i map_voxel_num_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    double virtual_ceil_height_, ground_height_;
    Eigen::Vector3i box_min_, box_max_;
    Eigen::Vector3d box_mind_, box_maxd_;

    Eigen::Vector3i frontier_min_, frontier_max_;
    Eigen::Vector3d frontier_mind_, frontier_maxd_;

    double default_dist_;
    bool optimistic_, signed_dist_;
    // map fusion
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                           // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit
    double max_ray_length_;
    double local_bound_inflate_;
    int local_map_margin_;
    double unknown_flag_;
    int change_colletion_mode_;
    bool enable_change_detection_;
  };

  struct MapData
  {
    // main map data, occupancy of each voxel and Euclidean distance
    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;
    std::unordered_set<int> change_buffer_;
    // data for updating
    vector<short> count_hit_, count_miss_, count_hit_and_miss_;
    vector<char> flag_rayend_, flag_visited_;
    char raycast_num_;
    queue<int> cache_voxel_;
    Eigen::Vector3i local_bound_min_, local_bound_max_;
    Eigen::Vector3d update_min_, update_max_;
    Eigen::Vector3d occupancy_min_, occupancy_max_;
    Eigen::Vector3i occupancy_min_i_, occupancy_max_i_;
    bool sdf_map_got_;
    bool reset_updated_box_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  inline void SDFMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
  }

  inline void SDFMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
  {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
  }

  inline void SDFMap::boundIndex(Eigen::Vector3i &id)
  {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
    id = id1;
  }

  inline int SDFMap::toAddress(const int &x, const int &y, const int &z)
  {
    return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
  }

  inline int SDFMap::toAddress(const Eigen::Vector3i &id)
  {
    return toAddress(id[0], id[1], id[2]);
  }

  inline Eigen::Vector3i SDFMap::AddressToIndex(const int &addr)
  {
    return Eigen::Vector3i(addr / (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)),
                           addr % (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) / mp_->map_voxel_num_(2),
                           addr % (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) % mp_->map_voxel_num_(2));
  }

  inline Eigen::Vector3d SDFMap::AddressToPos(const int &addr)
  {
    Eigen::Vector3d pos;
    indexToPos(AddressToIndex(addr), pos);
    return pos;
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_->map_min_boundary_(2) + 1e-4)
      return false;
    if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_->map_max_boundary_(2) - 1e-4)
      return false;
    return true;
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3i &idx)
  {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
      return false;
    if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
        idx(2) > mp_->map_voxel_num_(2) - 1)
      return false;
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3d &pos)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline bool SDFMap::isInBox(const int &addr)
  {
    Eigen::Vector3i id = AddressToIndex(addr);
    return isInBox(id);
  }

  inline void SDFMap::boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up)
  {
    for (int i = 0; i < 3; ++i)
    {
      low[i] = max(low[i], mp_->box_mind_[i]);
      up[i] = min(up[i], mp_->box_maxd_[i]);
    }
  }

  inline bool SDFMap::isInFrontierBox(const Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (id[i] < mp_->frontier_min_[i] || id[i] >= mp_->frontier_max_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline bool SDFMap::isInFrontierBox(const Eigen::Vector3d &pos)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (pos[i] < mp_->frontier_mind_[i] || pos[i] >= mp_->frontier_maxd_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    double occ = md_->occupancy_buffer_[toAddress(id)];
    if (occ < mp_->clamp_min_log_ - 1e-3)
      return UNKNOWN;
    if (occ > mp_->min_occupancy_log_)
      return OCCUPIED;
    return FREE;
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id);
  }

  inline int SDFMap::getOccupancy(const int &addr)
  {
    Eigen::Vector3i id = AddressToIndex(addr);
    return getOccupancy(id);
  }

  inline void SDFMap::setOccupied(const Eigen::Vector3d &pos, const int &occ)
  {
    if (!isInMap(pos))
      return;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id);
  }

  inline double SDFMap::getDistance(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    return md_->distance_buffer_[toAddress(id)];
  }

  inline double SDFMap::getDistance(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getDistance(id);
  }

  inline void SDFMap::resetChangeDetection()
  {
    md_->change_buffer_.clear();
  }

  inline void SDFMap::enableChangeDetection()
  {
    mp_->enable_change_detection_ = true;
  }

  inline void SDFMap::disableChangeDetection()
  {
    mp_->enable_change_detection_ = false;
  }

  inline std::unordered_set<int>::iterator SDFMap::changesBegin()
  {
    return md_->change_buffer_.begin();
  }

  inline std::unordered_set<int>::iterator SDFMap::changesEnd()
  {
    return md_->change_buffer_.end();
  }

  inline double SDFMap::getMaxRayLength()
  {
    return mp_->max_ray_length_;
  }

  inline double SDFMap::getInflateRadius()
  {
    return mp_->obstacles_inflation_;
  }

  inline void SDFMap::inflatePoint(const Eigen::Vector3i &pt, int step,
                                   vector<Eigen::Vector3i> &pts)
  {
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z)
        {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }
}
#endif