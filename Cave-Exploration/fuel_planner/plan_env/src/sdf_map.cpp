#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/raycast.h>

namespace fast_planner
{
  SDFMap::SDFMap()
  {
  }

  SDFMap::~SDFMap()
  {
  }

  void SDFMap::initMap(ros::NodeHandle &nh)
  {
    mp_.reset(new MapParam);
    md_.reset(new MapData);
    mr_.reset(new MapROS);

    // Params of map properties
    double x_size, y_size, z_size;
    nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0); // 0.199
    nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);  // 0.5
    nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
    nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
    nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
    nh.param("sdf_map/optimistic", mp_->optimistic_, true);
    nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);
    // mp的一些基本参数
    mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
    mp_->resolution_inv_ = 1 / mp_->resolution_;
    mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
    // mp_->map_origin_ = Eigen::Vector3d(0, 0, mp_->ground_height_);
    mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    for (int i = 0; i < 3; ++i)
      mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
    // ESDF的范围不是基于(0,0,0)对称的
    mp_->map_min_boundary_ = mp_->map_origin_;
    mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
    mp_->enable_change_detection_ = false;
    mp_->change_colletion_mode_ = static_cast<int>(ALL_IN_PERCEPTION_RANGE);

    // Params of raycasting-based fusion
    nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
    nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
    nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
    nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
    nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
    nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
    nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);

    auto logit = [](const double &x)
    { return log(x / (1 - x)); };
    mp_->prob_hit_log_ = logit(mp_->p_hit_);
    mp_->prob_miss_log_ = logit(mp_->p_miss_);
    mp_->clamp_min_log_ = logit(mp_->p_min_);
    mp_->clamp_max_log_ = logit(mp_->p_max_);
    mp_->min_occupancy_log_ = logit(mp_->p_occ_);
    mp_->unknown_flag_ = 0.01;
    cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
         << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
         << ", thresh: " << mp_->min_occupancy_log_ << endl;

    // Initialize data buffer of map
    int buffer_size = (mp_->map_voxel_num_(0) + 1) * (mp_->map_voxel_num_(1) + 1) * (mp_->map_voxel_num_(2) + 1);
    md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
    md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
    md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
    md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
    md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
    md_->count_hit_ = vector<short>(buffer_size, 0);
    md_->count_miss_ = vector<short>(buffer_size, 0);
    md_->flag_rayend_ = vector<char>(buffer_size, -1);
    md_->flag_visited_ = vector<char>(buffer_size, -1);
    md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
    md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
    md_->raycast_num_ = 0;
    md_->reset_updated_box_ = true;
    md_->sdf_map_got_ = false;
    md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);
    md_->occupancy_min_ = Eigen::Vector3d(1e10, 1e10, 1e10);
    md_->occupancy_max_ = Eigen::Vector3d(-1e10, -1e10, -1e10);
    md_->occupancy_min_i_ = Eigen::Vector3i(1e10, 1e10, 1e10);
    md_->occupancy_max_i_ = Eigen::Vector3i(0, 0, 0);

    // Try retriving bounding box of map, set box to map size if not specified
    vector<string> axis = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
      nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
      nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
    }
    posToIndex(mp_->box_mind_, mp_->box_min_);
    posToIndex(mp_->box_maxd_, mp_->box_max_);

    for (int i = 0; i < 3; ++i)
    {
      nh.param("sdf_map/map_min_" + axis[i], mp_->frontier_mind_[i], mp_->map_min_boundary_[i]);
      nh.param("sdf_map/map_max_" + axis[i], mp_->frontier_maxd_[i], mp_->map_max_boundary_[i]);
    }
    posToIndex(mp_->frontier_mind_, mp_->frontier_min_);
    posToIndex(mp_->frontier_maxd_, mp_->frontier_max_);

    ROS_WARN("frontier_mind_z:%0.2f", mp_->frontier_mind_.z());

    // Initialize ROS wrapper
    mr_->setMap(this);
    mr_->node_ = nh;
    mr_->init();

    caster_.reset(new RayCaster);
    caster_->setParams(mp_->resolution_, mp_->map_origin_);
    collision_check_caster_.reset(new RayCaster);
    collision_check_caster_->setParams(mp_->resolution_, mp_->map_origin_);

    // 法向量计算用点云
    occupancy_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    occupancy_cloud_stack_.resize(20);
    for (int i = 0; i < occupancy_cloud_stack_.size(); i++)
    {
      occupancy_cloud_stack_[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    local_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  
  }

  void SDFMap::resetBuffer()
  {
    resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
    md_->local_bound_min_ = Eigen::Vector3i::Zero();
    md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
  }

  void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
  {
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y)
        for (int z = min_id(2); z <= max_id(2); ++z)
        {
          md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
          md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
        }
  }

  template <typename F_get_val, typename F_set_val>
  void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val,
                        int start, int end, int dim)
  {
    int v[mp_->map_voxel_num_(dim)];
    double z[mp_->map_voxel_num_(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++)
    {
      k++;
      double s;

      do
      {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
      } while (s <= z[k]);

      k++;

      v[k] = q;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++)
    {
      while (z[k + 1] < q)
        k++;
      double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
      f_set_val(q, val);
    }
  }

  void SDFMap::updateESDF3d()
  {
    Eigen::Vector3i min_esdf = md_->local_bound_min_;
    Eigen::Vector3i max_esdf = md_->local_bound_max_;

    if (mp_->optimistic_)
    {
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        {
          fillESDF(
              [&](int z)
              {
                return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();
              },
              [&](int z, double val)
              { md_->tmp_buffer1_[toAddress(x, y, z)] = val; },
              min_esdf[2],
              max_esdf[2], 2);
        }
    }
    else
    {
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        {
          fillESDF(
              [&](int z)
              {
                int adr = toAddress(x, y, z);
                return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                        md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                           ? 0
                           : std::numeric_limits<double>::max();
              },
              [&](int z, double val)
              { md_->tmp_buffer1_[toAddress(x, y, z)] = val; },
              min_esdf[2],
              max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
      {
        fillESDF(
            [&](int y)
            { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val)
            { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
            min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
      {
        fillESDF(
            [&](int x)
            { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val)
            {
              md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }

    if (mp_->signed_dist_)
    {
      // Compute negative distance
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        {
          fillESDF(
              [&](int z)
              {
                return md_->occupancy_buffer_inflate_
                                   [x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
                                    y * mp_->map_voxel_num_(2) + z] == 0
                           ? 0
                           : std::numeric_limits<double>::max();
              },
              [&](int z, double val)
              { md_->tmp_buffer1_[toAddress(x, y, z)] = val; },
              min_esdf[2],
              max_esdf[2], 2);
        }
      for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
        {
          fillESDF(
              [&](int y)
              { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
              [&](int y, double val)
              { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
              min_esdf[1],
              max_esdf[1], 1);
        }
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
        {
          fillESDF(
              [&](int x)
              { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
              [&](int x, double val)
              {
                md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
              },
              min_esdf[0], max_esdf[0], 0);
        }
      // Merge negative distance with positive
      for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
          for (int z = min_esdf(2); z <= max_esdf(2); ++z)
          {
            int idx = toAddress(x, y, z);
            if (md_->distance_buffer_neg_[idx] > 0.0)
              md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
          }
    }
  }

  void SDFMap::setCacheOccupancy(const int &adr, const int &occ)
  {
    // Add to update list if first visited
    if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0)
    {
      md_->cache_voxel_.push(adr);
      if (mp_->enable_change_detection_ && isInBox(AddressToIndex(adr)) &&
          mp_->change_colletion_mode_ == ALL_IN_PERCEPTION_RANGE)
      {
        md_->change_buffer_.insert(adr);
      }
    }

    if (occ == 0)
      md_->count_miss_[adr] = 1;
    else if (occ == 1)
      md_->count_hit_[adr] += 1;

    // md_->count_hit_and_miss_[adr] += 1;
    // if (occ == 1)
    //   md_->count_hit_[adr] += 1;
    // if (md_->count_hit_and_miss_[adr] == 1)
    //   md_->cache_voxel_.push(adr);
  }

  void SDFMap::inputPointCloud(
      const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
      const Eigen::Vector3d &camera_pos)
  {
    if (point_num == 0)
      return;
    md_->raycast_num_ += 1;

    Eigen::Vector3d update_min = camera_pos;
    Eigen::Vector3d update_max = camera_pos;
    if (md_->reset_updated_box_)
    {
      md_->update_min_ = camera_pos;
      md_->update_max_ = camera_pos;
      md_->reset_updated_box_ = false;
    }

    occupancy_cloud_stack_[occupancy_cloud_count_]->clear();
    local_cloud_->clear();

    Eigen::Vector3d pt_w, tmp;
    Eigen::Vector3i idx;
    int vox_adr;
    double length;
    for (int i = 0; i < point_num; ++i)
    {
      // 取出每一个世界坐标系下的点
      auto &pt = points.points[i];
      pt_w << pt.x, pt.y, pt.z;
      int tmp_flag; // 找过替代点的话要将tmp_flag设为0
      // Set flag for projected point
      if (!isInMap(pt_w))
      {
        // Find closest point in map and set free
        // 找出在map范围内离pt最近的点
        pt_w = closetPointInMap(pt_w, camera_pos);
        length = (pt_w - camera_pos).norm();
        // 找出在map范围内且max_ray_length_长度内离pt最近的点
        if (length > mp_->max_ray_length_)
          pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        // if (pt_w[2] < 0.2)
        //   continue;
        tmp_flag = 0;
      }
      else
      {
        length = (pt_w - camera_pos).norm();
        if (length > mp_->max_ray_length_)  // 感知范围外的点 & 补的点
        {
          pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
          // if (pt_w[2] < 0.2)
          //   continue;
          tmp_flag = 0;
        }
        else  // 感知范围内的点
        {
          tmp_flag = 1;

          occupancy_cloud_stack_[occupancy_cloud_count_]->points.push_back(points.points[i]);
          local_cloud_->points.push_back(points.points[i]);
        }    
      }

      posToIndex(pt_w, idx);
      vox_adr = toAddress(idx);
      setCacheOccupancy(vox_adr, tmp_flag);

      // 确定要更新区域的最终的update_min和update_max
      for (int k = 0; k < 3; ++k)
      {
        update_min[k] = min(update_min[k], pt_w[k]);
        update_max[k] = max(update_max[k], pt_w[k]);
      }
      // Raycasting between camera center and point
      // 同一次执行inputPointCloud若出现多次到vox_adr的判断，则只raycast一次
      if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
        continue;
      else
        md_->flag_rayend_[vox_adr] = md_->raycast_num_;

      caster_->input(pt_w, camera_pos);
      caster_->nextId(idx);
      while (caster_->nextId(idx)) // 到终点才会返回false
      
        // if(getOccupancy(idx) == OCCUPANCY::OCCUPIED);
        //   break;
      

        setCacheOccupancy(toAddress(idx), 0);
    } // end for (int i = 0; i < point_num; ++i)

    ros::Time t1 = ros::Time::now();
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(occupancy_cloud_stack_[occupancy_cloud_count_]);  // 设置输入点云
    downsample.setLeafSize(0.2f, 0.2f, 0.2f);
    downsample.filter(*occupancy_cloud_stack_[occupancy_cloud_count_]);
    ros::Time t2 = ros::Time::now();

    // ROS_WARN("sdf downsample spend:%f", (t2 - t1).toSec() * 1000);

    occupancy_cloud_count_++;
    occupancy_cloud_count_%=20;

    Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
    posToIndex(update_max + bound_inf, md_->local_bound_max_);
    posToIndex(update_min - bound_inf, md_->local_bound_min_);
    boundIndex(md_->local_bound_min_);
    boundIndex(md_->local_bound_max_);
    mr_->local_updated_ = true;

    // ROS_WARN("Bounding box for subsequent updating:%f", (t2 - t1).toSec() * 1000);

    // Bounding box for subsequent updating
    for (int k = 0; k < 3; ++k)
    {
      md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
      md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);

      md_->occupancy_min_[k] = min(md_->occupancy_min_[k], md_->update_min_[k]);
      md_->occupancy_max_[k] = max(md_->occupancy_max_[k], md_->update_max_[k]);

      md_->occupancy_min_i_[k] = floor((md_->occupancy_min_[k] - mp_->map_origin_[k]) / mp_->resolution_);
      md_->occupancy_max_i_[k] = floor((md_->occupancy_max_[k] - mp_->map_origin_[k]) / mp_->resolution_);
    }
    md_->sdf_map_got_ = true;

    // ROS_WARN("Update log:%f", (t2 - t1).toSec() * 1000);

    // 不断更新各个位置的log值
    while (!md_->cache_voxel_.empty())
    {
      int adr = md_->cache_voxel_.front();
      md_->cache_voxel_.pop();
      
      Eigen::Vector3i temp_idx = AddressToIndex(adr);
      int last_state = getOccupancy(temp_idx);
      
      // TODO 变为OCCUPIED后就不再被消掉了，防止出现障碍物后的frontier
      if(last_state==OCCUPANCY::OCCUPIED)
      {
        md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
        continue;
      }
      // 如果count_hit_>=count_miss_，说明这个位置是一个occupied的可能性更大，否则是free的可能性更大
      double log_odds_update =
          md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;

      md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
      if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
        md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

      md_->occupancy_buffer_[adr] = std::min(
          std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
          mp_->clamp_max_log_);

      if (mp_->enable_change_detection_ && isInBox(temp_idx) &&
          mp_->change_colletion_mode_ == ONLY_UPDATED && getOccupancy(temp_idx) != last_state)
      {
        md_->change_buffer_.insert(adr);
      }
    }
  }

  Eigen::Vector3d
  SDFMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
  {
    Eigen::Vector3d diff = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
    Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
    double min_t = 1000000;
    for (int i = 0; i < 3; ++i)
    {
      if (fabs(diff[i]) > 0)
      {
        double t1 = max_tc[i] / diff[i];
        if (t1 > 0 && t1 < min_t)
          min_t = t1;
        double t2 = min_tc[i] / diff[i];
        if (t2 > 0 && t2 < min_t)
          min_t = t2;
      }
    }
    return camera_pt + (min_t - 1e-3) * diff;
  }

  void SDFMap::clearAndInflateLocalMap()
  {
    // /*clear outside local*/
    // const int vec_margin = 5;

    // Eigen::Vector3i min_cut = md_->local_bound_min_ -
    //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
    //     mp_->local_map_margin_);
    // Eigen::Vector3i max_cut = md_->local_bound_max_ +
    //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
    //     mp_->local_map_margin_);
    // boundIndex(min_cut);
    // boundIndex(max_cut);

    // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin,
    // vec_margin); Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin,
    // vec_margin, vec_margin); boundIndex(min_cut_m); boundIndex(max_cut_m);

    // // clear data outside the local range

    // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

    //     for (int z = min_cut_m(2); z < min_cut(2); ++z) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

    //     for (int y = min_cut_m(1); y < min_cut(1); ++y) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

    //     for (int x = min_cut_m(0); x < min_cut(0); ++x) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // update inflated occupied cells
    // clean outdated occupancy
    // 膨胀步长
    int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
    // 一个存储三维向量的容器 inf_pts，用于存储膨胀范围内的点坐标
    vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
    // inf_pts.resize(4 * inf_step + 3);

    // 使用三重循环遍历局部边界内的每个点，并将其对应的膨胀占用缓冲区的值设置为0，表示未被占用
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
        for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
        {
          md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        }

    // inflate newest occpuied cells
    // 再次使用三重循环遍历局部边界内的每个点。首先，通过 toAddress(x, y, z) 将三维坐标转换为索引。
    // 然后，检查该点对应的占用缓冲区的值是否大于 mp_->min_occupancy_log_，
    // 如果是，则调用 inflatePoint 函数对该点进行膨胀，将膨胀后的点坐标存储在 inf_pts 中。
    // 最后，遍历 inf_pts 中的每个点，并将其转换为索引 idx_inf，检查索引的范围是否在地图的有效范围内，
    // 如果是，则将对应的膨胀占用缓冲区的值设置为1，表示被占用。
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
        for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
        {
          
          int id1 = toAddress(x, y, z);
          if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_)
          {
            inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

            for (auto inf_pt : inf_pts)
            {
              int idx_inf = toAddress(inf_pt);
              if (idx_inf >= 0 &&
                  idx_inf <
                      mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))
              {
                md_->occupancy_buffer_inflate_[idx_inf] = 1;
              }
            }
          }
        }

    // add virtual ceiling to limit flight height
    // 添加虚拟天花板以限制飞行高度：如果 mp_->virtual_ceil_height_ 大于-0.5，表示存在虚拟天花板限制飞行高度。
    // 首先，计算虚拟天花板的索引 ceil_id，将其从高度坐标转换为索引。
    // 然后，使用两重循环遍历局部边界内的每个点，并将对应的占用缓冲区的值设置为 mp_->clamp_max_log_，将其限制为最大占用值。
    if (mp_->virtual_ceil_height_ > -0.5)
    {
      int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
      for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
        for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
        {
          // md_->occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
          md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
        }
    }
  }

  double SDFMap::getResolution()
  {
    return mp_->resolution_;
  }

  int SDFMap::getVoxelNum()
  {
    return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
  }

  void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
  {
    ori = mp_->map_origin_, size = mp_->map_size_;
  }

  void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax)
  {
    bmin = mp_->box_mind_;
    bmax = mp_->box_maxd_;
  }

  void SDFMap::getFrontierBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax)
  {
    bmin = mp_->frontier_mind_;
    bmax = mp_->frontier_maxd_;
  }

  void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset)
  {
    bmin = md_->update_min_;
    bmax = md_->update_max_;
    if (reset)
      md_->reset_updated_box_ = true;
  }

  void SDFMap::setChangeCollectionMode(CHANGE_COLLECTION_MODE mode)
  {
    mp_->change_colletion_mode_ = static_cast<int>(mode);
  }

  double SDFMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad)
  {
    if (!isInMap(pos))
    {
      grad.setZero();
      return 0;
    }

    /* trilinear interpolation */
    Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    posToIndex(pos_m, idx);
    Eigen::Vector3d idx_pos, diff;
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_->resolution_inv_;

    double values[2][2][2];
    for (int x = 0; x < 2; x++)
      for (int y = 0; y < 2; y++)
        for (int z = 0; z < 2; z++)
        {
          Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
          values[x][y][z] = getDistance(current_idx);
        }

    double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
    double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
    double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
    double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
    double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
    double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
    double dist = (1 - diff[2]) * v0 + diff[2] * v1;

    grad[2] = (v1 - v0) * mp_->resolution_inv_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
    grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
    grad[0] *= mp_->resolution_inv_;

    return dist;
  }

  bool SDFMap::isCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target)
  {
    if (isInBox(source) && isInBox(target))
    {
      if (getOccupancy(source) != FREE || getOccupancy(target) != FREE)
      {
        return false;
      }
      else
      {
        collision_check_caster_->input(source, target);
        Eigen::Vector3i start_idx = collision_check_caster_->getStartIdx() + collision_check_caster_->getOffset().cast<int>();
        Eigen::Vector3i end_idx = collision_check_caster_->getEndIdx() + collision_check_caster_->getOffset().cast<int>();
        int maxDist = (start_idx - end_idx).squaredNorm();

        // ROS_ERROR("start_idx:(%d, %d, %d)", start_idx.x(), start_idx.y(), start_idx.z());
        // ROS_ERROR("end_idx:(%d, %d, %d)", end_idx.x(), end_idx.y(), end_idx.z());
        // ROS_ERROR("maxDist: %d", maxDist);

        if (start_idx == end_idx)
        {
          return true;
        }
        else
        {
          Eigen::Vector3i current_idx;
          while (collision_check_caster_->nextId(current_idx))
          {
            if ((current_idx - end_idx).squaredNorm() > maxDist)
            {
              break;
            }
            if (getOccupancy(current_idx) != FREE || !isInBox(current_idx))
            {
              return false;
            }
          }
          return true;
        }
      }
    }
    else
    {
      return false;
    }
  }
} // namespace fast_planner
// SDFMap
