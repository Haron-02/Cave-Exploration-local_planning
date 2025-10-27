#include <path_searching/astar3.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
  Astar3::Astar3()
  {
  }

  Astar3::~Astar3()
  {
    for (int i = 0; i < allocate_num_; i++)
      delete path_node_pool_[i];
  }

  void Astar3::init(ros::NodeHandle &nh, const SDFMap::Ptr &sdf_map)
  {
    nh.param("astar/resolution_astar", resolution_, -1.0);
    nh.param("astar/lambda_heu", lambda_heu_, -1.0);
    nh.param("astar/max_search_time", max_search_time_, -1.0);
    nh.param("astar/allocate_num", allocate_num_, -1);

    tie_breaker_ = 1.0 + 1.0 / 1000;

    this->sdf_map_ = sdf_map;

    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    sdf_map_->getRegion(origin_, map_size_3d_);
    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;

    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new Node3;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
    early_terminate_cost_ = 0.0;
  }

  void Astar3::setResolution(const double &res)
  {
    resolution_ = res;
    this->inv_resolution_ = 1.0 / resolution_;
  }

  int Astar3::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
  {
    Node3Ptr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    posToIndex(start_pt, cur_node->index);
    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

    Eigen::Vector3i end_index;
    posToIndex(end_pt, end_index);

    open_set_.push(cur_node);
    open_set_map_.insert(make_pair(cur_node->index, cur_node));
    use_node_num_ += 1;

    const auto t1 = ros::Time::now();
    
    // if (sdf_map_->getInflateOccupancy(start_pt) == 1 ||
    //   sdf_map_->getOccupancy(start_pt) == SDFMap::UNKNOWN)
    //   ROS_ERROR("[search]: start pt x:%0.2f y:%0.2f z:%0.2f in obstacle", start_pt.x(), start_pt.y(), start_pt.z());

    // if (sdf_map_->getInflateOccupancy(end_pt) == 1 ||
    //   sdf_map_->getOccupancy(end_pt) == SDFMap::UNKNOWN)
    //   ROS_ERROR("[search]: end pt x:%0.2f y:%0.2f z:%0.2f in obstacle", end_pt.x(), end_pt.y(), end_pt.z());

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      cur_node = open_set_.top();
      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
      if (reach_end)
      {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }

      // Early termination if time up
      if ((ros::Time::now() - t1).toSec() > max_search_time_)
      {
        double end_time = (ros::Time::now() - t1).toSec() * 1000.0;
      
        // std::cout << "early";
        early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
        ROS_WARN("[search]: Astar time out");
        ROS_ERROR("(%0.2f: %0.2f: %0.2f)->(%0.2f: %0.2f: %0.2f)", start_pt.x(), start_pt.y(), start_pt.z(),  end_pt.x(), end_pt.y(), end_pt.z());
        ROS_ERROR("spend time:%0.6f, use node num:%d", end_time, use_node_num_);
        return NO_PATH;
      }

      open_set_.pop();
      open_set_map_.erase(cur_node->index);
      close_set_map_.insert(make_pair(cur_node->index, 1));
      iter_num_ += 1;

      Eigen::Vector3d cur_pos = cur_node->position;
      Eigen::Vector3d nbr_pos;
      Eigen::Vector3d step;

      // ROS_WARN("x:%0.2f  y:%0.2f  z:%0.2f", cur_pos.x(), cur_pos.y(), cur_pos.z());
      // ROS_WARN("Find neighbor");
      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
          for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
          {
            step << dx, dy, dz;
            if (step.norm() < 1e-3)
              continue;
            nbr_pos = cur_pos + step;
            // Check safety
            if (!sdf_map_->isInBox(nbr_pos))
              continue;
            if (sdf_map_->getInflateOccupancy(nbr_pos) == 1 ||
                sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN)
              {
                // ROS_WARN("neighbor in obstacle");
                continue;
              }
              

            bool safe = true;
            Vector3d dir = nbr_pos - cur_pos;
            double len = dir.norm();
            dir.normalize();
            // for (double l = 0.1; l < len; l += 0.1)
            // {
            //   Vector3d ckpt = cur_pos + l * dir;
            //   if (sdf_map_->getInflateOccupancy(ckpt) == 1 ||
            //       sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN)
            //   {
            //     safe = false;
            //     break;
            //   }
            // }
            // if (!safe)
            //   continue;

            // Check not in close set
            Eigen::Vector3i nbr_idx;
            posToIndex(nbr_pos, nbr_idx);
            if (close_set_map_.find(nbr_idx) != close_set_map_.end())
              continue;

            Node3Ptr neighbor;
            double tmp_g_score = step.norm() + cur_node->g_score;
            auto node_iter = open_set_map_.find(nbr_idx);
            if (node_iter == open_set_map_.end())
            {
              neighbor = path_node_pool_[use_node_num_];
              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_)
              {
                ROS_WARN("[search]: run out of node pool");
                cout << "run out of node pool." << endl;
                return NO_PATH;
              }
              neighbor->index = nbr_idx;
              neighbor->position = nbr_pos;
            }
            else if (tmp_g_score < node_iter->second->g_score)
            {
              neighbor = node_iter->second;
            }
            else
              continue;

            neighbor->parent = cur_node;
            neighbor->g_score = tmp_g_score;
            neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
            open_set_.push(neighbor);
            open_set_map_[nbr_idx] = neighbor;
          }
    }
    // cout << "open set empty, no path!" << endl;
    // cout << "use node num: " << use_node_num_ << endl;
    // cout << "iter num: " << iter_num_ << endl;
    ROS_ERROR("[search]: open set empty, no path!   iter num:%d", iter_num_);
    return NO_PATH;
  }

  double Astar3::getEarlyTerminateCost()
  {
    return early_terminate_cost_;
  }

  void Astar3::reset()
  {
    open_set_map_.clear();
    close_set_map_.clear();
    path_nodes_.clear();

    std::priority_queue<Node3Ptr, std::vector<Node3Ptr>, NodeComparator1> empty_queue;
    open_set_.swap(empty_queue);
    for (int i = 0; i < use_node_num_; i++)
    {
      path_node_pool_[i]->parent = NULL;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
  }

  double Astar3::pathLength(const vector<Eigen::Vector3d> &path)
  {
    double length = 0.0;
    if (path.size() < 2)
      return length;
    for (int i = 0; i < path.size() - 1; ++i)
      length += (path[i + 1] - path[i]).norm();
    return length;
  }

  void Astar3::backtrack(const Node3Ptr &end_node, const Eigen::Vector3d &end)
  {
    path_nodes_.push_back(end);
    path_nodes_.push_back(end_node->position);
    Node3Ptr cur_node = end_node;
    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node->position);
    }
    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  std::vector<Eigen::Vector3d> Astar3::getPath()
  {
    return path_nodes_;
  }

  double Astar3::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    double h;
    double diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return tie_breaker_ * h;
  }

  double Astar3::getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
  }

  double Astar3::getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  std::vector<Eigen::Vector3d> Astar3::getVisited()
  {
    vector<Eigen::Vector3d> visited;
    for (int i = 0; i < use_node_num_; ++i)
      visited.push_back(path_node_pool_[i]->position);
    return visited;
  }

  void Astar3::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx)
  {
    idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  }

} // namespace fast_planner
