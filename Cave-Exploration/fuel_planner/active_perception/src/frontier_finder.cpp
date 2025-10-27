// 自主探索和前沿检测
#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>


#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigenvalues>

#include <lkh_tsp_solver/lkh_interface.h>
#include <omp.h>

namespace fast_planner
{
  FrontierFinder::FrontierFinder(const SDFMap::Ptr &sdf_map, ros::NodeHandle &nh)
  {
    this->sdf_map_ = sdf_map;
    int voxel_num = sdf_map_->getVoxelNum();
    frontier_flag_ = vector<char>(voxel_num, 0);
    obstacle_flag_ = vector<char>(voxel_num, 0);
    fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

    nh.param("frontier/cluster_min", cluster_min_, -1);
    nh.param("frontier/cluster_size_xyz", cluster_size_xyz_, -1.0);
    nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);

    nh.param("frontier/angle_thres", angle_thres_, -1.0);


    nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
    nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
    nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);

    nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
    nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
    nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
    nh.param("frontier/candidate_theta", candidate_theta_, -1.0);

    nh.param("frontier/down_sample", down_sample_, -1);
    nh.param("frontier/min_visib_num", min_visib_num_, -1);
    nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
    nh.param("frontier/frontier_local_ub", frontier_local_ub_, -1.0);
    nh.param("frontier/frontier_local_lb", frontier_local_lb_, -1.0);
    nh.param("frontier/frontier_global_ub", frontier_global_ub_, -1.0);
    nh.param("frontier/frontier_global_lb", frontier_global_lb_, -1.0);
    nh.param("frontier/sensor_height", sensor_height_, -1.0);
    nh.param("frontier/disable_simple_finder", disable_simple_finder_, true);

    nh.param("frontier/surface_frontier_cluster_min", surface_frontier_cluster_min_, -1);
    nh.param("frontier/surface_frontier_cluster_distance", surface_frontier_cluster_distance_, -1.0);
    nh.param("frontier/obstacle_dist", surface_frontier_obstacle_dist_, 0.0);
    nh.param("frontier/use_normal_vector", use_normal_vector_, false);
    nh.param("frontier/find_path_twice", find_path_twice_, false);
    

    nh.param("frontier/obstacle_cluster_min", obstacle_cluster_min_, -1);

    nh.param("frontier/viewpoint_check", viewpoint_check_, false);
    
    nh.param("exploration/tsp_dir", tsp_dir_, string("null"));
    nh.param("exploration/w_dir", w_dir_, -1.0);

    nh.param("exploration/vm", vm_, -1.0);

    nh.param("exploration/total_sequence_cost", total_sequence_cost_, 8.0);
    nh.param("exploration/single_sequence_cost", single_sequence_cost_, 0.2);

    nh.param("exploration/use_sequence_cost", use_sequence_cost_, false);
    nh.param("exploration/use_velocity_estimate", use_velocity_estimate_, false);
    nh.param("exploration/use_local_refine", use_local_refine_, false);

    std::string start_time, scene;
    nh.param("analysis/start_time", start_time, string("null"));
    nh.param("analysis/scene", scene, string("null"));

    // Initialize TSP par file
    ofstream par_file(tsp_dir_ + "/new_tsp.par");
    par_file << "PROBLEM_FILE = " << tsp_dir_ << "/new_tsp.tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << tsp_dir_ << "/new_tsp.txt\n";
    par_file << "RUNS = 1\n";

    sdf_map_->disableChangeDetection();

    resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);

    thread_num_ = omp_get_max_threads();

    raycaster_.reset(new RayCaster);
    raycaster_->setParams(resolution_, origin);

    raycasters_.resize(thread_num_);
    for(int i = 0; i < thread_num_; i++)
    {
      raycasters_[i].reset(new RayCaster);
      raycasters_[i]->setParams(resolution_, origin);
    }

    Astar_.reset(new Astar3);
    Astar_->init(nh, sdf_map_);

    Astars_.resize(thread_num_);
    for(int i = 0; i < thread_num_; i++)
    {
      Astars_[i].reset(new Astar3);
      Astars_[i]->init(nh, sdf_map_);
    }

    last_surface_frontier_cluster_center_.x() = 5;
    last_surface_frontier_cluster_center_.y() = 5;
    last_surface_frontier_cluster_center_.z() = 2;

    obstacle_cluster_id_ = 0;

    surface_frontier_cluster_id_ = 1;

    viewpoint_cluster_id_ = 1;

    global_planning_update_ = false;

    main_obstacle_id_ = -1;

    percep_utils_.reset(new PerceptionUtils(nh));

    local_occupancy_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    SDFMap_frontier_statistics_pub_ = nh.advertise<active_perception::frontierStatistics>("/topo_planner/SDFMap_frontier_statistics", 1);

    global_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/global_frontier_marker", 1);

    surface_frontier_clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/surface_frontier_clusters_marker", 1);
    merged_clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/merged_clusters_marker", 1);
    local_surface_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/local_surface_frontiers_marker", 1);


    change_cells_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/change_cells_marker", 1);

    surface_frontiers_normals_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/surface_frontiers_normals_maker", 1);

    following_obstacle_cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/following_obstacle_cluster_marker", 1);
    // following_obstacle_cluster_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/following_obstacle_cluster_marker", 1);
    following_surface_frontier_cluster_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/following_surface_frontier_cluster_marker", 1);
    following_surface_frontier_normal_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/following_surface_frontier_normal_marker", 1);

    global_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/global_path_marker", 1);
  
    viewpoints_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/viewpoints_marker", 1);
    max_gain_viewpoint_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/max_gain_viewpoint_marker", 1);

    viewpoint_coverage_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/viewpoint_coverage_marker", 1);
    viewpoint_with_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/viewpoint_with_frontiers_marker", 1);

    viewpoint_clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/viewpoint_clusters_marker", 1);
    viewpoint_cluster_lines_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/viewpoint_cluster_lines_marker", 10);
    
    frontier_clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/frontier_clusters_marker", 1);

    surface_frontier_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/surface_frontier_path_marker", 1);
  
    local_occupancy_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/local_occupancy_cloud_marker", 1);

    // local planning
    local_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/local_frontiers", 10);
    local_viewpoints_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/local_viewpoints", 1);
    local_path_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/local_path", 1);
    local_viewpoints_with_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/local_viewpoints_with_frontiers", 10); 
    local_all_viewpoints_with_frontiers_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/local_all_viewpoints_with_frontiers", 10); 
    // DEBUG
    end_vels_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/end_vels_marker", 1);
    currrent_direction_pub_ = nh.advertise<visualization_msgs::Marker>("/topo_planner/current_direction_marker", 1);
    roadmap_paths_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/roadmap_paths_marker", 1);
    cluster_relation_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("/topo_planner/cluster_relation_marker", 1);
    find_path_failed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/topo_planner/failed_viewpoints_marker", 1);
    // DEBUG

    std::time_t time = static_cast<std::time_t>(std::stoll(start_time));
    std::tm* local_time = std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y_%m_%d-%H_%M_%S");
    start_time =  oss.str();

    string pkg_path = ros::package::getPath("planner");
    string dir_path = pkg_path + "/../../exploration_data_files/" + scene + "/" + start_time;
    file_utils::createDirectory(dir_path);

    // tsp_solve_time_txt_name_ = dir_path + "/tsp_solve_time";
    // file_utils::writeToFileByTrunc(tsp_solve_time_txt_name_, "\t", "start(ms)", "end(ms)", "duration(ms)",
    //                                 "iteration_num", "average(ms)", "nodes_num");
    cost_mat_time_txt_name_ = dir_path + "/cost_mat_time";

    file_utils::writeToFileByTrunc(cost_mat_time_txt_name_, "\t", 
                                  "iter", 
                                  "duration", 
                                  "dimension",
                                  "generate_adj", 
                                  "compute_path",
                                  "compute_end_vel", 
                                  "solve_tsp",
                                  "cur_time");  

    exploration_txt_name_ = dir_path + "/exploration_time";
    file_utils::writeToFileByTrunc(exploration_txt_name_, "\t", 
                                  "total", 
                                  "changedCell", 
                                  "detect_obstacle", 
                                  "detect_global_frontier",
                                  "cluster_frontier",
                                  "sample_viewpoint",
                                  "global_planning",
                                  "viewpoint_refine");  

    update_frontier_cluster_txt_name_ = dir_path + "/update_frontier_cluster_time";
    file_utils::writeToFileByTrunc(update_frontier_cluster_txt_name_, "\t", 
                                  "total", 
                                  "erase_old_cluster",
                                  "calc_normal_vector_num",
                                  "find_new_frontier", 
                                  "frontier_cluster"); 

    start_time_ms_ = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
  }

  FrontierFinder::~FrontierFinder()
  {
  }

  void FrontierFinder::setCurrentPose(const geometry_msgs::Pose &current_pos)
  {
    this->current_pose_ = current_pos;
    this->current_pos_.x() = current_pos.position.x;
    this->current_pos_.y() = current_pos.position.y;
    this->current_pos_.z() = current_pos.position.z;
  }

  void FrontierFinder::setCurrentPose(const Vector3d current_pos)
  {
    this->current_pos_ = current_pos;
  }
  
  void FrontierFinder::setRoadMap(const shared_ptr<preprocess::IkdTreeTopoGraph> &road_map)
  {
    this->road_map_ = road_map;
  }

  void FrontierFinder::simpleSearchFrontiers()
  {
    double t1, t2;
    time_utils::Timer total_time("total_time");
    total_time.Start();

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    // ROS_WARN("----------processChangedcells----------");
    processChangedcells();
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double process_changed_cells_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    // ROS_WARN("----------updateObstacleClusters----------");
    updateObstacleCloud();
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double detect_obstacle_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    // ROS_WARN("----------updateGlobalFrontier----------");
    updateGlobalFrontier(); // too slow
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double detect_global_frontier_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    // ROS_WARN("----------updateSurfaceFrontierCluster----------");
    updateSurfaceFrontierCluster(); // too slow
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double cluster_frontier_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    ROS_WARN("----------sampleSurfaceFrontierViewpoints2----------");
    sampleSurfaceFrontierViewpoints2();
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double sample_viewpoint_time = t2 - t1; 

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    ROS_WARN("----------updateViewpointCluster----------");
    updateViewpointCluster();
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double cluster_viewpoint_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    ROS_WARN("----------global_planning----------");
    global_planning5();
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double global_planning_time = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    ROS_WARN("----------viewpointRefine----------");
    viewpointRefine(); // 暂时关闭了
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double viewpoint_refine_time = t2 - t1;

    // if(use_local_planning_)
    // {
    //   t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    //   local_planning();
    //   t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    //   double local_planning_time = t2 - t1;
    // }

    total_time.Stop(); 
    total_time_ = static_cast<double>(total_time.GetDuration("us")) / 1e3;

    // double detect_frontier_time = process_changed_cells_time
     
    file_utils::writeToFileByAdd(exploration_txt_name_, "\t", 
                                 total_time_, 
                                 process_changed_cells_time, 
                                 detect_obstacle_time, 
                                 detect_global_frontier_time, 
                                 cluster_frontier_time,
                                 sample_viewpoint_time,
                                 global_planning_time);
  }

  void FrontierFinder::processChangedcells()
  {
    changed_cells_.clear();
    sdf_map_->change_mutex_.lock();
    // changed_cells_.insert(sdf_map_->changesBegin(), sdf_map_->changesEnd());
    sdf_map_->resetChangeDetection();
    sdf_map_->change_mutex_.unlock();
    // simple_known_cells_.insert(changed_cells_.begin(), changed_cells_.end());
  }

  void FrontierFinder::updateGlobalFrontier()
  {
    // Search new frontier within box slightly inflated from updated box
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max, true);
    Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
    Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
    Vector3d box_min, box_max;

    sdf_map_->getFrontierBox(box_min, box_max);
    for (int k = 0; k < 3; ++k) 
    {
      search_min[k] = max(search_min[k], box_min[k]);
      search_max[k] = min(search_max[k], box_max[k]);
    }
    Eigen::Vector3i min_id, max_id;
    sdf_map_->posToIndex(search_min, min_id);
    sdf_map_->posToIndex(search_max, max_id);

    #pragma omp parallel                                                                               \
    shared(min_id, max_id)
    {
      vector<int> need_add;
      vector<int> need_erase;

      #pragma omp for nowait

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z) 
          {
            Eigen::Vector3i cur(x, y, z);
            int addr = sdf_map_->toAddress(cur);
            if (knownfree(cur) && isNeighborUnknown(cur)) // 添加新的frontier
              need_add.push_back(addr);
            else if (global_frontier_cells_.find(addr)!=global_frontier_cells_.end())  // 删去不再是frontier的
              need_erase.push_back(addr);
          }

      #pragma omp barrier

      #pragma omp critical
      {
        global_frontier_cells_.insert(need_add.begin(), need_add.end());
        for(auto &id:need_erase)
          global_frontier_cells_.erase(id);
      }
    }
  }

  void FrontierFinder::updateSurfaceFrontierCluster()
  { 
    double t1, t2;
    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    // ROS_WARN("flag size:%d    array size:%d", surface_frontier_flag_.size(), surface_frontier_array_.size());
    // Bounding box of updated region
    Vector3d update_min, update_max, box_min, box_max;
    sdf_map_->getUpdatedBox(update_min, update_max, true);

    sdf_map_->getFrontierBox(box_min, box_max);

    Vector3d search_min = update_min - Vector3d(5, 5, 5);
    Vector3d search_max = update_max + Vector3d(5, 5, 5);

    new_surface_frontiers_.clear();

    for (auto iter = surface_frontier_clusters_.begin(); iter != surface_frontier_clusters_.end();)
    {
      if (haveOverlap(iter->second.box_min_, iter->second.box_max_, search_min, search_max)
       && isSurfaceFrontierClusterChanged(iter->second))
      {
        for(auto cell : iter->second.cells_)
        {
          if(isSurfaceFrontier(cell.first))
            new_surface_frontiers_[cell.first] = cell.second;

          surface_frontier_flag_.erase(cell.first);
          surface_frontier_array_.erase(cell.first);
        }
        // 删除对应的视点,以及视点聚类
        if(viewpoints_.count(iter->first))
        {
          // viewpoint_clusters_.erase(viewpoints_.at(iter->first).viewpoint_cluster_id_);
          // viewpoint_cluster_ids_.push(viewpoints_.at(iter->first).viewpoint_cluster_id_);     
          int cluster_id = viewpoints_.at(iter->first).viewpoint_cluster_id_;
          if(cluster_id!=-1)
          {
            auto cluster = viewpoint_clusters_.at(cluster_id);
            for(auto &cell:cluster.cells_)
            {
              viewpoints_.at(cell).is_clustered_ = false;
              viewpoints_.at(cell).viewpoint_cluster_id_ = -1;
            }
            viewpoint_clusters_.erase(cluster_id);
            viewpoint_cluster_ids_.push(cluster_id);     
          }

          viewpoints_.erase(iter->first);
        }
          
        auto tmp_iter = iter;
        ++iter;
        eraseSurfaceFrontierCluster(tmp_iter->first);
      }
      else
      {
        ++iter;
      }
    }
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double erase_old_frontier = t2 - t1;

    // 寻找新的surface frontier并计算法向量
    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    local_occupancy_cloud_->clear();  
    for (const auto& cloud : sdf_map_->occupancy_cloud_stack_)  // 合并多个点云
      *local_occupancy_cloud_ += *cloud;
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double merge_pointcloud = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;

    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(local_occupancy_cloud_);  // 设置输入点云
    downsample.setLeafSize(0.1f, 0.1f, 0.1f);
    downsample.filter(*local_occupancy_cloud_);
    
    local_occupancy_cloud_kdtree_.setInputCloud(local_occupancy_cloud_);
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double create_kdtree = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    for (int k = 0; k < 3; ++k)
    {
      search_min[k] = max(search_min[k], box_min[k]);
      search_max[k] = min(search_max[k], box_max[k]);
    }
    Eigen::Vector3i min_id, max_id;
    sdf_map_->posToIndex(search_min, min_id);
    sdf_map_->posToIndex(search_max, max_id);

    // DEBUG
    int calc_normal_vector_num = 0;
    // DEBUG
    vector<int> need_compute;

    #pragma omp parallel shared(need_compute, calc_normal_vector_num)
    {
      vector<int> tmp_need_compute;
      int tmp_compute_num = 0;

      #pragma omp for nowait

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z)
          {
            Eigen::Vector3i cur(x, y, z);
            int addr = sdf_map_->toAddress(cur);
            if (!isClusteredFrontier(addr) && isSurfaceFrontier(addr))
            {
              if(new_surface_frontiers_.find(addr)!=new_surface_frontiers_.end())
              {

              }
              else
              {
                tmp_need_compute.push_back(addr);
                tmp_compute_num++;
              }
            }
          }

      #pragma omp barrier

      #pragma omp critical
      {
        need_compute.insert(need_compute.end(), tmp_need_compute.begin(), tmp_need_compute.end());
        calc_normal_vector_num += tmp_compute_num;
      }
    }

    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double find_new_frontier = t2 - t1;

    t1 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    int point_num = 6;
    double radius = 5.0;
    // ROS_WARN("Parallel calc normal vector num:%d", calc_normal_vector_num);
    #pragma omp parallel shared(need_compute, point_num, radius)
    {
      unordered_map<int, Vector3d> tmp_normal_vector;
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;

      #pragma omp for nowait
      for(int i = 0; i < need_compute.size(); i++)
      {
        // 计算法向量
        int addr = need_compute[i];
        Vector3d cell_pos = sdf_map_->AddressToPos(addr);
        tmp_normal_vector[addr] = computeNormalVector(cell_pos, local_occupancy_cloud_kdtree_, normal_estimation, point_num, radius); 
      }

      #pragma omp barrier

      #pragma omp critical
      {
        new_surface_frontiers_.insert(tmp_normal_vector.begin(), tmp_normal_vector.end());
        surface_frontiers_.insert(tmp_normal_vector.begin(), tmp_normal_vector.end());
      }
    }
    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double compute_vector = t2 - t1;

    // splitLargeFrontiers(tmp_frontiers_);
    new_surface_frontier_clusters_.clear();
    for(const auto &frontier:new_surface_frontiers_)
    {
      int addr = frontier.first;
      if(!isClusteredFrontier(addr))
      {
        // expandSurfaceFrontier(addr);
        expandSurfaceFrontier2(addr);
      }
    }

    vector<int> new_cluster_ids;
    new_cluster_ids.insert(new_cluster_ids.end(), new_surface_frontier_clusters_.begin(), new_surface_frontier_clusters_.end());
    splitLargeSurfaceFrontiers(new_cluster_ids);

    t2 = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
    double frontier_cluster = t2 - t1;

    double total_time = erase_old_frontier + find_new_frontier + compute_vector + frontier_cluster;
      
    file_utils::writeToFileByAdd(update_frontier_cluster_txt_name_, "\t", 
                                 total_time, 
                                 erase_old_frontier, 
                                 find_new_frontier,
                                 calc_normal_vector_num,
                                 merge_pointcloud,
                                 create_kdtree,
                                 compute_vector,
                                 frontier_cluster);
  }

  Vector3d FrontierFinder::computeNormalVector(Vector3d pos, 
                                               pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, 
                                               pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation,
                                               int point_num, 
                                               double radius)
  {
    pcl::PointXYZ point(pos.x(), pos.y(), pos.z());
    // 搜索近邻点
    vector<int> indices;
    vector<float> distance;
    local_occupancy_cloud_kdtree_.radiusSearch(point, radius, indices,  distance);
    if(indices.size() > point_num)
    {
      // 使用近邻点计算法向量
      Eigen::Vector4f plane_params;
      float curvature;
      normal_estimation.computePointNormal(*local_occupancy_cloud_, indices, plane_params, curvature);
      // 原始法向量
      Vector3d normal_vector(plane_params[0], plane_params[1], plane_params[2]);
      Vector3d current_to_cell = pos-current_pos_;
      double dot_product = normal_vector.dot(current_to_cell);
      // 如果点积为正，反转法向量
      if (dot_product > 0)
        normal_vector = -normal_vector;      
      normal_vector.normalize();
      return normal_vector;
    }
    else
    {
      // 近邻点数不足，不计算法向量
      Vector3d normal_vector(404, 404, 404);
      return normal_vector;
    }   
  }

  void FrontierFinder::splitLargeSurfaceFrontiers(vector<int> &new_frontier_ids)
  {
    vector<SurfaceFrontierCluster> split_clusters;
    for (auto cluster_id:new_frontier_ids)
    {
      // Check if each frontier needs to be split horizontally
      if(split(surface_frontier_clusters_.at(cluster_id), split_clusters))
      {
        eraseSurfaceFrontierCluster(cluster_id);
        new_surface_frontier_clusters_.erase(cluster_id);
      }
    }
  }

  bool FrontierFinder::split(SurfaceFrontierCluster &cluster, vector<SurfaceFrontierCluster> &split_clusters)
  {
    // Split a frontier into small piece if it is too large;
    auto mean = cluster.average_pos_;
    bool need_split = false;
    for (auto iter : cluster.cells_)
    {
      auto pos = sdf_map_->AddressToPos(iter.first);
      if ((pos - mean).norm() > cluster_size_xyz_)
      {
        need_split = true;
        break;
      }
    }
    if (!need_split)
      return false;

    // Compute principal component
    // Covariance matrix of cells
    Eigen::Matrix3d cov;
    cov.setZero();
    for (auto iter:cluster.cells_)
    {
      auto pos = sdf_map_->AddressToPos(iter.first);
      Eigen::Vector3d diff = pos - mean;
    
      cov += diff * diff.transpose();
    }
    cov /= double(cluster.cells_.size());

    // Find eigenvector corresponds to maximal eigenvector
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int max_idx;
    double max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i)
    {
      if (values[i] > max_eigenvalue)
      {
        max_idx = i;
        max_eigenvalue = values[i];
      }
    }
    Eigen::Vector3d first_pc = vectors.col(max_idx);
    // std::cout << "max idx: " << max_idx << std::endl;
    // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

    // Split the frontier into two groups along the first PC
    SurfaceFrontierCluster ftr1, ftr2;
    for (auto iter:cluster.cells_)
    {
      auto pos = sdf_map_->AddressToPos(iter.first);
      if ((pos - mean).dot(first_pc) >= 0)
        ftr1.cells_[iter.first] = iter.second;
      else
        ftr2.cells_[iter.first] = iter.second;
    }

    computeSurfaceFrontierClusterInfo(ftr1);
    computeSurfaceFrontierClusterInfo(ftr2);

    // Recursive call to split frontier that is still too large
    vector<SurfaceFrontierCluster> split_clusters2;
    if (split(ftr1, split_clusters2))
    {
      split_clusters.insert(split_clusters.end(), split_clusters2.begin(), split_clusters2.end());
      split_clusters2.clear();
    }
    else
    {
      split_clusters.push_back(ftr1);

      int cluster_id = generateSurfaceFrontierClusterId();
      ftr1.id_ = cluster_id;
      for(auto iter:ftr1.cells_)
        surface_frontier_array_[iter.first] = cluster_id;

      surface_frontier_clusters_[cluster_id] = ftr1;
      new_surface_frontier_clusters_.insert(cluster_id);
    }
      

    if (split(ftr2, split_clusters2))
      split_clusters.insert(split_clusters.end(), split_clusters2.begin(), split_clusters2.end());
    else
    {
      split_clusters.push_back(ftr2);

      int cluster_id = generateSurfaceFrontierClusterId();
      ftr2.id_ = cluster_id;
      for(auto iter:ftr2.cells_)
        surface_frontier_array_[iter.first] = cluster_id;

      surface_frontier_clusters_[cluster_id] = ftr2;
      new_surface_frontier_clusters_.insert(cluster_id);
    }

    return true;
  }

  void FrontierFinder::sampleSurfaceFrontierViewpoints()
  {
    surface_frontier_viewpoint_.clear();
    // 将所有 surface frontier cluster 的 cell 合在一起
    vector<int> cluster_ids;
    for(auto iter:surface_frontier_clusters_)
    {
      // 忽略太小的聚类
      if(surface_frontier_clusters_[iter.second.id_].cells_.size() < surface_frontier_cluster_min_)
        continue;
      cluster_ids.push_back(iter.second.id_);
    }

     
    // 遍历所有聚类
    for(auto cluster_id:cluster_ids)
    {
      if(surface_frontier_clusters_[cluster_id].is_sampled_viewpoint_)  // 已经采样过了
        continue;
      
      vector<Vector3d> sampled_frontiers;
      vector<Vector3d> viewpoints;
      for(auto cell:surface_frontier_clusters_[cluster_id].cells_)
      {
        Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);

        bool is_too_close = false;
        for(auto sampled_frontier:sampled_frontiers)
        {
          if((cell_pos-sampled_frontier).norm()<1.8)
            is_too_close = true;
        }
        if(is_too_close)
          continue;

        Vector3d normal_vector = cell.second.normalized();

        // 计算法向量与xy平面夹角
        double z = normal_vector.z();
        double length = normal_vector.norm();
        double pitch = asin(z / length) * (180.0 / M_PI);  
        double yaw = atan2(normal_vector.y(), normal_vector.x()) * (180.0 / M_PI); // -180~180

        Eigen::Vector2d mid_angle(yaw, pitch);
        Eigen::Vector2d angle_range(30, 30);
        if(pitch + angle_range[1]/2 > 0) // 小于雷达俯角
          mid_angle[1] = -angle_range[1]/2;
        else if(pitch - angle_range[1]/2 < -40)
          mid_angle[1] = -40 + angle_range[1]/2;
        
        Eigen::Vector2d min_angle = mid_angle - angle_range/2;
        Eigen::Vector2d max_angle = mid_angle + angle_range/2;

        double step = 15;
        double min_distance = 5.0;
        double max_distance = 7.5;
        double max_free_distance = 0;
        Vector3d max_direction;
        for(int cur_yaw = min_angle[0]; cur_yaw < max_angle[0]+1; cur_yaw+=step)
        {
          for(int cur_pitch = min_angle[1]; cur_pitch < max_angle[1]+1; cur_pitch+=step)
          {
            Vector3d cur_direction;
            cur_direction.x() = cos(normalize_angle(cur_pitch)* M_PI / 180.0) * cos(normalize_angle(cur_yaw)* M_PI / 180.0);
            cur_direction.y() = cos(normalize_angle(cur_pitch)* M_PI / 180.0) * sin(normalize_angle(cur_yaw)* M_PI / 180.0);
            cur_direction.z() = sin(normalize_angle(cur_pitch)* M_PI / 180.0);
            double free_distance = collisionFreeDistance(cell_pos, cur_direction, max_distance);
            if(free_distance > max_distance)
            {
              max_direction = cur_direction;
              max_free_distance = max_distance;
              break;
            }
            else if(free_distance > max_free_distance)
            {
              max_free_distance = free_distance;
              max_direction = cur_direction;
            }
          }
        }

        if(max_free_distance > min_distance)
        {
          double sample_num = 4;
          double sample_step = max(1.0, (max_free_distance-min_distance)/(sample_num-1));

          for (int i = 0; i < sample_num; ++i) 
          {
              double distance = min_distance + i * sample_step;
              Eigen::Vector3d point = cell_pos + distance * max_direction;
              // free的且未在膨胀障碍物中
              if(sdf_map_->getInflateOccupancy(point)!=1 && 
                sdf_map_->getOccupancy(point) == SDFMap::FREE &&
                (current_pos_-point).norm() > 2.0 &&
                point.z() > frontier_global_lb_
                )
              {
                viewpoints.push_back(point);
                surface_frontier_clusters_[cluster_id].viewpoints_.push_back(point);
              }
          }
          sampled_frontiers.push_back(cell_pos);
        }
      }
      surface_frontier_viewpoint_[cluster_id] = viewpoints;
        
        
      if(!surface_frontier_clusters_[cluster_id].viewpoints_.empty())
      {
        int max_cover_num = 0;
        Vector3d max_viewpoint = surface_frontier_clusters_[cluster_id].viewpoints_[0];
        // 遍历所有viewpoint
        for(auto viewpoint:surface_frontier_clusters_[cluster_id].viewpoints_)
        {
          int cover_num = 0;
          for(auto cell:surface_frontier_clusters_[cluster_id].cells_)  // 遍历聚类内所有 frontier
          {
            
            Vector3d frontier_pos = sdf_map_->AddressToPos(cell.first);
            if((frontier_pos - viewpoint).norm() < sdf_map_->getMaxRayLength()
              && sdf_map_->isCollisionFreeStraight(frontier_pos, viewpoint))
            {
              cover_num++;
            }
          }
          if(cover_num > max_cover_num)
          {
            max_cover_num = cover_num;
            max_viewpoint = viewpoint;
          }
        }
        surface_frontier_clusters_[cluster_id].cover_range_ = double(max_cover_num)/double(surface_frontier_clusters_[cluster_id].cells_.size());
        // ROS_WARN("Cluster id:%d  Cover range:%0.2f  Total:%d  Cover:%d", cluster_id, surface_frontier_clusters_[cluster_id].cover_range_, surface_frontier_clusters_[cluster_id].cells_.size(), max_cover_num);
        surface_frontier_clusters_[cluster_id].viewpoints_.clear();
        surface_frontier_clusters_[cluster_id].viewpoints_.push_back(max_viewpoint);
        surface_frontier_viewpoint_[cluster_id].clear();
        surface_frontier_viewpoint_[cluster_id].push_back(max_viewpoint);
      }
    }
  }

  void FrontierFinder::sampleSurfaceFrontierViewpoints2()
  {
    surface_frontier_viewpoint_.clear();
   
    vector<int> cluster_ids;
   
    for(auto &iter:surface_frontier_clusters_)
    {
      auto &cluster = iter.second;
      // 忽略太小的聚类
      if(cluster.cells_.size() < surface_frontier_cluster_min_ || cluster.is_sampled_viewpoint_ || cluster.merged_level_!=0)
        continue;
      cluster_ids.push_back(iter.first);
      // ROS_WARN("Cluster %d  bound size:%0.2f", cluster_id, (cluster.box_max_-cluster.box_min_).norm());
    }

    vector<int> need_erase_viewpoints;
    for(auto &iter:viewpoints_)
    {
      auto &viewpoint = iter.second;
      if(!viewpoint.is_clustered_)
      {
        vector<Vector3d> path;
        if(!isFarFromOccupied(viewpoint.pos_, surface_frontier_obstacle_dist_)||
           sdf_map_->getInflateOccupancy(viewpoint.pos_)==1)
          //  || !getPath(current_pos_, viewpoint.pos_, -1, path))
        {
          ROS_WARN("[sampleViewpoints]:viewpoint in obstacle!");
          need_erase_viewpoints.push_back(viewpoint.id_);
          if(surface_frontier_clusters_.count(viewpoint.id_))
          {
            auto &frontier_cluster = surface_frontier_clusters_.at(viewpoint.id_);
            frontier_cluster.can_be_observed_ = false;
            frontier_cluster.is_sampled_viewpoint_ = false;
          }
        }
      }
    }
    for(auto viewpoint:need_erase_viewpoints)
      viewpoints_.erase(viewpoint);
  

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> u_dist(0.0, 1.0);
    uniform_real_distribution<> phi_dist(0.0, 2 * M_PI);
    // double dis_min = 3.0;
    double dis_min = candidate_rmin_;
    // double dis_max = 10.0;
    double dis_max = candidate_rmax_;
    // double theta_max = 60;
    double theta_max = candidate_theta_;
    double theta_max_rad = theta_max / 180.0 * M_PI;
    int sample_num = 200;
    // // 遍历所有聚类
    for(auto cluster_id:cluster_ids)
    {
      auto & cluster = surface_frontier_clusters_[cluster_id];

      // cluster.viewpoints_.clear();  // 每次都重新采样
      Vector3d average_normal = cluster.average_normal_.normalized();
      // TODO 还是得对采样法向量进行矫正
      // 计算法向量与xy平面夹角
      double z = average_normal.z();
      double length = average_normal.norm();
      double pitch = asin(z / length) * (180.0 / M_PI);  
      double yaw = atan2(average_normal.y(), average_normal.x()) * (180.0 / M_PI); // -180~180

      Eigen::Vector2d angle_range(theta_max, theta_max); // 锥形采样的范围

      if(pitch + angle_range[1]/2 > 45) // 小于雷达俯角
        pitch = -angle_range[1]/2 + 45;
      else if(pitch - angle_range[1]/2 < -45)
        pitch = -45 + angle_range[1]/2;

      average_normal.x() = cos(pitch/180.0*M_PI) * cos(yaw/180.0*M_PI);
      average_normal.y() = cos(pitch/180.0*M_PI) * sin(yaw/180.0*M_PI);
      average_normal.z() = sin(pitch/180.0*M_PI);
      average_normal.normalize();

      for (int i = 0; i < sample_num; ++i) 
      {
          // 使用逆变换抽样法生成均匀分布的距离和角度
          double u = u_dist(gen);
          double v = u_dist(gen);

          double distance = dis_max * std::cbrt(u);  // 确保距离在 [0, radius] 范围内均匀分布
          double theta = std::acos(1 - v * (1 - std::cos(theta_max_rad)));  // 确保角度在 [0, theta_max] 范围内均匀分布
          double phi = phi_dist(gen);  // 均匀分布在 [0, 2π] 范围内

          // Convert polar coordinates to Cartesian coordinates
          double x = distance * sin(theta) * cos(phi);
          double y = distance * sin(theta) * sin(phi);
          double z = distance * cos(theta);

          // Local point in the cone
          Vector3d local_point(x, y, z);

          // Align the local cone direction with the given direction
          Vector3d dir = average_normal;
          Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), dir);

          // Translate and rotate the local point to global coordinates
          Vector3d viewpoint = cluster.average_pos_ + rotation * local_point;

          // free的且未在膨胀障碍物中
          if(sdf_map_->getInflateOccupancy(viewpoint)!=1 
          && sdf_map_->getOccupancy(viewpoint) == SDFMap::FREE
          && (viewpoint-current_pos_).norm() > 3.0
          && isFarFromOccupied(viewpoint, surface_frontier_obstacle_dist_)
          && sdf_map_->isInBox(viewpoint))
            cluster.viewpoints_.push_back(viewpoint);
      }
      
      if(!cluster.viewpoints_.empty())
      {
        double max_gain = -DBL_MAX;
        Vector3d max_gain_viewpoint = cluster.viewpoints_[0];
        int max_cover_num;
        unordered_set<int > max_covered_cells;
        // 遍历所有viewpoint
        for(auto viewpoint:cluster.viewpoints_)
        {
          double gain = 0;
          int cover_num = 0;
          unordered_set<int> covered_cells;
          for(auto cell:cluster.cells_)  // 遍历聚类内所有 frontier
          {
            Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);

            if(isVisible(viewpoint, cell_pos))
            {
              Vector3d cell_to_viewpoint = viewpoint - cell_pos;
              Vector3d normal = cell.second;
              gain += cell_to_viewpoint.dot(normal) / (cell_to_viewpoint.norm() * normal.norm());
              cover_num++;
              covered_cells.insert(cell.first);
            }
          }
          cluster.viewpoint_gains_.push_back(gain);
          // if(gain > max_gain)
          if(gain > max_gain && covered_cells.size() > min_visib_num_)
          {
            max_gain = gain;
            max_gain_viewpoint = viewpoint;
            max_cover_num = cover_num;
            max_covered_cells = covered_cells; 
          }
        }
        cluster.cover_range_ = double(max_cover_num)/double(cluster.cells_.size());
        cluster.max_gain_viewpoint_ = max_gain_viewpoint;
        cluster.max_gain_ = max_gain;
        cluster.covered_cells_ = max_covered_cells;
        // if(max_gain > 2.0)
        if(max_covered_cells.size() > min_visib_num_)
        {
          vector<Vector3d> path;
          if(getPath(current_pos_, max_gain_viewpoint, -1, path) || !viewpoint_check_)
          {
            cluster.can_be_observed_ = true;
            cluster.is_sampled_viewpoint_ = true;
            Viewpoint viewpoint;
            viewpoint.pos_ = max_gain_viewpoint;
            viewpoint.gain_ = max_gain;
            viewpoint.cover_num_ = max_cover_num;
            viewpoint.frontier_cluster_id_ = cluster.id_;
            viewpoint.id_ = cluster.id_;
            viewpoint.covered_cells_ = max_covered_cells;
            viewpoints_[cluster.id_] = viewpoint;
          }
          else
            ROS_WARN("[sampleViewpoint]:Find viewpoint path failed, id:%d", cluster.id_);
        }
        // ROS_WARN("Cluster id:%d  Cover range:%0.2f  Total:%d  Cover:%d  Gain:%0.3f", cluster_id, cluster.cover_range_, cluster.cells_.size(), max_cover_num, cluster.max_gain_);
      }
    }
    
    // 球形采样
    // 遍历所有聚类
    dis_min = candidate_rmin_;
    // double dis_max = 10.0;
    dis_max = candidate_rmax_;
    // dis_min = 3.0;
    // dis_max = 10.0;
    theta_max = 42.0;
    theta_max_rad = theta_max / 180.0 * M_PI;
    sample_num = 400;
    

    for(auto cluster_id:cluster_ids)
    {
      auto & cluster = surface_frontier_clusters_[cluster_id];

      cluster.is_sampled_viewpoint_ = true;
      // ROS_WARN("Cluster id:%d  ball sample", cluster.id_);
      for (int i = 0; i < sample_num; ++i) 
      {
          // 使用逆变换抽样法生成均匀分布的距离和角度
          double u = u_dist(gen);
          double v = u_dist(gen);

          double distance = std::cbrt(u * pow(dis_max, 3));  // 确保距离在 [0, radius] 范围内均匀分布
          double theta = v * theta_max_rad * 2 - theta_max_rad;  // 确保角度在 [0, theta_max] 范围内均匀分布
          double phi = phi_dist(gen);  // 均匀分布在 [0, 2π] 范围内

          // Convert polar coordinates to Cartesian coordinates
          double x = distance * cos(theta) * cos(phi);
          double y = distance * cos(theta) * sin(phi);
          double z = distance * sin(theta);

          // Local point in the cone
          Vector3d local_point(x, y, z);

          // Translate and rotate the local point to global coordinates
          Vector3d viewpoint = cluster.average_pos_ + local_point;

          // free的且未在膨胀障碍物中
          if(sdf_map_->getInflateOccupancy(viewpoint)!=1 
          && sdf_map_->getOccupancy(viewpoint) == SDFMap::FREE
          && (viewpoint-current_pos_).norm() > 3.0
          && isFarFromOccupied(viewpoint, surface_frontier_obstacle_dist_)
          && sdf_map_->isInBox(viewpoint))
            cluster.viewpoints_.push_back(viewpoint);

          // ROS_WARN("Point x:%0.1f  y:%0.1f  z:%0.1f", global_point[0], global_point[1], global_point[2]);
      }
      
      if(!cluster.viewpoints_.empty())
      {
        
        Vector3d max_gain_viewpoint;
        double max_gain = -DBL_MAX;
        int max_cover_num = -INT32_MAX;
        unordered_set<int > max_covered_cells;
        for(auto viewpoint:cluster.viewpoints_)
        {
          double gain = 0;
          int cover_num = 0;
          unordered_set<int> covered_cells;
          for(auto cell:cluster.cells_)  // 遍历聚类内所有 frontier
          {
            Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);

            if(isVisible(viewpoint, cell_pos))
            // if((cell_pos - viewpoint).norm() < sdf_map_->getMaxRayLength()
            //   && sdf_map_->isCollisionFreeStraight(cell_pos, viewpoint)) 
            {
              Vector3d cell_to_viewpoint = viewpoint - cell_pos;
              Vector3d normal = cell.second;
              gain += cell_to_viewpoint.dot(normal) / (cell_to_viewpoint.norm() * normal.norm());
              cover_num++;
              covered_cells.insert(cell.first);
            }
          }
          cluster.viewpoint_gains_.push_back(gain);
          // if(gain > max_gain)
          if(gain > max_gain && covered_cells.size() > min_visib_num_)
          {
            max_gain = gain;
            max_gain_viewpoint = viewpoint;
            max_cover_num = cover_num;
            max_covered_cells = covered_cells; 
          }
        }
        cluster.cover_range_ = double(max_cover_num)/double(cluster.cells_.size());
        cluster.max_gain_viewpoint_ = max_gain_viewpoint;
        cluster.max_gain_ = max_gain;
        cluster.covered_cells_ = max_covered_cells;
        // if(max_gain > 2.0)
        if(max_covered_cells.size() > min_visib_num_)
        {
          vector<Vector3d> path;
          if(getPath(current_pos_, max_gain_viewpoint, -1, path) || !viewpoint_check_)
          {
            cluster.can_be_observed_ = true;

            Viewpoint viewpoint;
            viewpoint.pos_ = max_gain_viewpoint;
            viewpoint.gain_ = max_gain;
            viewpoint.cover_num_ = max_cover_num;
            viewpoint.frontier_cluster_id_ = cluster.id_;
            viewpoint.id_ = cluster.id_;
            viewpoints_[cluster.id_] = viewpoint;
          }
          else
            ROS_WARN("[sampleViewpoint]:Find viewpoint path failed, id:%d", cluster.id_);
        }
      }
    }
    for(auto &viewpoint:viewpoints_)
    {
      if(surface_frontier_clusters_.find(viewpoint.second.id_) == surface_frontier_clusters_.end())
        ROS_WARN("[sample viewpoint]:frontier id, viewpoint id not match");
    }
  }

  bool FrontierFinder::isFarFromOccupied(Eigen::Vector3d pos, double dist)
  {
    int test = ceil(dist/sdf_map_->getResolution());

    int step = ceil(dist/sdf_map_->getResolution());

    Vector3i ind;
    sdf_map_->posToIndex(pos, ind);

    Vector3i check_ind;
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          //膨胀点
          check_ind(0) = ind(0) + x;
          check_ind(1) = ind(1) + y;
          check_ind(2) = ind(2) + z;

          if(sdf_map_->getInflateOccupancy(check_ind) == 1)
            return false;
          if(sdf_map_->getOccupancy(ind) == SDFMap::UNKNOWN)
            return false;
        }
    return true;
  }

  void FrontierFinder::sampleSurfaceFrontierViewpoints(SurfaceFrontierCluster &cluster)
  {   
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> u_dist(0.0, 1.0);
    uniform_real_distribution<> phi_dist(0.0, 2 * M_PI);
    double dis_min = 3.0;
    double dis_max = 10.0;
    double theta_max = 60;
    double theta_max_rad = theta_max / 180.0 * M_PI;
    int sample_num = 80;

    Vector3d average_normal = cluster.average_normal_.normalized();
    // TODO 还是得对采样法向量进行矫正
    // 计算法向量与xy平面夹角
    double z = average_normal.z();
    double length = average_normal.norm();
    double pitch = asin(z / length) * (180.0 / M_PI);  
    double yaw = atan2(average_normal.y(), average_normal.x()) * (180.0 / M_PI); // -180~180

    Eigen::Vector2d angle_range(theta_max, theta_max); // 锥形采样的范围

    if(pitch + angle_range[1]/2 > 45) // 小于雷达俯角
      pitch = -angle_range[1]/2 + 45;
    else if(pitch - angle_range[1]/2 < -45)
      pitch = -45 + angle_range[1]/2;

    average_normal.x() = cos(pitch/180.0*M_PI) * cos(yaw/180.0*M_PI);
    average_normal.y() = cos(pitch/180.0*M_PI) * sin(yaw/180.0*M_PI);
    average_normal.z() = sin(pitch/180.0*M_PI);
    average_normal.normalize();

    for (int i = 0; i < sample_num; ++i) 
    {
      // 使用逆变换抽样法生成均匀分布的距离和角度
      double u = u_dist(gen);
      double v = u_dist(gen);

      double distance = dis_max * std::cbrt(u);  // 确保距离在 [0, radius] 范围内均匀分布
      double theta = std::acos(1 - v * (1 - std::cos(theta_max_rad)));  // 确保角度在 [0, theta_max] 范围内均匀分布
      double phi = phi_dist(gen);  // 均匀分布在 [0, 2π] 范围内
      // Convert polar coordinates to Cartesian coordinates
      double x = distance * sin(theta) * cos(phi);
      double y = distance * sin(theta) * sin(phi);
      double z = distance * cos(theta);
      // Local point in the cone
      Vector3d local_point(x, y, z);
      // Align the local cone direction with the given direction
      Vector3d dir = average_normal;
      Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), dir);
      // Translate and rotate the local point to global coordinates
      Vector3d viewpoint = cluster.average_pos_ + rotation * local_point;
      // free的且未在膨胀障碍物中
      if(sdf_map_->getInflateOccupancy(viewpoint)!=1 
      && sdf_map_->getOccupancy(viewpoint) == SDFMap::FREE
      && (viewpoint-current_pos_).norm() > 3.0)
        cluster.viewpoints_.push_back(viewpoint);
    }

    // 球形采样
    dis_min = 3.0;
    dis_max = 10.0;
    theta_max = 37.0;
    theta_max_rad = theta_max / 180.0 * M_PI;
    sample_num = 400;
    
    for (int i = 0; i < sample_num; ++i) 
    {
      // 使用逆变换抽样法生成均匀分布的距离和角度
      double u = u_dist(gen);
      double v = u_dist(gen);

      double distance = std::cbrt(u * pow(dis_max, 3));  // 确保距离在 [0, radius] 范围内均匀分布
      double theta = v * theta_max_rad * 2 - theta_max_rad;  // 确保角度在 [0, theta_max] 范围内均匀分布
      double phi = phi_dist(gen);  // 均匀分布在 [0, 2π] 范围内
      // Convert polar coordinates to Cartesian coordinates
      double x = distance * cos(theta) * cos(phi);
      double y = distance * cos(theta) * sin(phi);
      double z = distance * sin(theta);
      // Local point in the cone
      Vector3d local_point(x, y, z);
      // Translate and rotate the local point to global coordinates
      Vector3d viewpoint = cluster.average_pos_ + local_point;

      // free的且未在膨胀障碍物中
      if(sdf_map_->getInflateOccupancy(viewpoint)!=1 
      && sdf_map_->getOccupancy(viewpoint) == SDFMap::FREE)
        cluster.viewpoints_.push_back(viewpoint);
    }
    cluster.is_sampled_viewpoint_ = true;
      
    if(!cluster.viewpoints_.empty())
    {
      double max_gain = -DBL_MAX;
      Vector3d max_gain_viewpoint = cluster.viewpoints_[0];
      int max_cover_num;
      unordered_set<int > max_covered_cells;
      // 遍历所有viewpoint
      for(auto viewpoint:cluster.viewpoints_)
      {
        double gain = 0;
        int cover_num = 0;
        unordered_set<int> covered_cells;
        for(auto cell:cluster.cells_)  // 遍历聚类内所有 frontier
        {
          Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);

          if(isVisible(viewpoint, cell_pos))
          // if((cell_pos - viewpoint).norm() < sdf_map_->getMaxRayLength()
          //   && sdf_map_->isCollisionFreeStraight(cell_pos, viewpoint)) 
          {
            Vector3d cell_to_viewpoint = viewpoint - cell_pos;
            Vector3d normal;
            if(cell.second.x()==404)
              normal = cluster.average_normal_;
            else
              normal = cell.second;

            gain += cell_to_viewpoint.dot(normal) / (cell_to_viewpoint.norm() * normal.norm());
            cover_num++;
            covered_cells.insert(cell.first);
          }
        }
        cluster.viewpoint_gains_.push_back(gain);
        if(gain > max_gain)
        {
          max_gain = gain;
          max_gain_viewpoint = viewpoint;
          max_cover_num = cover_num;
          max_covered_cells = covered_cells; 
        }
      }
      cluster.cover_range_ = double(max_cover_num)/double(cluster.cells_.size());
      cluster.max_gain_viewpoint_ = max_gain_viewpoint;
      cluster.max_gain_ = max_gain;
      cluster.covered_cells_ = max_covered_cells;
      if(max_gain > 2.0)
      {
        cluster.can_be_observed_ = true;
        // ROS_WARN("Cluster id:%d  can be observed", cluster.id_);
      }
    }
  }

  double FrontierFinder::normalize_angle(double angle) 
  {
    while (angle < -180) angle += 360;
    while (angle >= 180) angle -= 360;
    return angle;
  }

  /*
  void FrontierFinder::sampleSurfaceFrontierViewpoints()
  {
    // 将所有 surface frontier cluster 的 cell 合在一起
    unordered_map<int, Vector3d> cells;
    for(auto cluster_id:obstacle_clusters_[main_obstacle_id_].surface_frontier_ids_)
    {
      // 忽略太小的聚类
      if(surface_frontier_clusters_[cluster_id].cells_.size() < surface_frontier_cluster_min_)
        continue;

      auto & cluster_cells = surface_frontier_clusters_[cluster_id].cells_;
      cells.insert(cluster_cells.begin(), cluster_cells.end());
    }
    // 将当前obstacle聚类对应的 surface_frontier 构建为点云 
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_frontier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    surface_frontiers_.clear();
    for(auto cell:cells)
    {
      if(cell.second.x() == 404)  // 未计算法向量的点
        continue;
      // if(covered_surface_frontier_.find(cell.first)!=covered_surface_frontier_.end()) // 已经被vp覆盖的frontier
      //   continue;

      Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);
      pcl::PointXYZ point(cell_pos.x(), cell_pos.y(), cell_pos.z());
      surface_frontier_cloud->points.push_back(point);
      surface_frontiers_.push_back(cell.first);
    }
    surface_frontier_cloud->height = 1;
    surface_frontier_cloud->width = surface_frontier_cloud->points.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(surface_frontier_cloud);
    uniform_sampling.setRadiusSearch(2);  // 设置采样半径
    uniform_sampling.filter(*downsampled_cloud);

    ROS_WARN("downsampled_cloud.size:%d", downsampled_cloud->points.size());

    downsampled_surface_frontier_.clear();
    for(auto point:downsampled_cloud->points)
    {
      Vector3d pos(point.x, point.y, point.z);
      Vector3i ind;
      sdf_map_->posToIndex(pos, ind);
      int addr = sdf_map_->toAddress(ind);
      
      if(cells.find(addr)!=cells.end())
      {
        Vector3d normal = cells[addr];
        downsampled_surface_frontier_[addr] = normal;
      }
    }

    surface_frontier_viewpoint_.clear();
    for(auto frontier:downsampled_surface_frontier_)
    {
      Vector3d cell_pos = sdf_map_->AddressToPos(frontier.first);
      vector<Vector3d> points;
      Vector3d normal = frontier.second;

      // 计算法向量与xy平面夹角
      double z = -normal.z();
      double length = normal.norm();
      double angle_deg = asin(z / length) * (180.0 / M_PI);  

      if(angle_deg < -10.0) // 小于雷达俯角
      { 
        // 将采样角度改为雷达俯角，避免生成无意义的vp 
        double xy_distance = sqrt(normal.x()*normal.x() + normal.y()*normal.y());
        normal.z() = tan(10.0 / 180.0 * M_PI) * xy_distance;
        normal.normalize();
      }
        
      
      double min_distance = 5.0;
      double max_distance = 10.0;
      int point_num = 3;
      double step = (max_distance - min_distance) / point_num;

      Vector3d current_pos(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
      for (int i = 0; i < point_num; ++i) 
      {
          double distance = min_distance + i * step;
          Eigen::Vector3d point = cell_pos + distance * normal;
          // free的且未在膨胀障碍物中
          if(sdf_map_->getInflateOccupancy(point)!=1 && 
             sdf_map_->getOccupancy(point) == SDFMap::FREE &&
             sdf_map_->isCollisionFreeStraight(point, cell_pos) &&
             (current_pos-point).norm() > 2.0 &&
             point.z() > frontier_global_lb_
             ) 
            points.push_back(point);
      }
      surface_frontier_viewpoint_[frontier.first] = points;
    }

  }
  */

  double FrontierFinder::collisionFreeDistance(const Vector3d &point, const Vector3d &direction, double check_distance)
  {
    check_distance += 0.3;
    Vector3d target = point + direction * check_distance;
    auto &collision_check_caster = sdf_map_->collision_check_caster_;
    collision_check_caster->input(point, target);
    // ROS_WARN("target x:%f  y:%f  z:%f", target.x(), target.y(), target.z());
    // ROS_WARN("point x:%f  y:%f  z:%f", point.x(), point.y(), point.z());
    Vector3i start_idx = collision_check_caster->getStartIdx() + collision_check_caster->getOffset().cast<int>();
    Vector3i end_idx = collision_check_caster->getEndIdx() + collision_check_caster->getOffset().cast<int>();
    int maxDist = (start_idx - end_idx).squaredNorm();

        // ROS_ERROR("start_idx:(%d, %d, %d)", start_idx.x(), start_idx.y(), start_idx.z());
        // ROS_ERROR("end_idx:(%d, %d, %d)", end_idx.x(), end_idx.y(), end_idx.z());
        // ROS_ERROR("maxDist: %d", maxDist);

    if (start_idx == end_idx)
    {
      return 0;
    }
    else
    {
      Vector3i current_idx;
      Vector3i last_idx = current_idx;
      while (collision_check_caster->nextId(current_idx))
      {
        if ((current_idx - end_idx).squaredNorm() > maxDist)
        {
          break;
        }
        if (sdf_map_->getOccupancy(current_idx) != SDFMap::FREE || !sdf_map_->isInBox(current_idx))
        {
          Vector3d last_pos;
          sdf_map_->indexToPos(last_idx, last_pos);
          double max_distance = (last_pos-point).norm();
          return max_distance;
        }
        last_idx = current_idx;
      }
    
      return check_distance;
    }
  }

  void FrontierFinder::mergeCluster()
  {
    ros::Time start_time = ros::Time::now();

    queue<int> seed_queue;
    unordered_set<int> close;

    // merged_surface_frontier_clusters_ = surface_frontier_clusters_;

    vector<double> dists = {25.0, 30.0, 35.0, 40.0};
    // vector<double> dists = {40.0, 50.0, 60.0};
    for(int i = 0; i < dists.size(); i++)
    {
      close.clear();
      for(auto &iter:surface_frontier_clusters_)
      {
        seed_queue.push(iter.first);
        while(!seed_queue.empty())
        {
          int cur = seed_queue.front();
          seed_queue.pop();

          auto &cur_cluster = surface_frontier_clusters_[cur];
          // Vector3d &viewpoint = cur_cluster.max_gain_viewpoint_;
          if(// cur_cluster.can_be_observed_ && 
            cur_cluster.merged_level_ < (i+1) && 
            distToBox(cur_cluster.box_min_, cur_cluster.box_max_, current_pos_) > dists[i] &&
            // (current_pos_-cur_cluster.average_pos_).norm() > dists[i] &&
            !cur_cluster.is_merged_ &&
            !close.count(cur))
          {
            close.insert(cur);
            mergeFrontierCluster(cur, seed_queue, i+1, dists[i]);
          }
        }
      }
    }
 
    int before_merge = 0;
    int after_merge = 0;

    for(auto &cluster:surface_frontier_clusters_)
    {
      if(cluster.second.merged_level_ == 0 && cluster.second.can_be_observed_)
        before_merge++;
    }

    for(auto &cluster:surface_frontier_clusters_)
    {
      if(!cluster.second.is_merged_ && cluster.second.can_be_observed_)
        after_merge++;
    }


    ROS_WARN("Original cluster num:%d  After merge cluster num:%d", before_merge, after_merge);
  }

  void FrontierFinder::mergeFrontierCluster(int seed, queue<int> &seed_queue, int merge_level, double dist)
  {
    ROS_WARN("Seed:%d", seed);
    auto &current_cluster = surface_frontier_clusters_[seed];

    int smallest_cluster_id = -1;
    int smallest_num = INT16_MAX;
    for(auto cluster_id:current_cluster.neighbor_cluster_ids_)
    {
      auto &neighbor_cluster = surface_frontier_clusters_[cluster_id];

      ROS_WARN("Neighbor:%d    is_merged:%d    merge_level:%d    size:%d    can_be_observed:%d", 
                cluster_id, neighbor_cluster.is_merged_, neighbor_cluster.merged_level_, neighbor_cluster.cells_.size(), neighbor_cluster.can_be_observed_);

      if(// neighbor_cluster.can_be_observed_ && 
          distToBox(neighbor_cluster.box_min_, neighbor_cluster.box_max_, current_pos_) > dist &&
          neighbor_cluster.cells_.size() < smallest_num &&
          neighbor_cluster.obstacle_cluster_id_ == current_cluster.obstacle_cluster_id_ &&
          neighbor_cluster.merged_level_ < merge_level &&
          !neighbor_cluster.is_merged_)
      {
        smallest_num = surface_frontier_clusters_[cluster_id].cells_.size();
        smallest_cluster_id = cluster_id;
      }
    }

    ROS_WARN("Smallest id:%d", smallest_cluster_id);
    
    if(smallest_cluster_id!=-1)
    {
      auto &neighbor_cluster = surface_frontier_clusters_[smallest_cluster_id];

      SurfaceFrontierCluster new_cluster;
      // 设置id
      new_cluster.id_ = generateSurfaceFrontierClusterId();
      // 设置障碍物聚类
      new_cluster.obstacle_cluster_id_ = current_cluster.obstacle_cluster_id_;
      // 设置邻居聚类
      new_cluster.neighbor_cluster_ids_.insert(
        current_cluster.neighbor_cluster_ids_.begin(), 
        current_cluster.neighbor_cluster_ids_.end());
      
      new_cluster.neighbor_cluster_ids_.insert(
        neighbor_cluster.neighbor_cluster_ids_.begin(), 
        neighbor_cluster.neighbor_cluster_ids_.end());

      new_cluster.neighbor_cluster_ids_.erase(current_cluster.id_);
      new_cluster.neighbor_cluster_ids_.erase(neighbor_cluster.id_);

      new_cluster.merged_cluster_ids_.push_back(current_cluster.id_);
      new_cluster.merged_cluster_ids_.push_back(neighbor_cluster.id_);

      for(auto &iter:new_cluster.neighbor_cluster_ids_)
      {
        auto &new_nbr = surface_frontier_clusters_[iter];
        // new_nbr.neighbor_cluster_ids_.erase(current_cluster.id_);
        // new_nbr.neighbor_cluster_ids_.erase(neighbor_cluster.id_);
        new_nbr.neighbor_cluster_ids_.insert(new_cluster.id_);
      }
      // 设置box
      for(int i = 0; i < 3;i++)
      {
        new_cluster.box_min_[i] = min(current_cluster.box_min_[i], neighbor_cluster.box_min_[i]);
        new_cluster.box_max_[i] = max(current_cluster.box_max_[i], neighbor_cluster.box_max_[i]);
      }
      // 设置代表视点
      if(current_cluster.can_be_observed_ && neighbor_cluster.can_be_observed_)
      {
        new_cluster.can_be_observed_ = true;
        Vector3d viewpoint_1 = current_cluster.max_gain_viewpoint_;
        Vector3d viewpoint_2 = neighbor_cluster.max_gain_viewpoint_;
        Vector3d mid_pos = (viewpoint_1 + viewpoint_2) / 2;
        int steps = (viewpoint_2 - mid_pos).norm() / sdf_map_->getResolution();

        for(int i = 0; i <= steps; i++)
        {
          Vector3d inter_pos;
          inter_pos = mid_pos + (viewpoint_2 - mid_pos) * i / steps;
          // free的且未在膨胀障碍物中
          if(sdf_map_->getInflateOccupancy(inter_pos)!=1 && 
            sdf_map_->getOccupancy(inter_pos) == SDFMap::FREE)
          {
            new_cluster.max_gain_viewpoint_ = inter_pos;
            break;
          }
          inter_pos = mid_pos - (mid_pos - viewpoint_1) * i / steps;
          // free的且未在膨胀障碍物中
          if(sdf_map_->getInflateOccupancy(inter_pos)!=1 && 
            sdf_map_->getOccupancy(inter_pos) == SDFMap::FREE)
          {
            new_cluster.max_gain_viewpoint_ = inter_pos;
            break;
          }
        }
      }
      else if(current_cluster.can_be_observed_)
      {
        new_cluster.can_be_observed_ = true;
        new_cluster.max_gain_viewpoint_ = current_cluster.max_gain_viewpoint_;
      }
      else if(neighbor_cluster.can_be_observed_)
      {
        new_cluster.can_be_observed_ = true;
        new_cluster.max_gain_viewpoint_ = neighbor_cluster.max_gain_viewpoint_;
      }
      else
      {
        new_cluster.can_be_observed_ = false;
      }
     
      // double dist_1 = (current_cluster.max_gain_viewpoint_ - new_cluster.average_pos_).norm();
      // double dist_2 = (neighbor_cluster.max_gain_viewpoint_ - new_cluster.average_pos_).norm();
      // if(dist_1 < dist_2)
      //   new_cluster.max_gain_viewpoint_ = current_cluster.max_gain_viewpoint_;
      // else
      //   new_cluster.max_gain_viewpoint_ = neighbor_cluster.max_gain_viewpoint_;

      // 添加cell
      for(auto &cell:current_cluster.cells_)
        new_cluster.cells_[cell.first] = cell.second;
      for(auto &cell:neighbor_cluster.cells_)
        new_cluster.cells_[cell.first] = cell.second;
      // 设置中心位置
      new_cluster.average_pos_ = current_cluster.average_pos_ * current_cluster.cells_.size() / new_cluster.cells_.size()
                               + neighbor_cluster.average_pos_ * neighbor_cluster.cells_.size() / new_cluster.cells_.size();

      new_cluster.is_sampled_viewpoint_ = true;
      
      new_cluster.merged_level_ = max(current_cluster.merged_level_, neighbor_cluster.merged_level_) + 1;

      current_cluster.is_merged_ = true;
      neighbor_cluster.is_merged_ = true;
      
      surface_frontier_clusters_[new_cluster.id_] = new_cluster;

      for(auto &nbr:new_cluster.neighbor_cluster_ids_)
        seed_queue.push(nbr);
    }
  }

  void FrontierFinder::unmergeCluster()
  {

    // vector<double> dists = {30.0, 40.0, 50.0, 60.0};
    vector<double> dists = {25.0, 30.0, 35.0, 40.0};

    for(int i = 0; i < dists.size(); i++)
    {
      vector<int> need_erase;
      for(auto &iter:surface_frontier_clusters_)
      {
        auto &cur = iter.second;
        
        if(cur.merged_level_ == (i+1) &&
           !cur.is_merged_)
        {
          if(distToBox(cur.box_min_, cur.box_max_, current_pos_) < dists[i])
          {
            auto &child_1 = surface_frontier_clusters_[cur.merged_cluster_ids_[0]];
            auto &child_2 = surface_frontier_clusters_[cur.merged_cluster_ids_[1]];

            child_1.is_merged_ = false;
            child_2.is_merged_ = false;

            for(auto &nbr:cur.neighbor_cluster_ids_)
              surface_frontier_clusters_[nbr].neighbor_cluster_ids_.erase(cur.id_);

            need_erase.push_back(cur.id_);
          }
        }
      }

      for(auto &id:need_erase)
      {
        surface_frontier_clusters_.erase(id);
        surface_frontier_cluster_ids_.push(id);
      }
    }
  }

  // 基于历史TSP的聚类
  void FrontierFinder::mergeViewpointCluster()
  {
    ros::Time start_time = ros::Time::now();

    queue<int> seed_queue;
    unordered_set<int> close;

    // merged_surface_frontier_clusters_ = surface_frontier_clusters_;

    vector<double> dists = {25.0, 35.0};
    // vector<double> dists = {40.0, 50.0, 60.0};
    for(int i = 0; i < dists.size(); i++)
    {
      close.clear();
      for(auto &iter:viewpoint_clusters_)
      {
        seed_queue.push(iter.first);
        while(!seed_queue.empty())
        {
          int cur = seed_queue.front();
          seed_queue.pop();

          auto &cur_cluster = viewpoint_clusters_[cur];
    
          double distance = 100000;
          for(auto &vertex_cell:cur_cluster.vertex_cells_)
          {
            distance = min(distance, (viewpoints_[vertex_cell].pos_-current_pos_).norm());
          }
          // Vector3d &viewpoint = cur_cluster.max_gain_viewpoint_;
          if(cur_cluster.merged_level_ < (i+1) && 
             distance > dists[i] &&
             !cur_cluster.is_merged_ &&
             !close.count(cur))
          {
            close.insert(cur);
            viewpointCluster(cur, seed_queue, i+1, dists[i]);
          }
        }
      }
    }
  }

  // 基于历史TSP的聚类
  void FrontierFinder::viewpointCluster(int seed, queue<int> &seed_queue, int merge_level, double dist)
  {
    ROS_WARN("Seed:%d", seed);
    auto &cur_cluster = viewpoint_clusters_[seed];

    int smallest_direction = -1;
    int smallest_num = INT16_MAX;

    if(cur_cluster.forward_target_!=-1)
    {
      auto &forward_viewpoint_id = cur_cluster.forward_target_;
      auto &nbr_cluster = viewpoint_clusters_[viewpoints_[forward_viewpoint_id].viewpoint_cluster_id_];
      
      double distance = 100000;
      for(auto &vertex_cell:nbr_cluster.vertex_cells_)
      {
        distance = min(distance, (viewpoints_[vertex_cell].pos_-current_pos_).norm());
      }

      if(distance > dist &&
         nbr_cluster.merged_level_ < merge_level &&
         !nbr_cluster.is_merged_)
      {
        smallest_num = nbr_cluster.ordered_cells_.size();
        smallest_direction = 1;
      }
    }

    if(cur_cluster.backward_target_!=-1)
    {
      auto &backward_viewpoint_id = cur_cluster.backward_target_;
      auto &nbr_cluster = viewpoint_clusters_[viewpoints_[backward_viewpoint_id].viewpoint_cluster_id_];
      
      double distance = 100000;
      for(auto &vertex_cell:nbr_cluster.vertex_cells_)
      {
        distance = min(distance, (viewpoints_[vertex_cell].pos_-current_pos_).norm());
      }

      if(distance > dist &&
         nbr_cluster.merged_level_ < merge_level &&
         !nbr_cluster.is_merged_)
      {
        if(nbr_cluster.ordered_cells_.size() < smallest_num)
        {
          smallest_num = nbr_cluster.ordered_cells_.size();
          smallest_direction = 0;
        }
      }
    }

    if(smallest_direction == 0)
    {
      auto &nbr_cluster = viewpoint_clusters_[viewpoints_[cur_cluster.backward_target_].viewpoint_cluster_id_];
      ViewpointCluster viewpoint_cluster;

      int new_cluster_id = generateViewpointClusterId();
      viewpoint_cluster.id_ = new_cluster_id;

      vector<int> forward_cells;
      for(auto &cell:cur_cluster.ordered_cells_)
      {
        forward_cells.push_back(cell);
        viewpoints_[cell].viewpoint_cluster_id_ = new_cluster_id;
        if(forward_cells.back()!=cur_cluster.backward_source_)
          reverse(forward_cells.begin(), forward_cells.end());
      }

      vector<int> backward_cells;
      for(auto &cell:nbr_cluster.ordered_cells_)
      {
        backward_cells.push_back(cell);
        viewpoints_[cell].viewpoint_cluster_id_ = new_cluster_id;
        if(backward_cells.front()!=cur_cluster.backward_target_)
          reverse(backward_cells.begin(), backward_cells.end());
      }
      viewpoint_cluster.ordered_cells_.insert(viewpoint_cluster.ordered_cells_.end(), 
                                              forward_cells.begin(), forward_cells.end());

      viewpoint_cluster.ordered_cells_.insert(viewpoint_cluster.ordered_cells_.end(), 
                                              backward_cells.begin(), backward_cells.end());

      Vector3d total_pos(0, 0, 0);
      for(auto &cell:viewpoint_cluster.ordered_cells_)
        total_pos+=viewpoints_[cell].pos_;
      viewpoint_cluster.avg_pos_=total_pos/viewpoint_cluster.ordered_cells_.size();

      viewpoint_cluster.vertex_cells_.push_back(viewpoint_cluster.ordered_cells_.front());
      viewpoint_cluster.vertex_cells_.push_back(viewpoint_cluster.ordered_cells_.back());
      
      viewpoint_cluster.child_cluster_ids_.push_back(cur_cluster.id_);
      viewpoint_cluster.child_cluster_ids_.push_back(nbr_cluster.id_);

      viewpoint_cluster.merged_level_ = max(cur_cluster.merged_level_, nbr_cluster.merged_level_) + 1;

      viewpoint_cluster.dist_ = cur_cluster.dist_ + nbr_cluster.dist_ 
                                + adj_list_[cur_cluster.backward_source_][cur_cluster.backward_target_];

      viewpoint_clusters_[new_cluster_id] = viewpoint_cluster;

      cur_cluster.is_merged_ = true;
      nbr_cluster.is_merged_ = true;
    }

    if(smallest_direction == 1)
    {
      auto &nbr_cluster = viewpoint_clusters_[viewpoints_[cur_cluster.forward_target_].viewpoint_cluster_id_];
      ViewpointCluster viewpoint_cluster;

      int new_cluster_id = generateViewpointClusterId();
      viewpoint_cluster.id_ = new_cluster_id;

      vector<int> forward_cells;
      for(auto &cell:nbr_cluster.ordered_cells_)
      {
        forward_cells.push_back(cell);
        viewpoints_[cell].viewpoint_cluster_id_ = new_cluster_id;
        if(forward_cells.back()!=cur_cluster.forward_target_)
          reverse(forward_cells.begin(), forward_cells.end());
      }

      vector<int> backward_cells;
      for(auto &cell:cur_cluster.ordered_cells_)
      {
        backward_cells.push_back(cell);
        viewpoints_[cell].viewpoint_cluster_id_ = new_cluster_id;
        if(backward_cells.front()!=cur_cluster.forward_source_)
          reverse(backward_cells.begin(), backward_cells.end());
      }
      viewpoint_cluster.ordered_cells_.insert(viewpoint_cluster.ordered_cells_.end(), 
                                              forward_cells.begin(), forward_cells.end());

      viewpoint_cluster.ordered_cells_.insert(viewpoint_cluster.ordered_cells_.end(), 
                                              backward_cells.begin(), backward_cells.end());                                              

      Vector3d total_pos(0, 0, 0);
      for(auto &cell:viewpoint_cluster.ordered_cells_)
        total_pos+=viewpoints_[cell].pos_;
      viewpoint_cluster.avg_pos_=total_pos/viewpoint_cluster.ordered_cells_.size();

      viewpoint_cluster.vertex_cells_.push_back(viewpoint_cluster.ordered_cells_.front());
      viewpoint_cluster.vertex_cells_.push_back(viewpoint_cluster.ordered_cells_.back());
      
      viewpoint_cluster.child_cluster_ids_.push_back(cur_cluster.id_);
      viewpoint_cluster.child_cluster_ids_.push_back(nbr_cluster.id_);

      viewpoint_cluster.merged_level_ = max(cur_cluster.merged_level_, nbr_cluster.merged_level_) + 1;

      viewpoint_cluster.dist_ = cur_cluster.dist_ + nbr_cluster.dist_ 
                                + adj_list_[cur_cluster.forward_source_][cur_cluster.forward_target_];

      viewpoint_clusters_[new_cluster_id] = viewpoint_cluster;

      cur_cluster.is_merged_ = true;
      nbr_cluster.is_merged_ = true;
    }
  }

  // 基于历史TSP的聚类
  void FrontierFinder::unmergeViewpointCluster()
  {
    vector<int> need_erase;
    // vector<double> dists = {30.0, 40.0, 50.0, 60.0};
    vector<double> dists = {25.0, 35.0};

    for(int i = 0; i < dists.size(); i++)
    {
      for(auto &iter:viewpoint_clusters_)
      {
        auto &cur = iter.second;
        if(cur.merged_level_ == (i+1) && !cur.is_merged_)
        {     
          double distance = 100000;
          for(auto &vertex_cell:cur.vertex_cells_)
          {
            distance = min(distance, (viewpoints_[vertex_cell].pos_-current_pos_).norm());
          }
          if(distance < dists[i])
          {
            auto &child_1 = viewpoint_clusters_[cur.child_cluster_ids_[0]];
            auto &child_2 = viewpoint_clusters_[cur.child_cluster_ids_[1]];  

            child_1.is_merged_ = false;
            child_2.is_merged_ = false;

            child_1.forward_source_ = child_1.forward_target_ = -1;
            child_1.backward_source_ = child_1.backward_target_ = -1;

            child_2.forward_source_ = child_2.forward_target_ = -1;
            child_2.backward_source_ = child_2.backward_target_ = -1;

            for(auto &cell:child_1.ordered_cells_)
              viewpoints_[cell].viewpoint_cluster_id_ = child_1.id_;

            for(auto &cell:child_2.ordered_cells_)
              viewpoints_[cell].viewpoint_cluster_id_ = child_2.id_;
            need_erase.push_back(iter.first);
          }
        }   
      }

      for(auto &id:need_erase)
      {
        viewpoint_clusters_.erase(id);
        viewpoint_cluster_ids_.push(id);
      }
    }
  }

  double FrontierFinder::distToBox(Vector3d &box_min, Vector3d &box_max, Vector3d &point)
  {
    bool is_outside_box = (point.x() < box_min.x() || point.x() > box_max.x() ||
                           point.y() < box_min.y() || point.y() > box_max.y() ||
                           point.z() < box_min.z() || point.z() > box_max.z());

    if (!is_outside_box) {
        return -1; 
    }
    // 计算包围盒上与point最近的点
    Vector3d closest_point_on_box;
    closest_point_on_box.x() = std::max(box_min.x(), std::min(point.x(), box_max.x()));
    closest_point_on_box.y() = std::max(box_min.y(), std::min(point.y(), box_max.y()));
    closest_point_on_box.z() = std::max(box_min.z(), std::min(point.z(), box_max.z()));

    double distance = sqrt(pow(point.x() - closest_point_on_box.x(), 2) +
                           pow(point.y() - closest_point_on_box.y(), 2) +
                           pow(point.z() - closest_point_on_box.z(), 2));
    return distance;
  }

  void FrontierFinder::updateViewpointCluster()
  {
    // ROS_WARN("Erase old cluster");
    vector<int> need_erase_cluster_ids;
    for(auto &iter:viewpoint_clusters_)
    {
      auto &cluster = iter.second;
      if((cluster.avg_pos_-current_pos_).norm() < 80.0)
      {
        for(auto &cell:cluster.cells_)
        {
          // ROS_WARN("Check viewpoint");
          auto &viewpoint = viewpoints_.at(cell);
          if((viewpoint.pos_-current_pos_).norm() < 26.0)
          {
            need_erase_cluster_ids.push_back(cluster.id_);
            break;
          }
        }
      }
    }

    for(auto &id:need_erase_cluster_ids)
    {
      auto &cluster = viewpoint_clusters_.at(id);
      for(auto &cell:cluster.cells_)
      {
        // ROS_WARN("Change viewpoint status");
        viewpoints_.at(cell).is_clustered_ = false;
        viewpoints_.at(cell).viewpoint_cluster_id_ = -1;
      }
      viewpoint_clusters_.erase(id);
      viewpoint_cluster_ids_.push(id);     
    }
    vp_.erased_viewpoint_cluster_ids_.insert(need_erase_cluster_ids.begin(), need_erase_cluster_ids.end());

    // ROS_WARN("Find unclustered viewpoint");

    std::unordered_set<int> unclustered_viewpoints;
    std::unordered_set<int> changed_viewpoint_clusters;
    for(auto &iter:viewpoints_)
    {
      auto &viewpoint = iter.second;
      if(!viewpoint.is_clustered_ && ((current_pos_-viewpoint.pos_).norm() > 28.0))
      {
        unclustered_viewpoints.insert(viewpoint.id_);
      }
    }

    // ROS_WARN("Cluster unclustered viewpoint");

    while(!unclustered_viewpoints.empty())
    {
      int nearest_viewpoint_id = -1;
      int nearest_clustered_viewpoint_id = -1;
      int nearest_cluster_id = -1;
      double min_dist = 100000; 
      // ROS_WARN("Find nearest pair");
      for(auto &viewpoint_id:unclustered_viewpoints)  // 遍历所有未聚类的视点
      {
        // ROS_WARN("viewpoint_id:%d", viewpoint_id);
        auto &cur_viewpoint = viewpoints_.at(viewpoint_id);
        for(auto &iter:viewpoint_clusters_) // 遍历所有聚类,并检查是否符合条件
        {
          int tmp_nearest_clustered_viewpoint_id = -1;
          double tmp_min_dist = 100000; 
          auto &cluster = iter.second;

          if(can_be_clustered(cluster, cur_viewpoint, tmp_nearest_clustered_viewpoint_id, tmp_min_dist))
          {
            if(tmp_min_dist < min_dist)
            {
              nearest_viewpoint_id = cur_viewpoint.id_;
              nearest_clustered_viewpoint_id = tmp_nearest_clustered_viewpoint_id;
              nearest_cluster_id = cluster.id_;
              min_dist = tmp_min_dist;
            }
          }  
        }
      }
      // 存在距离最近的，可以聚类的点
      if(nearest_viewpoint_id != -1 && nearest_cluster_id != -1)
      {
        // ROS_WARN("Found nearest pair");
        auto &cluster = viewpoint_clusters_.at(nearest_cluster_id);
        auto &viewpoint = viewpoints_.at(nearest_viewpoint_id);
        cluster.avg_pos_ = (cluster.avg_pos_ * cluster.cells_.size() + viewpoint.pos_) / (cluster.cells_.size()+1);
        cluster.avg_gain_ = (cluster.avg_gain_ * cluster.cells_.size() + viewpoint.gain_) / (cluster.cells_.size()+1);
        cluster.avg_cover_num_ = (cluster.avg_cover_num_ * cluster.cells_.size() + viewpoint.cover_num_) / (cluster.cells_.size()+1);
        cluster.cells_.insert(viewpoint.id_);
        viewpoint.is_clustered_ = true;
        viewpoint.viewpoint_cluster_id_ = cluster.id_;
        unclustered_viewpoints.erase(viewpoint.id_);
        changed_viewpoint_clusters.insert(cluster.id_);
      }
      else  // 不存在则新建一个聚类
      {
        // ROS_WARN("Not found nearest pair");
        int viewpoint_id = *unclustered_viewpoints.begin();
        auto &viewpoint = viewpoints_.at(viewpoint_id);
        ViewpointCluster new_cluster;
        int new_id = generateViewpointClusterId();
        new_cluster.cells_.insert(viewpoint.id_);
        new_cluster.avg_pos_ = viewpoint.pos_;
        new_cluster.avg_gain_ = viewpoint.gain_;
        new_cluster.avg_cover_num_ = viewpoint.cover_num_;
        new_cluster.id_ = new_id;
        viewpoint_clusters_[new_id] = new_cluster;

        viewpoint.is_clustered_ = true;
        viewpoint.viewpoint_cluster_id_ = new_id;
        unclustered_viewpoints.erase(viewpoint_id);
        changed_viewpoint_clusters.insert(new_id);
      }
    }

    // ROS_WARN("Update changed cluster info");
    for(auto &cluster_id:changed_viewpoint_clusters)
    {
      auto &cluster = viewpoint_clusters_.at(cluster_id);
      computeViewpointClusterInfo(cluster);
    }
    vp_.changed_viewpoint_cluster_ids_.insert(changed_viewpoint_clusters.begin(), changed_viewpoint_clusters.end());

    // ROS_WARN("Viewpoint cluster size:%d", viewpoint_clusters_.size());

    // for(auto &iter:viewpoint_clusters_)
    // {
    //   auto &cluster = iter.second;
    //   ROS_WARN("id:%d  cell_num:%d  vertex_num:%d", cluster.id_, cluster.cells_.size(), cluster.vertex_pos_.size());
    // }

    for(auto &viewpoint:viewpoints_)
    {
      if(surface_frontier_clusters_.find(viewpoint.second.id_) == surface_frontier_clusters_.end())
        ROS_WARN("[viewpoint cluster]:frontier id, viewpoint id not match");
    }
  }

  bool FrontierFinder::can_be_clustered(ViewpointCluster  &cluster,
                                        Viewpoint         &viewpoint,
                                        int               &nearest_viewpoint_id,
                                        double            &min_dist)
  {
    if((viewpoint.pos_-cluster.avg_pos_).norm() > 15.0)
      return false;
    // if(cluster.cells_.size() > 5)
    //   return false;

    nearest_viewpoint_id = -1;
    min_dist = 100000; 
    double fail_num = 0;
        
    for(auto &cell_iter:cluster.cells_)
    {
      auto &nbr_viewpoint = viewpoints_.at(cell_iter);
      // if(sdf_map_->isCollisionFreeStraight(viewpoint.pos_, nbr_viewpoint.pos_))
      
      // 检测两点之间是否有障碍物
      bool visiable = true;
      Vector3i idx;
      raycaster_->input(viewpoint.pos_, nbr_viewpoint.pos_);
      while (raycaster_->nextId(idx))
      {
        if (sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED)
        {
          visiable = false;
          break;
        }
      }
      
       
      if(visiable)
      {
        if((viewpoint.pos_-nbr_viewpoint.pos_).norm() < min_dist)
        {
          min_dist = (viewpoint.pos_-nbr_viewpoint.pos_).norm();
          nearest_viewpoint_id = nbr_viewpoint.id_;
        }
      }
      else
      {
        fail_num+=1.0;
      }
    }  
    if(min_dist < 12.0 && (fail_num/cluster.cells_.size() < 0.1))
      return true;
    else
      return false;
  }

  void FrontierFinder::computeViewpointClusterInfo(ViewpointCluster &cluster)
  {
    cluster.vertex_cells_.clear();
    cluster.vertex_pos_.clear();
    if(cluster.cells_.size()==1)
    {
      cluster.vertex_cells_.push_back(*cluster.cells_.begin());
      cluster.vertex_pos_.push_back(viewpoints_.at(*cluster.cells_.begin()).pos_);
    }
    else if(cluster.cells_.size() > 1)
    {
      vector<Vector3d> points;
      vector<int> index_map;
      for(auto &iter:cluster.cells_)
      {
        auto &viewpoint = viewpoints_.at(iter);
        points.push_back(viewpoint.pos_);
        index_map.push_back(iter);
      }

      Vector3d mean, eigenvalues;
      Eigen::Matrix3d eigenvectors;
      pca(points, mean, eigenvectors, eigenvalues);
      Vector3d dict = eigenvectors.col(2);
      int vertex_1, vertex_2;
      findVertex(mean, dict, points, vertex_1, vertex_2);

      cluster.dict_ = dict;
      cluster.eigenvalue_ = eigenvalues[2];

      cluster.vertex_cells_.push_back(index_map[vertex_1]);
      cluster.vertex_cells_.push_back(index_map[vertex_2]);

      vector<Vector3d> vertices;
      vertices.push_back(Vector3d(cluster.avg_pos_-cluster.dict_.normalized()*sqrt(cluster.eigenvalue_)));
      vertices.push_back(Vector3d(cluster.avg_pos_+cluster.dict_.normalized()*sqrt(cluster.eigenvalue_)));

      if(cluster.cells_.size() == 2)
      {
        for(auto &iter:cluster.cells_)
        {
          const auto &pos = viewpoints_.at(iter).pos_;
          cluster.vertex_pos_.push_back(pos);
        }
      }
      else
      {
        for(int i = 0; i < 2; ++i)
        {
          int nearest_id = -1;
          Vector3d nearest_pos;
          double nearest_dist = 100000;
          for(auto &iter:cluster.cells_)
          {
            const auto &pos = viewpoints_.at(iter).pos_;
            if((vertices[i]-pos).norm() < nearest_dist)
            {
              nearest_id = iter;
              nearest_dist = (vertices[i]-pos).norm();
              nearest_pos = viewpoints_.at(iter).pos_;
            }
          }
          double dist = (vertices[i]-nearest_pos).norm();
          double step = 0.4;
          int points_num = dist/step;
          for(int j = 0; j <= points_num; j++)
          {
            double t = static_cast<double>(j) / points_num;
            Vector3d inter_pos = (1 - t) * vertices[i] + t * nearest_pos;
            
            bool collision_free = true;
            raycaster_->input(nearest_pos, inter_pos);
            Eigen::Vector3i idx;
            while (raycaster_->nextId(idx) && ros::ok())
            {
              Vector3d pos;
              sdf_map_->indexToPos(idx, pos);
              if (sdf_map_->getInflateOccupancy(idx) == 1 ||
                  sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN || 
                  !isFarFromOccupied(pos, surface_frontier_obstacle_dist_))
              {
                collision_free = false;
                break;
              }
            }
            if(collision_free)
            {
              cluster.vertex_pos_.push_back(inter_pos);
              break;              
            }

            // if(sdf_map_->getInflateOccupancy(inter_pos)!=1 
            // && sdf_map_->getOccupancy(inter_pos) == SDFMap::FREE)
            // {
            //   cluster.vertex_pos_.push_back(inter_pos);
            //   break;
            // }
          }
        }
      }
    }
  }

  void FrontierFinder::pca(const vector<Vector3d>& points, Vector3d &mean, Eigen::Matrix3d &eigenvectors, Eigen::Vector3d &eigenvalues) 
  {
      int point_num = points.size();
      if (point_num == 0) 
      {
          ROS_WARN("Empty viewpoint cluster!");
          return;
      }
      // 1. 计算均值
      mean = Vector3d::Zero();
      for (const auto& point : points) 
          mean += point;
      mean /= point_num;
      // 2. 计算协方差矩阵
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto& point : points) 
      {
          Vector3d centered = point - mean;  // 将点减去均值
          covariance += centered * centered.transpose();  // 累加协方差
      }
      covariance /= point_num;
      // 3. 进行特征值分解
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
      if (eigen_solver.info() != Eigen::Success) 
      {
        ROS_WARN("Solve eigen failed!");
        return;
      }
      // 4. 获取特征值和特征向量
      eigenvalues = eigen_solver.eigenvalues();
      eigenvectors = eigen_solver.eigenvectors();
  }

  void FrontierFinder::findVertex(Vector3d &mean, Vector3d &dict, vector<Vector3d> &points, int &vertex_1, int &vertex_2)
  {
      int min_id = 0, max_id = 0;
      double min_dist = 10000, max_dist = -10000;
      for(int i = 0; i < points.size(); i++)
      {        
          double dist = (points[i] - mean).dot(dict) / dict.dot(dict); 
          if(dist < min_dist)
          {
              min_id = i;
              min_dist = dist;
          }
          if(dist > max_dist)
          {
              max_id = i;
              max_dist = dist;
          }
      }
      vertex_1 = min_id;
      vertex_2 = max_id;
  } 

  void FrontierFinder::global_planning2()
  {
    // DEBUG
    end_points_.clear();
    end_vels_.clear();
    road_map_paths_.clear();
    // DEBUG


    global_planning_iteration_num_++;
    ROS_WARN("Start global planning ~~~~~~~~~~~~~~~~~~~");

    double start_global_planning = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;

    adj_list_.clear();
    generateAdjList();

    index_map_.clear();
    for(auto &iter:adj_list_)
      index_map_.push_back(iter.first);

    int tour_points_num = index_map_.size();
    ROS_WARN("tour_points_num:%d", tour_points_num);

    // DEBUG
    int calc_num = 0; // 路径搜索次数
    // DEBUG

    if(tour_points_num > 1)
    {
      vector<vector<double>> cost_mat(tour_points_num+1, vector<double>(tour_points_num+1, 100000));

      for(int i = 0; i < tour_points_num; i++)
      {
        if(index_map_[i] < 0)
          continue;
        auto &viewpoint = viewpoints_[index_map_[i]];

        vector<Vector3d> path;
        Vector3d viewpoint_pos = viewpoint.pos_;

        calc_num++;
        /* FUEL 中的 Astar */
        Astar_->reset();
        
        if (Astar_->search(current_pos_, viewpoint_pos) != Astar3::REACH_END)
        {
          ROS_ERROR("Find path from cur_pos to viewpoint in grid map failed");
        }
        else
          // ROS_WARN("FUEL path finder successed");
        path = Astar_->getPath();
        // gridmap找不到再到pre_paths_或roadmap中找
        if (path.empty())
        {
          path = getPathInRoadMap(current_pos_, viewpoint_pos);
        }

        if(!path.empty())
        {
          double distance = 0;
          for(int j = 0; j < static_cast<int>(path.size())-1; j++)
          {
            distance += (path[j+1]-path[j]).norm();
          }
          cost_mat[0][i+1] = distance;  // TODO 暂时不使用方向惩罚

          // double t = 0;
          // for(int i = 0; i < 3; i++)
          // {
          //   t = max(t, calculateMotionTime(forward_directory_[i], 2, 2, viewpoint_pos[i]-current_pos_[i]));
          // }

          // Vector3d zero(0, 0, 0);
          // vector<Eigen::MatrixXd> p;
          // vector<double> jerk_integrals;
          // jerk_traj(current_pos_, forward_directory_, zero, viewpoint_pos, zero, zero, t, p, jerk_integrals);
          // Vector3d vel = get_velocity(p, t*0.99);

          // double angle_rad =  acos(forward_directory_.dot(vel) / (forward_directory_.norm() * vel.norm()));
          
          // // Vector3d diff_vector = viewpoint - current_pos_;
          // // double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));
          
          // double angle_deg = angle_rad * (180.0 / M_PI);
          // double alpha = 0.2;
          // // FAEL的方法
          // double cost = ((1-alpha)*log2(angle_deg/180.0+1) + alpha) * distance * tour_points_num;

          // // DEBUG
          // double cos = forward_directory_.dot(vel) / (forward_directory_.norm() * vel.norm());
          // double angle = angle_deg;
          // double angle_cost = log2(angle_deg/180.0+1);
          // // ROS_WARN("cos:%0.2f  angle:%0.2f  angle_cost:%0.2f", cos, angle, angle_cost);

          // end_points_.push_back(viewpoint_pos);
          // end_vels_.push_back(vel);
          // // DEBUG

          // cost_mat[0][i+1] = cost;
          // // ROS_WARN("Cost:%f", cost_mat[0][i+1]);
        }
        else
        {
          ROS_ERROR("Find current to cluster:%d viewpoint path failed!", index_map_[i]);
          ROS_ERROR("Find path from cur_pos to viewpoint in grid map failed");
        }
        cost_mat[i+1][0] = 0;
      }

      double start_viewpoint_to_viewpoint = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;

      ROS_WARN("tour_points_num %d", tour_points_num);

      for(int i = 0; i < tour_points_num; i++)
      {
        auto &viewpoint_1 = viewpoints_[index_map_[i]];
        for(int j = 0; j < i; j++)
        {
          auto &viewpoint_2 = viewpoints_[index_map_[j]];
          vector<Vector3d> path;
          // 不在同一个聚类
          if(viewpoint_1.viewpoint_cluster_id_!=viewpoint_2.viewpoint_cluster_id_)
          {
            if(!viewpoint_1.is_vertex_ || !viewpoint_2.is_vertex_)
              continue;
          }
          else  // 在同一个聚类
          {
            if(viewpoint_1.viewpoint_cluster_id_!=-1)
            {
              if(!viewpoint_1.neighbor_viewpoint_ids_.count(index_map_[j]))
                continue;
            }
          }

          calc_num++;

          if((viewpoint_1.pos_-viewpoint_2.pos_).norm() < 20.0) // 距离近的用grid map找，远的直接用road map
          {
            // path = getPathInGridMap3D(viewpoint_1, viewpoint_2, false);
            /* FUEL 中的 Astar */
            Astar_->reset();
            ros::Time start_time = ros::Time::now();
            if (Astar_->search(viewpoint_1.pos_, viewpoint_2.pos_) != Astar3::REACH_END)
            {
              ROS_ERROR("Find path from viewpoint to viewpoint in grid map failed");
            }
            else
            {
              path = Astar_->getPath();
              ros::Time end_time = ros::Time::now();
              // ROS_WARN("Find path from viewpoint to viewpoint in grid map spend:%0.6f", (end_time-start_time).toSec());
            }
          }
          // gridmap找不到再到roadmap中找
          if (path.empty())
          {
            ros::Time start_time = ros::Time::now();
            path = getPathInRoadMap(viewpoint_1.pos_, viewpoint_2.pos_);
            // ROS_WARN("roadmap path point num:%d", path.size());
            if (!path.empty())
            {
              ros::Time end_time = ros::Time::now();
              // ROS_WARN("Find path from viewpoint to viewpoint in road map spend:%0.6f", (end_time-start_time).toSec());
              // DEBUG
              // if(road_map_paths_.size() < 5)
              //   road_map_paths_.push_back(path);
              // DEBUG
            }
            else
              ROS_ERROR("Find path from viewpoint to viewpoint in road map failed");
          }
          if(!path.empty())
          {
            double distance = 0;
            for(int k = 0; k < static_cast<int>(path.size())-1; k++)
            {
              distance += (path[k+1]-path[k]).norm();
            }
            // ROS_WARN("Path distance:%0.2f", distance);
            cost_mat[i+1][j+1] = distance;
            cost_mat[j+1][i+1] = distance;
          }
        }
      }

      ROS_WARN("Calc num:%d", calc_num);
      
      double start_tsp = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
      
      const int dimension = tour_points_num+1;
      ROS_WARN("TSP dimension:%d", dimension);
      vector<int> indices;

      // Write params and cost matrix to problem file
      ofstream prob_file(tsp_dir_ + "/new_tsp.tsp");
      // Problem specification part, follow the format of TSPLIB

      string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                        "\nEDGE_WEIGHT_TYPE : "
                        "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

      prob_file << prob_spec;
      // prob_file << "TYPE : TSP\n";
      // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
      // Problem data part
      const int scale = 100;

      // Use Asymmetric TSP
      for (int i = 0; i < dimension; ++i)
      {
        for (int j = 0; j < dimension; ++j)
        {
          int int_cost = cost_mat[i][j] * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
      
      prob_file << "EOF";
      prob_file.close();

      // Call LKH TSP solver
      solveTSPLKH((tsp_dir_ + "/new_tsp.par").c_str());

      // Read optimal tour from the tour section of result file
      ifstream res_file(tsp_dir_ + "/new_tsp.txt");
      string res;
      while (getline(res_file, res))
      {
        // Go to tour section
        if (res.compare("TOUR_SECTION") == 0)
          break;
      }

      tsp_index_.clear();
      // Read path for ATSP formulation
      while (getline(res_file, res))
      {
        // Read indices of frontiers in optimal tour
        int id = stoi(res);
        // ROS_WARN("id:%d", id);
        if (id == 1) // Ignore the current state
          continue;
        if (id == -1)
          break;
        tsp_index_.push_back(id - 2); // Idx of solver-2 == Idx of frontier
      }
      
      res_file.close();

      double end_tsp = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
      double iteration_time = end_tsp - start_tsp;
      total_tsp_solve_time_ = total_tsp_solve_time_ + iteration_time;
      
      // file_utils::writeToFileByAdd(tsp_solve_time_txt_name_, "\t", start_tsp-start_time_ms_, end_tsp-start_time_ms_, iteration_time,
      //                               global_planning_iteration_num_, total_tsp_solve_time_ / global_planning_iteration_num_, dimension);
    
      file_utils::writeToFileByAdd(cost_mat_time_txt_name_, "\t", iteration_time, end_tsp-start_global_planning, 
                                    start_viewpoint_to_viewpoint-start_global_planning, start_tsp-start_viewpoint_to_viewpoint, end_tsp-start_tsp, dimension);
    }
    else  // 只有一个tour point
    {
      tsp_index_.clear();
      tsp_index_.push_back(0);
    }

    ordered_tour_points_.clear();
    ordered_tour_points_frontiers_.clear();

    int points_num = min((int)tsp_index_.size(), 3);

    for(int i = 0; i < points_num; i++)
    {
      int viewpoint_id = index_map_[tsp_index_[i]];
      ordered_tour_points_.push_back(viewpoints_[viewpoint_id].pos_);

      vector<int> frontiers;
      for(auto &cell_iter:surface_frontier_clusters_[viewpoints_[viewpoint_id].frontier_cluster_id_].cells_)
        frontiers.push_back(cell_iter.first);
      ordered_tour_points_frontiers_.push_back(frontiers);
    }
    global_planning_update_ = true;


    for(int i = 0; i < tsp_index_.size(); i++)
    {
      int cur = index_map_[tsp_index_[i]];
      if(cur!=-1)
      {
        if(i > 0 && i < tsp_index_.size()-1)
        {
          int forward = index_map_[tsp_index_[i-1]];
          int backward = index_map_[tsp_index_[i+1]];
          if(forward!=-1)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = forward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
          }
          if(backward!=-1)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = backward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;          
          }
        }
        else if(i == 0)
        {
          int backward = index_map_[tsp_index_[i+1]];
          if(backward!=-1)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = backward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;
          }
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
        }
        else if(i == tsp_index_.size()-1)
        {
          int forward = index_map_[tsp_index_[i-1]];
          if(forward!=-1)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = forward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
          }
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;
        }
      }
    }
  }

  // 基于历史TSP的视点聚类
  void FrontierFinder::global_planning3()
  {
    // DEBUG
    end_points_.clear();
    end_vels_.clear();
    road_map_paths_.clear();
    // DEBUG

    global_planning_iteration_num_++;
    ROS_WARN("Start global planning ~~~~~~~~~~~~~~~~~~~");

    double start_global_planning = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;

    adj_list_.clear();
    generateAdjList();

    index_map_.clear();
    for(auto &iter:adj_list_)
      index_map_.push_back(iter.first);

    int tour_points_num = index_map_.size();
    ROS_WARN("tour_points_num:%d", tour_points_num);

    // DEBUG
    int calc_num = 0; // 路径搜索次数
    // DEBUG

    if(tour_points_num > 1)
    {
      vector<vector<double>> cost_mat(tour_points_num+1, vector<double>(tour_points_num+1, 100000));

      for(int i = 0; i < tour_points_num; i++)
      {
        if(index_map_[i] < 0)
          continue;
        auto &viewpoint = viewpoints_[index_map_[i]];
        vector<Vector3d> path;
        Vector3d viewpoint_pos = viewpoint.pos_;
        Astar_->reset();
        if (Astar_->search(current_pos_, viewpoint_pos) != Astar3::REACH_END)
        {
          ROS_ERROR("Find path from cur_pos to viewpoint in grid map failed");
        }
        else
          // ROS_WARN("FUEL path finder successed");
        path = Astar_->getPath();
        // gridmap找不到再到pre_paths_或roadmap中找
        if (path.empty())
        {
          path = getPathInRoadMap(current_pos_, viewpoint_pos);
        }
        if(!path.empty())
        {
          double distance = 0;
          for(int j = 0; j < path.size()-1; j++)
          {
            distance += (path[j+1]-path[j]).norm();
          }
          cost_mat[0][i+1] = distance;  // TODO 暂时不使用方向惩罚

          // double t = 0;
          // for(int i = 0; i < 3; i++)
          // {
          //   t = max(t, calculateMotionTime(forward_directory_[i], 2, 2, viewpoint_pos[i]-current_pos_[i]));
          // }

          // Vector3d zero(0, 0, 0);
          // vector<Eigen::MatrixXd> p;
          // vector<double> jerk_integrals;
          // jerk_traj(current_pos_, forward_directory_, zero, viewpoint_pos, zero, zero, t, p, jerk_integrals);
          // Vector3d vel = get_velocity(p, t*0.99);

          // double angle_rad =  acos(forward_directory_.dot(vel) / (forward_directory_.norm() * vel.norm()));
          
          // // Vector3d diff_vector = viewpoint - current_pos_;
          // // double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));
          
          // double angle_deg = angle_rad * (180.0 / M_PI);
          // double alpha = 0.2;
          // // FAEL的方法
          // double cost = ((1-alpha)*log2(angle_deg/180.0+1) + alpha) * distance * tour_points_num;

          // // DEBUG
          // double cos = forward_directory_.dot(vel) / (forward_directory_.norm() * vel.norm());
          // double angle = angle_deg;
          // double angle_cost = log2(angle_deg/180.0+1);
          // // ROS_WARN("cos:%0.2f  angle:%0.2f  angle_cost:%0.2f", cos, angle, angle_cost);

          // end_points_.push_back(viewpoint_pos);
          // end_vels_.push_back(vel);
          // // DEBUG

          // cost_mat[0][i+1] = cost;
          // // ROS_WARN("Cost:%f", cost_mat[0][i+1]);
        }
        else
        {
          ROS_ERROR("Find current to cluster:%d viewpoint path failed!", index_map_[i]);
          ROS_ERROR("Find path from cur_pos to viewpoint in grid map failed");
        }
        cost_mat[i+1][0] = 0;
      }

      double start_viewpoint_to_viewpoint = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;

      ROS_WARN("tour_points_num %d", tour_points_num);

      for(int i = 0; i < tour_points_num; i++)
      {
        cerr << index_map_[i] << ": ";
        if(adj_list_.count(index_map_[i]))
        {
          for(int j = 0; j < i; j++)
          {
            if(adj_list_[index_map_[i]].count(index_map_[j]))
            {
              cost_mat[i+1][j+1] = adj_list_[index_map_[i]][index_map_[j]];
              cost_mat[j+1][i+1] = adj_list_[index_map_[i]][index_map_[j]];
            }
            cerr << index_map_[j] << ":" << cost_mat[i+1][j+1] << "  ";
          }
        }
        cerr << endl;
      }

      ROS_WARN("Calc num:%d", calc_num);
      
      double start_tsp = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
      
      const int dimension = tour_points_num+1;
      ROS_WARN("TSP dimension:%d", dimension);
      vector<int> indices;

      // Write params and cost matrix to problem file
      ofstream prob_file(tsp_dir_ + "/new_tsp.tsp");
      // Problem specification part, follow the format of TSPLIB

      string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                        "\nEDGE_WEIGHT_TYPE : "
                        "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

      prob_file << prob_spec;
      // prob_file << "TYPE : TSP\n";
      // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
      // Problem data part
      const int scale = 100;

      // Use Asymmetric TSP
      for (int i = 0; i < dimension; ++i)
      {
        for (int j = 0; j < dimension; ++j)
        {
          int int_cost = cost_mat[i][j] * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
      
      prob_file << "EOF";
      prob_file.close();

      // Call LKH TSP solver
      solveTSPLKH((tsp_dir_ + "/new_tsp.par").c_str());

      // Read optimal tour from the tour section of result file
      ifstream res_file(tsp_dir_ + "/new_tsp.txt");
      string res;
      while (getline(res_file, res))
      {
        // Go to tour section
        if (res.compare("TOUR_SECTION") == 0)
          break;
      }

      tsp_index_.clear();
      // Read path for ATSP formulation
      while (getline(res_file, res))
      {
        // Read indices of frontiers in optimal tour
        int id = stoi(res);
        // ROS_WARN("id:%d", id);
        if (id == 1) // Ignore the current state
          continue;
        if (id == -1)
          break;
        tsp_index_.push_back(id - 2); // Idx of solver-2 == Idx of frontier
      }
      
      res_file.close();

      double end_tsp = static_cast<double>(time_utils::Timer::GetTimeNow("us")) / 1e3;
      double iteration_time = end_tsp - start_tsp;
      total_tsp_solve_time_ = total_tsp_solve_time_ + iteration_time;
      
      // file_utils::writeToFileByAdd(tsp_solve_time_txt_name_, "\t", start_tsp-start_time_ms_, end_tsp-start_time_ms_, iteration_time,
      //                               global_planning_iteration_num_, total_tsp_solve_time_ / global_planning_iteration_num_, dimension);
    
      file_utils::writeToFileByAdd(cost_mat_time_txt_name_, "\t", iteration_time, end_tsp-start_global_planning, 
                                    start_viewpoint_to_viewpoint-start_global_planning, start_tsp-start_viewpoint_to_viewpoint, end_tsp-start_tsp, dimension);
    }
    else  // 只有一个tour point
    {
      tsp_index_.clear();
      tsp_index_.push_back(0);
    }

    ordered_tour_points_.clear();
    ordered_tour_points_frontiers_.clear();

    int points_num = min((int)tsp_index_.size(), 3);

    // for(int i = 0; i < points_num; i++)
    // {

    //   int viewpoint_id = index_map_[tsp_index_[i]];
    //   ordered_tour_points_.push_back(viewpoints_[viewpoint_id].pos_);

    //   vector<int> frontiers;
    //   for(auto &cell_iter:surface_frontier_clusters_[viewpoints_[viewpoint_id].frontier_cluster_id_].cells_)
    //     frontiers.push_back(cell_iter.first);
    //   ordered_tour_points_frontiers_.push_back(frontiers);
    // }
    // global_planning_update_ = true;


    for(int i = 0; i < tsp_index_.size(); i++)
    {
      int cur = index_map_[tsp_index_[i]];
      if(cur >= 0)
      {
        if(i > 0 && i < tsp_index_.size()-1)
        {
          int forward = index_map_[tsp_index_[i-1]];
          int backward = index_map_[tsp_index_[i+1]];
          if(forward >= 0)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = forward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
          }
          if(backward >= 0)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = backward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;          
          }
        }
        else if(i == 0)
        {
          int backward = index_map_[tsp_index_[i+1]];
          if(backward >= 0)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = backward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;
          }
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
        }
        else if(i == tsp_index_.size()-1)
        {
          int forward = index_map_[tsp_index_[i-1]];
          if(forward!=-1)
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = cur;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = forward;
          }
          else
          {
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_source_ = -1;
            viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].forward_target_ = -1;
          }
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_source_ = cur;
          viewpoint_clusters_[viewpoints_[cur].viewpoint_cluster_id_].backward_target_ = -1;
        }
      }
    }
  }

  // 基于历史TSP的视点聚类
  void FrontierFinder::generateAdjList()
  {
    for(auto &cur_iter:viewpoint_clusters_)
    {
      auto &cur_clsuter = cur_iter.second;
      if(cur_clsuter.is_merged_)
        continue;
      for(auto &vertex_cell:cur_clsuter.vertex_cells_)  // 对于所有端点
      {
        for(auto &nbr_iter:viewpoint_clusters_)
        {
          auto &nbr_cluster = nbr_iter.second;
          if(nbr_cluster.is_merged_ || cur_clsuter.id_ == nbr_cluster.id_)
            continue;

          for(auto &nbr_vertex_cell:nbr_cluster.vertex_cells_)
          {
            auto viewpoint_1_pos = viewpoints_[vertex_cell].pos_;
            auto viewpoint_2_pos = viewpoints_[nbr_vertex_cell].pos_;
            vector<Vector3d> path;
            if((viewpoint_1_pos-viewpoint_2_pos).norm() < 20.0) // 距离近的用grid map找，远的直接用road map
            {
              Astar_->reset();
              if (Astar_->search(viewpoint_1_pos, viewpoint_2_pos) != Astar3::REACH_END)
              {
                ROS_ERROR("Find path from viewpoint to viewpoint in grid map failed");
              }
              else
              {
                path = Astar_->getPath();
                // ROS_WARN("Find path from viewpoint to viewpoint in grid map spend:%0.6f", (end_time-start_time).toSec());
              }
            }
            if (path.empty()) // gridmap找不到再到roadmap中找
            {
              path = getPathInRoadMap(viewpoint_1_pos, viewpoint_2_pos);
            }

            if (!path.empty())
            {
              double distance = 0;
              for(int k = 0; k < path.size()-1; k++)
              {
                distance += (path[k+1]-path[k]).norm();
              }
              adj_list_[vertex_cell][nbr_vertex_cell] = distance;
            }
            else
            {
              adj_list_[vertex_cell][nbr_vertex_cell] = 100000;
              ROS_ERROR("Find path from viewpoint to viewpoint in road map failed");
            }
          }
        } 
        if(cur_clsuter.vertex_cells_.size()==2)
        {
          adj_list_[-cur_clsuter.id_][vertex_cell] = cur_clsuter.dist_/2;
          adj_list_[vertex_cell][-cur_clsuter.id_] = cur_clsuter.dist_/2;
        }
      }
    }

    for(auto &adj:adj_list_)
    {
      cerr << adj.first << ": ";
      for(auto &nbr:adj.second)
      {
        cerr << nbr.first << ":" << nbr.second << "  ";
      }
      cerr << endl;
    }
  }

  void FrontierFinder::generateAdjList2()
  {
    for(auto &cur_iter:viewpoint_clusters_)
    {
      auto &cur_clsuter = cur_iter.second;
      for(auto &vertex_cell:cur_clsuter.vertex_cells_)  // 对于所有端点
      {
        viewpoints_.at(vertex_cell);
        for(auto &other_iter:viewpoint_clusters_) // 遍历所有视点聚类
        {
          auto &other_cluster = other_iter.second;
          if(cur_clsuter.id_ == other_cluster.id_)
            continue;

          for(auto &other_vertex_cell:other_cluster.vertex_cells_) // 遍历聚类内所有端点
          {
            viewpoints_.at(other_vertex_cell);
            adj_list_[vertex_cell][other_vertex_cell] = 10;
          }
        } 

        for(auto &iter:viewpoints_)
        {
          auto &other_viewpoint = iter.second;
          if(other_viewpoint.is_clustered_)
            continue;
          viewpoints_.at(other_viewpoint.id_);
          adj_list_[vertex_cell][other_viewpoint.id_] = 10;
        }

        if(cur_clsuter.vertex_cells_.size()==2)
        {
          adj_list_[-cur_clsuter.id_][vertex_cell] = 1;
          adj_list_[vertex_cell][-cur_clsuter.id_] = 1;
        }
      }
    }

    for(auto &iter:viewpoints_)
    {
      auto &cur_viewpoint = iter.second;
      if(cur_viewpoint.is_clustered_)
          continue;
      viewpoints_.at(cur_viewpoint.id_);
      for(auto &other_iter:viewpoint_clusters_) // 遍历所有视点聚类
      {
        auto &other_cluster = other_iter.second;

        for(auto &other_vertex_cell:other_cluster.vertex_cells_) // 遍历聚类内所有端点
        {
          viewpoints_.at(other_vertex_cell);
          adj_list_[cur_viewpoint.id_][other_vertex_cell] = 10.0;
        }    
      } 

      for(auto &other_iter:viewpoints_)
      {
        auto &other_viewpoint = other_iter.second;
        if(other_viewpoint.is_clustered_)
          continue;
        viewpoints_.at(other_viewpoint.id_);
        adj_list_[cur_viewpoint.id_][other_viewpoint.id_] = 10.0;
      }
    }

    // DEBUG
    // for(auto &adj:adj_list_)
    // {
    //   cerr << adj.first << ": ";
    //   for(auto &nbr:adj.second)
    //   {
    //     cerr << nbr.first << ":" << nbr.second << "  ";
    //   }
    //   cerr << endl;
    // }
    // DEBUG
  }

  void FrontierFinder::global_planning5()
  {
    time_utils::Timer total_timer("total_timer");
    total_timer.Start();

    global_planning_iteration_num_++;
    // ROS_WARN("Start global planning ~~~~~~~~~~~~~~~~~~~");

    end_points_.clear();
    end_vels_.clear();
    adj_list_.clear();

    time_utils::Timer generate_adj_timer("generate_adj_timer");
    generate_adj_timer.Start();

    ROS_WARN("[global planning]:generateAdjList3");
    generateAdjList3();

    generate_adj_timer.Stop();
    double generate_adj_time = static_cast<double>(generate_adj_timer.GetDuration("us")) / 1e3;

    int tour_points_num = adj_nodes_.size();
    // ROS_WARN("tour_points_num:%d", tour_points_num);

    if(tour_points_num > 1)
    {
      vector<vector<double>> cost_mat(tour_points_num+1, vector<double>(tour_points_num+1, 100000));
      
      time_utils::Timer compute_path_timer("compute_path_timer");
      compute_path_timer.Start();

      ROS_WARN("[global planning]:calcPathLength2");
      calcPathLength2(cost_mat);

      compute_path_timer.Stop();
      double compute_path_time = static_cast<double>(compute_path_timer.GetDuration("us")) / 1e3;
     
      // ROS_WARN("Parallel calc spend:%f", compute_path_time);

      time_utils::Timer compute_end_vel_timer("compute_end_vel_timer");
      compute_end_vel_timer.Start();

      ROS_WARN("[global planning]:calc move direction cost");

      double max_gain = -DBL_MAX;
      for(int i = 0; i < tour_points_num; i++)
      {
        double gain;
        if(adj_nodes_[i].type_  == 0)
          // gain = viewpoints_.at(adj_nodes_[i].viewpoint_id_).gain_;
          gain = viewpoints_.at(adj_nodes_[i].viewpoint_id_).cover_num_;
        else if(adj_nodes_[i].type_  == 1)
          // gain = viewpoint_clusters_.at(adj_nodes_[i].viewpoint_cluster_id_).avg_gain_;
          gain = viewpoint_clusters_.at(adj_nodes_[i].viewpoint_cluster_id_).avg_cover_num_;
        
        max_gain = max(max_gain, gain);
      }

      for(int i = 0; i < tour_points_num; i++)
      {
        if(adj_nodes_[i].type_ == 2)
          continue;
        double gain;
        if(adj_nodes_[i].type_  == 0)
          // gain = viewpoints_.at(adj_nodes_[i].viewpoint_id_).gain_;
          gain = viewpoints_.at(adj_nodes_[i].viewpoint_id_).cover_num_;
        else if(adj_nodes_[i].type_  == 1)
          // gain = viewpoint_clusters_.at(adj_nodes_[i].viewpoint_cluster_id_).avg_gain_;
          gain = viewpoint_clusters_.at(adj_nodes_[i].viewpoint_cluster_id_).avg_cover_num_;

        if(cost_mat[0][i+1] < 100000)
        {
          Vector3d forward_pos, forward_vel;
          
          // if(!getFutureStatus(total_time_, forward_pos, forward_vel))
          if(1)
          {
            forward_pos = current_pos_;
            forward_vel = forward_directory_;
          }
          else
          {
            // ROS_WARN("Got future status pos x:%0.2f, y:%0.2f, z:%0.2f  vel x:%0.2f, y:%0.2f, z:%0.2f", 
                    //  forward_pos.x(), forward_pos.y(), forward_pos.z(),
                    //  forward_vel.x(), forward_vel.y(), forward_vel.z());
          }

          Vector3d viewpoint_pos = adj_nodes_[i].pos_;

          // Consider velocity change
          double t = 0;
          for(int axis = 0; axis < 3; axis++)
          {
            t = max(t, calculateMotionTime(forward_vel[axis], 1.5, 2, viewpoint_pos[axis]-forward_pos[axis]));
          }
          Vector3d zero(0, 0, 0);
          vector<Eigen::MatrixXd> p;
          vector<double> jerk_integrals;
          jerk_traj(forward_pos, forward_vel, zero, viewpoint_pos, zero, zero, t, p, jerk_integrals);
          Vector3d vel = get_velocity(p, t*0.99);

          // Vector3d diff_vector = viewpoint - current_pos_;
          // double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));
          
          double angle_rad;
          if(use_velocity_estimate_)
          {
            angle_rad =  acos(forward_vel.dot(vel) / (forward_vel.norm() * vel.norm()));
          }
          else
          {
            Vector3d diff_vector = (viewpoint_pos - forward_pos).normalized();
            forward_vel = forward_vel.normalized();
            angle_rad = acos(forward_vel.dot(diff_vector));
          }
        

          // double angle_deg = angle_rad * (180.0 / M_PI);
          // double alpha = 0.2;
          // FAEL的方法
          // double cost = ((1-alpha)*log2(angle_deg/180.0+1) + alpha) * (cost_mat[0][i+1] + (viewpoint_pos.z()-forward_pos.z()));
          // double cost = ((1-alpha)*log2(angle_deg/180.0+1) + alpha) * cost_mat[0][i+1] * 2;
          // FUEL的方法
          // double cost = angle_rad * 10 + cost_mat[0][i+1]; // 效果好
          // double cost = (1-gain/max_gain) * 2 +  angle_rad * 3 + cost_mat[0][i+1];
          // double cost = angle_rad * w_dir_ + cost_mat[0][i+1]; // 效果好 4.0

          // if((forward_pos-viewpoint_pos).norm() < 3.0)
          //   cost+=10.0;
          
          // DEBUG
          // double cos = forward_vel.dot(vel) / (forward_vel.norm() * vel.norm());
          // double angle = angle_deg;
          // double angle_cost = log2(angle_deg/180.0+1);
          // ROS_WARN("cos:%0.2f  angle:%0.2f  angle_cost:%0.2f", cos, angle, angle_cost);

          end_points_.push_back(viewpoint_pos);
          end_vels_.push_back(vel);
          // DEBUG

          //  Consider velocity direction
          Vector3d diff = viewpoint_pos - current_pos_;
          double phi = asin(abs(diff.z()) / diff.norm()) / M_PI * 180.0;

          double direction_cost = 0;
          // direction_cost = 1.0 - abs(phi-45.0)/45.0;

          double cost = direction_cost * 8.0 + angle_rad * w_dir_ + cost_mat[0][i+1]; // 效果好 4.0

          cost_mat[0][i+1] = cost;
          // ROS_WARN("Cost:%f", cost_mat[0][i+1]);
        }
        else
        {
          // ROS_ERROR("Find current to cluster:%d viewpoint path failed!", adj_nodes_[i]);
          // ROS_ERROR("Find path from cur_pos to viewpoint in grid map failed");
        }
      }

      compute_end_vel_timer.Stop();
      double compute_end_vel_time = static_cast<double>(compute_end_vel_timer.GetDuration("us")) / 1e3;

      time_utils::Timer solve_tsp_timer("solve_tsp_timer");
      solve_tsp_timer.Start();
      
      ROS_WARN("[global planning]:solve tsp");

      const int dimension = tour_points_num+1;
      // ROS_WARN("TSP dimension:%d", dimension);

      // Write params and cost matrix to problem file
      ofstream prob_file(tsp_dir_ + "/new_tsp.tsp");
      // Problem specification part, follow the format of TSPLIB

      string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                        "\nEDGE_WEIGHT_TYPE : "
                        "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

      prob_file << prob_spec;
      // Problem data part
      const int scale = 100;
      // Use Asymmetric TSP
      for (int i = 0; i < dimension; ++i)
      {
        for (int j = 0; j < dimension; ++j)
        {
          int int_cost = cost_mat[i][j] * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
      prob_file << "EOF";
      prob_file.close();


      // Call LKH TSP solver
      solveTSPLKH((tsp_dir_ + "/new_tsp.par").c_str());
     
      // Read optimal tour from the tour section of result file
      ifstream res_file(tsp_dir_ + "/new_tsp.txt");
      string res;
      while (getline(res_file, res))
      {
        // Go to tour section
        if (res.compare("TOUR_SECTION") == 0)
          break;
      }

      tsp_index_.clear();
      // Read path for ATSP formulation
      while (getline(res_file, res))
      {
        // Read indices of frontiers in optimal tour
        int id = stoi(res);
        // ROS_WARN("id:%d", id);
        if (id == 1) // Ignore the current state
          continue;
        if (id == -1)
          break;
        tsp_index_.push_back(id - 2); // Idx of solver-2 == Idx of frontier
      }
      
      res_file.close();

      solve_tsp_timer.Stop();
      double solve_tsp_time = static_cast<double>(solve_tsp_timer.GetDuration("us")) / 1e3;

      total_timer.Stop();
      double total_time = static_cast<double>(total_timer.GetDuration("us")) / 1e3;

      if(trigger_)
      {
        iter_++;
        ros::Time cut_time = ros::Time::now();
        file_utils::writeToFileByAdd(cost_mat_time_txt_name_, "\t", 
                                    iter_,
                                    total_time, 
                                    dimension,
                                    generate_adj_time, 
                                    compute_path_time,
                                    compute_end_vel_time,
                                    solve_tsp_time,
                                    cut_time);
      }

    }
    else if(tour_points_num == 1)  // 只有一个tour point
    {
      tsp_index_.clear();
      tsp_index_.push_back(0);
    }
    else
    {
      return;
    }
    
    // 节点位置的顺序
    node_pos_sequence_.clear();
    if(tsp_index_.size() > 2)
    {
      ROS_WARN("[global planning]:store sequence relation");
      for(int i = 0; i < int(tsp_index_.size()) - 1; i++)
      {
        auto &node_1 = adj_nodes_.at(tsp_index_.at(i));
        auto &node_2 = adj_nodes_.at(tsp_index_.at(i+1));

        if(node_1.type_ != 2 && node_2.type_ != 2)
        {
          node_pos_sequence_[node_1.pos_] = node_2.pos_;
        }
      }
    }
    // viewpointRefine2();
  }

  void FrontierFinder::generateAdjList3()
  {
    adj_nodes_.clear();
    for(auto &cur_iter:viewpoint_clusters_)
    {
      auto &cur_clsuter = cur_iter.second;
      for(auto &vertex:cur_clsuter.vertex_pos_)  // 对于所有端点
      {
        AdjNode node;
        node.type_ = 1;
        node.viewpoint_cluster_id_ = cur_clsuter.id_;
        node.pos_ = vertex;
        adj_nodes_.push_back(node);
      }

      if(cur_clsuter.vertex_pos_.size()==2)
      {
        AdjNode node;
        node.type_ = 2;
        node.viewpoint_cluster_id_ = cur_clsuter.id_;
        adj_nodes_.push_back(node);
      }
    }

    for(auto &iter:viewpoints_)
    {
      auto &cur_viewpoint = iter.second;
      if(cur_viewpoint.is_clustered_)
        continue;

      AdjNode node;
      node.type_ = 0;
      node.viewpoint_id_ = cur_viewpoint.id_;
      if(surface_frontier_clusters_.find(node.viewpoint_id_)==surface_frontier_clusters_.end())
        ROS_WARN("[generateAdjList3]:viewpoint id and frontier cluster not match");
      node.pos_ = cur_viewpoint.pos_;
      adj_nodes_.push_back(node);
    }
  }

  void FrontierFinder::calcPathLength2(vector<vector<double>> &cost_mat)
  { 
    // DEBUG
    road_map_paths_.clear();
    // DEBUG

    vector<pair<int, int>> id_pairs;
    for(int i = 0; i < adj_nodes_.size(); ++i)
    {
      if(adj_nodes_[i].type_!=2)
        id_pairs.push_back(pair<int, int>(-1, i));
    }
    
    for(int i = 0; i < adj_nodes_.size(); ++i)
    {
      for(int j = 0; j < i; ++j)
      {
        if(adj_nodes_[i].type_==0)
        {
          if(adj_nodes_[j].type_!=2)
            id_pairs.push_back(pair<int, int>(i, j));
        }
        else if(adj_nodes_[i].type_==1)
        {
          if(adj_nodes_[j].type_!=2 && adj_nodes_[i].viewpoint_cluster_id_ != adj_nodes_[j].viewpoint_cluster_id_)
            id_pairs.push_back(pair<int, int>(i, j));
          else if(adj_nodes_[j].type_==2)
          {
            if(adj_nodes_[i].viewpoint_cluster_id_ == adj_nodes_[j].viewpoint_cluster_id_)
            {
              cost_mat[i+1][j+1] = 0;
              cost_mat[j+1][i+1] = 0;
            }
          }
        }
        else if(adj_nodes_[i].type_==2)
        {
          if(adj_nodes_[i].viewpoint_cluster_id_ == adj_nodes_[j].viewpoint_cluster_id_)
          {
            cost_mat[i+1][j+1] = 0;
            cost_mat[j+1][i+1] = 0;
          }
        }
      }    
    }

    vector<double> dists(id_pairs.size());
    #pragma omp parallel shared(id_pairs, dists)
    {
      unordered_map<int, vector<Vector3d>> tmp_paths;
      vector<Vector3d> failed_viewpoints;

      #pragma omp for nowait

      for(int i = 0; i < id_pairs.size(); ++i)
      {
        int thread_id = omp_get_thread_num();
        // int thread_id = 0;
        vector<Vector3d> path;
        Vector3d pos_1, pos_2;
        
        if(id_pairs[i].first == -1)
          pos_1 = current_pos_;
        else
          pos_1 = adj_nodes_.at(id_pairs[i].first).pos_;

        pos_2 = adj_nodes_.at(id_pairs[i].second).pos_;


        bool safe = true;
        Vector3i idx;
        raycasters_[thread_id]->input(pos_1, pos_2);
        while (raycasters_[thread_id]->nextId(idx)) {
          if (sdf_map_->getInflateOccupancy(idx) == 1 || sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN ||
              !sdf_map_->isInBox(idx)) {
            safe = false;
            break;
          }
        }
        if (safe) {
          path = { pos_1, pos_2 };
          ROS_WARN("Search directly in line");
        }


        if((pos_1-pos_2).norm() < 12.0 && path.empty()) // 距离近的用grid map找，远的直接用road map
        {
          Astars_[thread_id]->reset();
          if (Astars_[thread_id]->search(pos_1, pos_2) != Astar3::REACH_END)
          {
            if(id_pairs[i].first == -1)
              ROS_ERROR("[calcPath]:Thread %d find path from cur_pos to viewpoint:%d type:%d in grid map failed", 
                         thread_id, id_pairs[i].second, adj_nodes_.at(id_pairs[i].second).type_);
            else
              ROS_ERROR("[calcPath]:Thread %d find path from viewpoint:%d type:%d to viewpoint:%d type:%d in grid map failed", 
                        thread_id, id_pairs[i].first, adj_nodes_.at(id_pairs[i].first).type_, id_pairs[i].second, adj_nodes_.at(id_pairs[i].second).type_);
            failed_viewpoints.push_back(pos_1);
            failed_viewpoints.push_back(pos_2);

            if(sdf_map_->getInflateOccupancy(pos_1) == 1)
              ROS_ERROR("[calcPath]:start pt x:%0.2f y:%0.2f z:%0.2f in obstacle", pos_1.x(), pos_1.y(), pos_1.z());
            if(!isFarFromOccupied(pos_1, surface_frontier_obstacle_dist_))
              ROS_ERROR("[calcPath]:start pt x:%0.2f y:%0.2f z:%0.2f near obstacle", pos_1.x(), pos_1.y(), pos_1.z());

            if(sdf_map_->getInflateOccupancy(pos_2) == 1)
              ROS_ERROR("[calcPath]:end pt x:%0.2f y:%0.2f z:%0.2f in obstacle", pos_2.x(), pos_2.y(), pos_2.z());
            if(!isFarFromOccupied(pos_2, surface_frontier_obstacle_dist_))
              ROS_ERROR("[calcPath]:end pt x:%0.2f y:%0.2f z:%0.2f near obstacle", pos_2.x(), pos_2.y(), pos_2.z());
            
            if(find_path_twice_)
            {
              Astars_[thread_id]->reset();
              if (Astars_[thread_id]->search(pos_1, pos_2) != Astar3::REACH_END)
                ROS_ERROR("second find failed");
              else
                path = Astars_[thread_id]->getPath();
            }
          }
          else
          {
            path = Astars_[thread_id]->getPath();
            // ROS_WARN("Find path from viewpoint to viewpoint in grid map spend:%0.6f", (end_time-start_time).toSec());
          }
        }
        // gridmap找不到再到roadmap中找  
        if (path.empty())  
        {
          path = getPathInRoadMap(pos_1, pos_2);
          // ROS_WARN("roadmap path point num:%d", path.size());  
        }
        if(!path.empty())
        {
          shortenPath(path, thread_id); // 拉直路径
          double distance = getPathLength(path);  
          dists[i] = distance;
          // DEBUG
          if(road_map_paths_.size() < 10 && thread_id == 0)
          {
            road_map_paths_.push_back(path);
          }
          // DEBUG
        }
        else
        {
          if(id_pairs[i].first == -1)
            ROS_ERROR("[calcPath]:Thread %d find path from cur_pos to viewpoint:%d type:%d in road map failed", 
                        thread_id, id_pairs[i].second, adj_nodes_.at(id_pairs[i].second).type_);
          else
            ROS_ERROR("[calcPath]:Thread %d find path from viewpoint:%d type:%d to viewpoint:%d type:%d in road map failed", 
                      thread_id, id_pairs[i].first, adj_nodes_.at(id_pairs[i].first).type_, id_pairs[i].second, adj_nodes_.at(id_pairs[i].second).type_);
          dists[i] = 100000;
          failed_viewpoints.push_back(pos_1);
          failed_viewpoints.push_back(pos_2);

          if(sdf_map_->getInflateOccupancy(pos_1) == 1)
            ROS_ERROR("[calcPath]:pos_1 in obstacle");
          if(!isFarFromOccupied(pos_1, surface_frontier_obstacle_dist_))
            ROS_ERROR("[calcPath]:pos_1 near obstacle");

          if(sdf_map_->getInflateOccupancy(pos_2) == 1)
            ROS_ERROR("[calcPath]:pos_2 in obstacle");
          if(!isFarFromOccupied(pos_2, surface_frontier_obstacle_dist_))
            ROS_ERROR("[calcPath]:pos_2 near obstacle");
        }
      }


      #pragma omp barrier

      #pragma omp critical
      {
        failed_viewpoints_.insert(failed_viewpoints_.end(), failed_viewpoints.begin(), failed_viewpoints.end());
      }

      
    }

    double sequence_cost = 0;
    if(use_sequence_cost_)
    {
      sequence_cost = min(total_sequence_cost_/node_pos_sequence_.size(), single_sequence_cost_);
      ROS_WARN("node_pos_sequence_.size:%d  total_sequence_cost_/node_pos_sequence_.size:%0.2f" ,node_pos_sequence_.size(), total_sequence_cost_/node_pos_sequence_.size());
      ROS_WARN("sequence_cost:%0.2f", sequence_cost); 
    }
    else
      sequence_cost = 0;

  
    // 将路径距离写入cost矩阵
    for(int i = 0; i < id_pairs.size(); i++)
    {
      if(id_pairs[i].first == -1)
      {
        cost_mat[0][id_pairs[i].second+1] = dists[i];
        cost_mat[id_pairs[i].second+1][0] = 0;
      }
      else
      {
        cost_mat[id_pairs[i].first+1][id_pairs[i].second+1] = dists[i] + sequence_cost;
        cost_mat[id_pairs[i].second+1][id_pairs[i].first+1] = dists[i] + sequence_cost;

        auto &pos_1 = adj_nodes_.at(id_pairs[i].first).pos_;
        auto &pos_2 = adj_nodes_.at(id_pairs[i].second).pos_;

        if(node_pos_sequence_.find(pos_1) != node_pos_sequence_.end())
        {
          if(node_pos_sequence_.at(pos_1) == pos_2)
          {
            cost_mat[id_pairs[i].first+1][id_pairs[i].second+1] -= sequence_cost;
            // if(use_sequence_cost_)
            //   ROS_WARN("Found history sequence");
          }

        }

        if(node_pos_sequence_.find(pos_2) != node_pos_sequence_.end())
        {
          if(node_pos_sequence_.at(pos_2) == pos_1)
          {
            cost_mat[id_pairs[i].second+1][id_pairs[i].first+1] -= sequence_cost;
            // if(use_sequence_cost_)
            //   ROS_WARN("Found history sequence");
          }

        }
      }
    }
  }


  void FrontierFinder::local_planning()
  {
    // unordered_map<int, unordered_set<int>> local_frontiers;
    // unordered_set<int> local_frontier_clusters_;
    // unordered_map<int, pair<Vector3d, unordered_set<int>>> local_viewpoints;

    double local_range = 40.0;

    Vector3d outside_viewpoint = adj_nodes_[tsp_index_.back()].pos_;

    local_frontiers_.clear();
    local_frontier_clusters_.clear();
    local_viewpoints_.clear();

    int id = 0;
    int last_index = -1;
    
    bool reach_boundary = false;

    ROS_WARN("Find local viewpoints and frontiers");
    
    for(int i = 0; i < tsp_index_.size(); i++)
    {
      auto &adj_node = adj_nodes_[tsp_index_[i]];
      if(adj_node.type_ != 2)
      {
        if((adj_node.pos_-current_pos_).norm() < local_range)
        {
          last_index = i;
          if(adj_node.type_ == 0)
          {
            local_frontier_clusters_.insert(adj_node.viewpoint_id_);
          }
          else
          {
            const auto &viewpoint_cluster = viewpoint_clusters_.at(adj_node.viewpoint_cluster_id_);
            for(const auto &viewpoint_id:viewpoint_cluster.cells_)
            {
              if((viewpoints_.at(viewpoint_id).pos_-current_pos_).norm() < local_range)
              {
                local_frontier_clusters_.insert(viewpoint_id);
              }
              else
              {
                reach_boundary = true;
              }
            }
            if(reach_boundary)
              break;
          }
        }
        else
        {
          reach_boundary = true;
          break;
        }
      }
    }

    ROS_WARN("Find outside viewpoint");

    if((last_index == -1))
    {
      outside_viewpoint = adj_nodes_[tsp_index_.front()].pos_;
    }
    else if(last_index == tsp_index_.size()-1)
    {
      outside_viewpoint = adj_nodes_[tsp_index_.back()].pos_;
    }
    else 
    {
      last_index++;
      while(last_index < tsp_index_.size())
      {
        if(last_index == tsp_index_.size()-1)
        {
          outside_viewpoint = adj_nodes_[tsp_index_.back()].pos_;
          break;
        }
        else if(adj_nodes_[tsp_index_[last_index]].type_ != 2)
        {
          outside_viewpoint = adj_nodes_[tsp_index_[last_index]].pos_;
          break;
        }
        else
        {
          last_index++;
        }
      }
    }


    ROS_WARN("Generate viewpoint and frontier relation");

    for(auto cluster_id:local_frontier_clusters_)
    {
      auto &cluster = surface_frontier_clusters_.at(cluster_id);

      for(auto &frontier:cluster.cells_)
      {
        local_frontiers_[frontier.first];
      }
      // unordered_map<int, pair<Vector3d, unordered_set<int>>> local_;

      for(auto &viewpoint:cluster.viewpoints_)
      {
        pair<Vector3d, unordered_set<int>> viewpoint_pair(viewpoint, std::unordered_set<int>());
        local_viewpoints_[id] = viewpoint_pair;
        id++;
      }
    
    }

    vector<pair<int, int>> priority_queue;

    for(auto &iter:local_viewpoints_)
    {
      auto &viewpoint = iter.second;
 
      for(auto &frontier:local_frontiers_)
      {
        if(isVisible(viewpoint.first, sdf_map_->AddressToPos(frontier.first)))
        {
          viewpoint.second.insert(frontier.first);
          frontier.second.insert(iter.first);
        }
      }

      priority_queue.push_back(pair<int, int>(viewpoint.second.size(), iter.first));
    }

    selected_viewpoints_.clear();
    local_tsp_index_.clear();

    ROS_WARN("Start find coverage points");

    int min_cost = INT32_MAX;
    int K = 5;
    for(int round = 0; round < K; round++)
    {
      vector<pair<int, int>> queue_copy = priority_queue;
      unordered_map<int, unordered_set<int>> local_frontiers_copy = local_frontiers_;
      unordered_map<int, pair<Vector3d, unordered_set<int>>> local_viewpoints_copy = local_viewpoints_;
      vector<Vector3d> selected_viewpoints;
      vector<vector<int>> selected_viewpoints_attach_frontiers;

      random_device rd;
      mt19937 gen(rd());
    
      while(true)
      {
        sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);

        if (queue_copy.empty() || (queue_copy.at(0).first < 8))
        {
          break;
        }

        ROS_WARN("Sort queue finished");

        int sample_range = 0;
        for (int i = 0; i < queue_copy.size(); i++)
        {
          if (queue_copy.at(i).first >= 8)
          {
            sample_range++;
          }
        }

        sample_range = min(5, sample_range);
        std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
        int queue_idx = gen_next_queue_idx(gen);
        int cur_ind = queue_copy.at(queue_idx).second;
        // 将该vp能覆盖的所有frontier标记为已覆盖
        unordered_set<int> covered_frontiers = local_viewpoints_copy.at(cur_ind).second;

        ROS_WARN("Random index:%d   Queue size:%d   viewpoint_ind:%d  frontier num:%d", queue_idx, queue_copy.size(), cur_ind, covered_frontiers.size());

        selected_viewpoints.push_back(local_viewpoints_copy.at(cur_ind).first);
        
        vector<int> attached_frontiers(covered_frontiers.begin(), covered_frontiers.end());
        selected_viewpoints_attach_frontiers.push_back(attached_frontiers);
        
        queue_copy.erase(queue_copy.begin() + queue_idx);

        ROS_WARN("Update queue.");
        // 更新优先级队列
        for (int i = 0; i < queue_copy.size(); i++)
        {
          int viewpoint_id = queue_copy[i].second;
          
          auto &viewpoint = local_viewpoints_copy.at(viewpoint_id);

          for(const auto &frontier:covered_frontiers)
            viewpoint.second.erase(frontier);

          queue_copy[i].first = viewpoint.second.size();
        }
      }

      int tour_point_num = selected_viewpoints.size();
      ROS_WARN("local tour_point_num:%d", tour_point_num);

      if(tour_point_num > 0)
      {
        vector<vector<double>> cost_mat(tour_point_num+2, vector<double>(tour_point_num+2, 100000));

        for(int i = 0; i < tour_point_num; i++)
        {
          for(int j = 0; j < i; j++)
          {
            vector<Vector3d> path;
            if(getPath(selected_viewpoints.at(i), selected_viewpoints.at(j), 30.0, path))
            {
              double distance = getPathLength(path);
              cost_mat[i+1][j+1] = distance;
              cost_mat[j+1][i+1] = distance;
            }
            else
            {

            }

          }
        }
        
        for(int i = 0; i < tour_point_num; i++)
        {
          vector<Vector3d> path;
          if(getPath(current_pos_, selected_viewpoints.at(i), 30.0, path))
          {
            double distance = getPathLength(path);

            Vector3d diff_vector = selected_viewpoints.at(i) - current_pos_;

            double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));

            double cost = angle_rad * w_dir_ + distance; // 效果好 4.0

            cost_mat[0][i+1] = cost;
          }
          else
          {

          }
        }

        for(int i = 0; i < tour_point_num; i++)
        {
          vector<Vector3d> path;
          if(getPath(selected_viewpoints.at(i), outside_viewpoint, 30.0, path))
          {
            double distance = getPathLength(path);

            cost_mat[i+1][tour_point_num+1] = distance;
          }
          else
          {

          }
        }
        cost_mat[tour_point_num+1][0] = 0;

        ROS_WARN("Solve local tsp dim:%d", cost_mat.size());
        int cost = INT32_MAX;
        vector<int> local_tsp_index = sovleTSP(cost_mat, cost);

        for(auto id:local_tsp_index_)
          ROS_WARN("local tsp id:%d", id);

        if(cost < min_cost)
        {
          selected_viewpoints_.clear();
          selected_viewpoints_attach_frontiers_.clear();
          local_tsp_index_.clear();

          selected_viewpoints_ = selected_viewpoints;
          selected_viewpoints_attach_frontiers_ = selected_viewpoints_attach_frontiers;
          
          local_tsp_index_ = local_tsp_index;
          min_cost = cost;
        }

        ROS_WARN("Round %d  cost:%d  min_cost:%d", round, cost, min_cost);
      }

    }

    local_tsp_index_.push_back(selected_viewpoints_.size());
    selected_viewpoints_.push_back(outside_viewpoint);

    vector<int> attached_frontiers;
    selected_viewpoints_attach_frontiers_.push_back(attached_frontiers);
    // else
    // {
    //   local_tsp_index_.clear();
    //   local_tsp_index_.push_back(0);
    // }

    ordered_tour_points_.clear();
    ordered_tour_points_frontiers_.clear();

    for(auto &index:local_tsp_index_)
    {
      ordered_tour_points_.push_back(selected_viewpoints_.at(index));
      vector<int> frontiers;
      ordered_tour_points_frontiers_.push_back(frontiers);
    }
 
    global_planning_update_ = true;

  }

  vector<int> FrontierFinder::sovleTSP(vector<vector<double>> &cost_mat, int &cost)
  {
    const int dimension = cost_mat.size();
    // ROS_WARN("TSP dimension:%d", dimension);

    // Write params and cost matrix to problem file
    ofstream prob_file(tsp_dir_ + "/new_tsp.tsp");
    // Problem specification part, follow the format of TSPLIB
    string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                      "\nEDGE_WEIGHT_TYPE : "
                      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;
    // Problem data part
    const int scale = 100;
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        int int_cost = cost_mat[i][j] * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
    prob_file << "EOF";
    prob_file.close();
    // Call LKH TSP solver
    solveTSPLKH((tsp_dir_ + "/new_tsp.par").c_str());
    // Read optimal tour from the tour section of result file
    ifstream res_file(tsp_dir_ + "/new_tsp.txt");
    string res;


    // 按行读取文件内容
    while(getline(res_file, res))
    {
      // 查找包含 "Length =" 的行
      if (res.find("Length =") != string::npos) 
      {
        // 提取 "Length =" 后面的值
        size_t equal_pos = res.find("=");
        if (equal_pos != string::npos) {
            string value_str = res.substr(equal_pos + 1); // 提取等号后面的部分
            cost = stoi(value_str); // 转换为整数
            break; // 找到后直接退出循环
        }
      }
    }

    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    vector<int> tsp_index;
    // Read path for ATSP formulation
    while (getline(res_file, res))
    {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      // ROS_WARN("id:%d", id);
      if (id == 1) // Ignore the current state
        continue;
      if (id == -1)
        break;
      tsp_index.push_back(id - 2); // Idx of solver-2 == Idx of frontier
    }
    
    res_file.close();
    return tsp_index;
  }

  bool FrontierFinder::getFutureStatus(double time, Vector3d &pos, Vector3d &vel)
  {
    if(cur_traj_.poses.empty())
      return false;
    double start_time = cur_traj_.header.stamp.toSec();
    double forward_time = ros::Time::now().toSec() + time / 1000.0;
    ROS_WARN("start_time:%0.3f  forward time:%0.3f  diff time:%0.3f", start_time, forward_time, forward_time-start_time);
    int waypoint_id = (forward_time-start_time)/0.02;
    waypoint_id = min(waypoint_id, int(cur_traj_.poses.size())-1);

    pos.x() = cur_traj_.poses[waypoint_id].pose.position.x;
    pos.y() = cur_traj_.poses[waypoint_id].pose.position.y;
    pos.z() = cur_traj_.poses[waypoint_id].pose.position.z;

    vel.x() = cur_traj_.poses[waypoint_id].pose.orientation.x;
    vel.y() = cur_traj_.poses[waypoint_id].pose.orientation.y;
    vel.z() = cur_traj_.poses[waypoint_id].pose.orientation.z;
    return true;
  }

  // 优化第一个视点的运动方向变化
  void FrontierFinder::viewpointRefine()
  {
    ordered_tour_points_.clear();
    ordered_tour_points_frontiers_.clear();

    int points_num = min((int)tsp_index_.size(), 5);

    for(int i = 0; i < points_num; i++)
    {
      auto &adj_node = adj_nodes_[tsp_index_[i]];
      if(adj_node.type_ == 2)
        continue;
      else
      {
        if(adj_node.type_ == 0)
        {
          // 老报错，先不用了
          // ROS_WARN("[viewpoint refine]:adj_node.type_ == 0");
          if(use_local_refine_)
          {
            if(surface_frontier_clusters_.count(adj_node.viewpoint_id_))
            {
              auto & cluster = surface_frontier_clusters_.at(adj_node.viewpoint_id_); // 会出问题
              // ROS_WARN("[viewpoint refine]:foreach viewpoint");

              if(i == 0 && cluster.max_gain_ > 2)
              {
                double gain_thres = cluster.max_gain_ * 0.9;
                double min_angle_index = -1; 
                double min_rad = DBL_MAX;
                for(int j = 0; j < cluster.viewpoints_.size(); j++)
                {
                  // ROS_WARN("Viewpoint  x:%0.2f  y:%0.2f  z:%0.2f", cluster.viewpoints_.at(j).x(), cluster.viewpoints_.at(j).y(), cluster.viewpoints_.at(j).z());
                  if(cluster.viewpoint_gains_.at(j) > gain_thres)
                  {
                    vector<Vector3d> path;
                    if(getPath(current_pos_, cluster.viewpoints_.at(j), -1, path))
                    {
                      // ROS_WARN("Greater than gain_thres");
                      Vector3d diff_vector = cluster.viewpoints_.at(j) - current_pos_;
                      double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));
                      if(angle_rad < min_rad)
                      {
                        min_rad = angle_rad;
                        min_angle_index = j;
                      }
                    }
                  }
                }
                if(min_angle_index!=-1)
                {
                  ordered_tour_points_.push_back(cluster.viewpoints_.at(min_angle_index));
                  adj_nodes_[tsp_index_[i]].pos_ = cluster.viewpoints_.at(min_angle_index);
                }
                else
                  ordered_tour_points_.push_back(adj_node.pos_);
                // ROS_WARN("First viewpoint  x:%0.2f  y:%0.2f  z:%0.2f", (*ordered_tour_points_.begin()).x(), (*ordered_tour_points_.begin()).y(), (*ordered_tour_points_.begin()).z());
              }
              else
              {
                ordered_tour_points_.push_back(adj_node.pos_);
              }
            }
            else
            {
              // ROS_WARN("[viewpoint refine]: invalid frontier_cluster_id"); 
              ordered_tour_points_.push_back(adj_node.pos_);
            }
          }
          else 
            ordered_tour_points_.push_back(adj_node.pos_);
          
          // ROS_WARN("[viewpoint refine]:add frontier info");
          vector<int> frontiers;
          if(viewpoints_.find(adj_node.viewpoint_id_) != viewpoints_.end())
          {
            int frontier_cluster_id = viewpoints_.at(adj_node.viewpoint_id_).frontier_cluster_id_;
            if(surface_frontier_clusters_.find(frontier_cluster_id) != surface_frontier_clusters_.end())
            {
              for(auto &cell:surface_frontier_clusters_.at(frontier_cluster_id).covered_cells_)
                frontiers.push_back(cell);
            }
            // else
              // ROS_WARN("[viewpoint refine]: invalid frontier_cluster_id"); 
          }
          // else
            // ROS_WARN("[viewpoint refine]: invalid viewpoint_id_");

          ordered_tour_points_frontiers_.push_back(frontiers);
        }
        else
        {
          // ROS_WARN("[viewpoint refine]:adj_node.type_ != 0");
          ordered_tour_points_.push_back(adj_node.pos_);
          vector<int> frontiers;
          ordered_tour_points_frontiers_.push_back(frontiers);
        }
      }
    }
    global_planning_update_ = true;
  }

  // 优化第一个视点的运动方向变化
  void FrontierFinder::viewpointRefine2()
  {
    ordered_tour_points_.clear();
    ordered_tour_points_frontiers_.clear();
    auto &first_adj_node = adj_nodes_[tsp_index_[0]];
    if(first_adj_node.type_ == 0) // 为视点
    {
      auto & frontier_cluster = surface_frontier_clusters_.at(first_adj_node.viewpoint_id_);
      ROS_WARN("Get min distance id");
      // 寻找距离聚类中心最近的cell作为seed
      double min_distance = DBL_MAX;
      int min_id = frontier_cluster.cells_.begin()->first;
      Vector3d avg_pos = frontier_cluster.average_pos_;
      for(auto iter:frontier_cluster.cells_)
      {
        Vector3d cell_pos = sdf_map_->AddressToPos(iter.first);
        if((cell_pos-avg_pos).norm() < min_distance)
        {
          min_distance = (cell_pos-avg_pos).norm();
          min_id = iter.first;
        }
      }
      ROS_WARN("Init cluster");
      int seed = min_id;
      // Data for clustering
      queue<int> queue;              // region-grow队列
      unordered_set<int> expanded;               // 扩展的cell
      expanded.insert(seed);
      queue.push(seed);

      Vector3d avg_normal(404, 404, 404);  // 聚类的平均法向量
      Vector3d toal_normal(0, 0, 0);

      Vector3d total_pos = sdf_map_->AddressToPos(seed);

      int normal_num = 0;
      if(surface_frontiers_.count(seed)) // 可能找不到法向量
      {
        if(surface_frontiers_.at(seed).x()!=404)
        {
          toal_normal+=surface_frontiers_.at(seed);
          normal_num++;
          avg_normal = toal_normal / normal_num;
        }
      }
      ROS_WARN("Start cluster");
      // Search frontier cluster based on region growing (distance clustering)
      while (!queue.empty())
      {
        auto cur = queue.front();
        queue.pop();
        auto nbrs = allNeighbors(cur);
        for (auto nbr : nbrs)
        {
          int adr = toadr(nbr); 
          if (!sdf_map_->isInBox(nbr))  // Qualified cell should be inside bounding box
            continue;
          // 未扩展过且为surface_frontier
          if(expanded.find(adr)==expanded.end() && isSurfaceFrontier(adr))
          {  
            Vector3d nbr_pos = sdf_map_->AddressToPos(adr);
            ROS_WARN("Get nbr normal");
            Vector3d nbr_normal(404, 404, 404);

            if(surface_frontiers_.count(adr)) // 可能找不到法向量
              nbr_normal = surface_frontiers_.at(adr);

            ROS_WARN("Get nbr normal success");
            double diff_angle = 0;
            if(nbr_normal.x()!=404 && avg_normal.x()!=404)
            {
              double angle_rad = acos(nbr_normal.dot(avg_normal) / (nbr_normal.norm() * avg_normal.norm()));
              diff_angle = angle_rad * 180.0 / M_PI;
            }
            if(diff_angle < 40.0)
            {
              bool out_of_range = false;
              Vector3d new_avg_pos = (total_pos + nbr_pos)/(expanded.size()+1);
              for(const auto &cell:expanded)
              {
                if((new_avg_pos-sdf_map_->AddressToPos(cell)).norm() > 10.0)
                {
                  out_of_range = true;
                  break;
                }
              }

              if((new_avg_pos-nbr_pos).norm() > 10.0)
                out_of_range = true;
 
              if(!out_of_range)
              {
                if(nbr_normal.x()!=404)
                {
                  toal_normal+=nbr_normal;
                  normal_num++;
                  avg_normal=toal_normal/normal_num;
                }
                total_pos+=nbr_pos;
                queue.push(adr);
                expanded.insert(adr);
              }
            }
          }
        }
      }
      ROS_WARN("Stop cluster");
      local_surface_frontiers_.clear();
      for(const auto &cell:expanded)
      {
        if(surface_frontiers_.count(cell))
          local_surface_frontiers_[cell] = surface_frontiers_.at(cell);
        else
          local_surface_frontiers_[cell] = Vector3d(404, 404, 404);
      }

      SurfaceFrontierCluster refine_cluster;
      refine_cluster.cells_ = local_surface_frontiers_;
      refine_cluster.id_ = -100;

      computeSurfaceFrontierClusterInfo(refine_cluster);

      sampleSurfaceFrontierViewpoints(refine_cluster);

      if(refine_cluster.can_be_observed_)
      {
        double gain_thres = refine_cluster.max_gain_ * 0.9;
        double min_angle_index = -1; 
        double min_rad = DBL_MAX;
        for(int i = 0; i < refine_cluster.viewpoints_.size(); i++)
        {
          // ROS_WARN("Viewpoint  x:%0.2f  y:%0.2f  z:%0.2f", refine_cluster.viewpoints_.at(i).x(), refine_cluster.viewpoints_.at(i).y(), refine_cluster.viewpoints_.at(i).z());
          if(refine_cluster.viewpoint_gains_.at(i) > gain_thres)
          {
            // ROS_WARN("Greater than gain_thres");
            Vector3d diff_vector = refine_cluster.viewpoints_.at(i) - current_pos_;
            double angle_rad = acos(forward_directory_.dot(diff_vector) / (forward_directory_.norm() * diff_vector.norm()));
            if(angle_rad < min_rad)
            {
              min_rad = angle_rad;
              min_angle_index = i;
            }
          }
        }
  
        ordered_tour_points_.push_back(refine_cluster.viewpoints_.at(min_angle_index));
        first_adj_node.pos_ = refine_cluster.viewpoints_.at(min_angle_index);
        vector<int> frontiers;
        for(auto &cell:refine_cluster.covered_cells_)
          frontiers.push_back(cell);
        ordered_tour_points_frontiers_.push_back(frontiers);

        // ROS_WARN("First viewpoint  x:%0.2f  y:%0.2f  z:%0.2f", (*ordered_tour_points_.begin()).x(), (*ordered_tour_points_.begin()).y(), (*ordered_tour_points_.begin()).z());
      }
      else
      {
        ordered_tour_points_.push_back(first_adj_node.pos_);
        vector<int> frontiers;
        for(auto &cell_iter:frontier_cluster.cells_)
          frontiers.push_back(cell_iter.first);
        ordered_tour_points_frontiers_.push_back(frontiers);
      }
    } 
    else if(first_adj_node.type_ == 1) // 为视点聚类
    {
      ordered_tour_points_.push_back(first_adj_node.pos_);
      vector<int> frontiers;
      ordered_tour_points_frontiers_.push_back(frontiers);
    }
    global_planning_update_ = true;
  }

  bool FrontierFinder::getPathInGridMap(const Vector3d &start_point, const Vector3d &end_point, vector<Vector3d>& path)
  {
    Astar_->reset();
    path.clear();
    if (Astar_->search(start_point, end_point) == Astar3::REACH_END)
    {
      path = Astar_->getPath();
      return true;
    }
    else
      return false;
  }

  vector<Vector3d> FrontierFinder::getPathInRoadMap(const Vector3d &start_point, const Vector3d &end_point)
  {
    utils::Point3D start(start_point.x(), start_point.y(), start_point.z()); 
    utils::Point3D end(end_point.x(), end_point.y(), end_point.z()); 
    utils::Point3D nearest_point;
    vector<Vector3d> path;
    int start_id, end_id;
    
    // ros::Time t1 = ros::Time::now();
    if(!road_map_->nearestSearch(start_point, nearest_point, start_id))
    {
      ROS_ERROR("[getPathInRoadMap]: start point nearestSearch failed");
      ROS_ERROR("start_pt x:%0.2f y:%0.2f z:%0.2f", start_point.x(), start_point.y(), start_point.z());
      return path;
    }
      
    // ROS_WARN("Start_point id:%d  x:%f y:%f z:%f", start_id, nearest_point.x(), start_id, nearest_point.y(), start_id, nearest_point.z());
    
    if(!road_map_->nearestSearch(end_point, nearest_point, end_id))
    {
      ROS_ERROR("[getPathInRoadMap]: end point nearestSearch failed");
      ROS_ERROR("end_pt x:%0.2f y:%0.2f z:%0.2f", end_point.x(), end_point.y(), end_point.z());
      return path;
    }
      
    // ROS_WARN("End_point id:%d  x:%f y:%f z:%f", end_id, nearest_point.x(), start_id, nearest_point.y(), start_id, nearest_point.z());
    // ros::Time t2 = ros::Time::now();
    // ROS_WARN("Roadmap nearest search spend:%f", (t2-t1).toSec());

    ros::Time t1 = ros::Time::now();
    road_map_->findShortestPath(start_id, end_id, path);
    ros::Time t2 = ros::Time::now();

    if(path.empty())
    {
      ROS_ERROR("[getPathInRoadMap]: Roadmap find path failed");
      int start_edge_num = road_map_->graph_.getLinkNums(start_id);
      int end_edge_num = road_map_->graph_.getLinkNums(end_id);
      // ROS_WARN("start_edge_num:%d    end_edge_num:%d", start_edge_num, end_edge_num);
      ROS_ERROR("time:%0.4f", (t2-t1).toSec()*1000.0);
      ROS_ERROR("(%0.2f: %0.2f: %0.2f)->(%0.2f: %0.2f: %0.2f)", start_point.x(), start_point.y(), start_point.z(),  end_point.x(), end_point.y(), end_point.z());
    }
    // ROS_WARN("Roadmap graph search spend:%f", (t1-t2).toSec());

    return path;
  }

  bool FrontierFinder:: getPath(const Vector3d &start_point, const Vector3d &end_point, double road_map_thres, vector<Vector3d>& path)
  {
    path.clear();
    if((start_point-end_point).norm() < road_map_thres || road_map_thres < 0) // 距离近的用grid map找，远的直接用road map
    {
      Astar_->reset();
      if (Astar_->search(start_point, end_point) != Astar3::REACH_END)
        ROS_ERROR("[getPath]:Find path in grid map failed");
      else
        path = Astar_->getPath();
    }
    if (path.empty() && road_map_thres > 0) // gridmap找不到到roadmap中找
      path = getPathInRoadMap(start_point, end_point);

    if (!path.empty())
      return true;
    else
    {
      ROS_ERROR("[getPath]:Find path failed");
      return false;
    }
  }

  void FrontierFinder::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }

    // 先对间距过大点进行插值
    vector<Vector3d> new_path;

    int threshold = sdf_map_->getResolution() * 1.8;
    for (size_t i = 0; i < static_cast<int>(path.size()) - 1; ++i) 
    {
      Vector3d cur = path[i];
      Vector3d next = path[i + 1];
      double distance = (next - cur).norm();

      new_path.push_back(cur);

      if (distance > threshold) 
      {
        int num_points = static_cast<int>(distance / threshold);
        for (int j = 1; j <= num_points; ++j) 
        {
            double t = static_cast<double>(j) / (num_points + 1); 
            Vector3d interpolated_point = (1.0 - t) * cur + t * next;
            new_path.push_back(interpolated_point);
        }
      }
    }
    // 将最后一个点加入新路径
    if (!path.empty()) 
      new_path.push_back(path.back());

    // 拉直路径
    vector<Vector3d> short_tour = {new_path.front()};
    for (int i = 1; i < int(new_path.size()) - 1; ++i)
    {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), new_path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok())
      {
        if (sdf_map_->getInflateOccupancy(idx) == 1 ||
            sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
        {
          short_tour.push_back(new_path[i]);
          break;
        }
      }
    }
    // 加上终点point
    if ((new_path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(new_path.back());
    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));

    path = short_tour;
  }

  // 多线程时使用
  void FrontierFinder::shortenPath(vector<Vector3d> &path, int thread_id)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }

    // 先对间距过大点进行插值
    vector<Vector3d> new_path;

    int threshold = sdf_map_->getResolution() * 1.8;
    for (size_t i = 0; i < static_cast<int>(path.size()) - 1; ++i) 
    {
      Vector3d cur = path[i];
      Vector3d next = path[i + 1];
      double distance = (next - cur).norm();

      new_path.push_back(cur);

      if (distance > threshold) 
      {
        int num_points = static_cast<int>(distance / threshold);
        for (int j = 1; j <= num_points; ++j) 
        {
            double t = static_cast<double>(j) / (num_points + 1); 
            Vector3d interpolated_point = (1.0 - t) * cur + t * next;
            new_path.push_back(interpolated_point);
        }
      }
    }
    // 将最后一个点加入新路径
    if (!path.empty()) 
      new_path.push_back(path.back());
      
    // 拉直路径
    vector<Vector3d> short_tour = {new_path.front()};
    for (int i = 1; i < int(new_path.size()) - 1; ++i)
    {
      // Add waypoints to shorten path only to avoid collision
      raycasters_[thread_id]->input(short_tour.back(), new_path[i + 1]);
      Eigen::Vector3i idx;
      while (raycasters_[thread_id]->nextId(idx) && ros::ok())
      {
        if (sdf_map_->getInflateOccupancy(idx) == 1 ||
            sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
        {
          short_tour.push_back(new_path[i]);
          break;
        }
      }
    }
    // 加上终点point
    if ((new_path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(new_path.back());
    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));

    path = short_tour;
  }
  
  double FrontierFinder::getPathLength(const vector<Vector3d>& path)
  {
    double distance = 0;
    for(int i = 0; i < static_cast<int>(path.size())-1; i++)
      distance += (path[i+1]-path[i]).norm();

    return distance;
  }

  Vector3d FrontierFinder::get_velocity(vector<Eigen::MatrixXd> &p, double t)
  {
      Vector3d vel;
      for(int i = 0; i < 3; i++)
      {
          vel[i] = p[i](0) * pow(t, 4) * 5 + p[i](1) * pow(t, 3) * 4 + p[i](2) * pow(t, 2) * 3 + p[i](3) * t * 2 + p[i](4);
      }
      return vel;
  }

  // bool FrontierFinder::addPosToGraph(const Vector3d pos)
  // {
  //   int point_id = -1;
  //   preprocess::Point3D point(pos[0], pos[1], pos[2]);
  //   bool result = false;

  //   if (!road_map_->getPointId(point, point_id))
  //   {
  //     double local_range = 2.0;
  //     preprocess::Point3DQueue neighbor_vertexs;
  //     std::vector<int> neighbor_vertex_ids;
      
  //     road_map_->nearRangeSearch(point, local_range,
  //                                 neighbor_vertexs, neighbor_vertex_ids);

  

  //     point_id = road_map_->addVertex(pos);

  //     for (const auto &item : neighbor_vertexs)
  //     {
  //       if (map_3d_manager_->isCollisionFreeStraight(
  //               preprocess::GridPoint3D(pos.x(), pos.y(), pos.z()),
  //               preprocess::GridPoint3D(item.x(), item.y(), item.z())))
  //       {
  //         int item_id = -1;
  //         if (point_id != item_id && road_map_->getPointId(item, item_id))
  //         {
  //             road_map_->addTwoWayEdge(point_id, item_id);
  //             result =  true;
  //         }
  //       }
  //     }
  //   }
  //   return result;
  // }

  void FrontierFinder::sample_traj(vector<Eigen::MatrixXd> &p, double t, vector<Vector3d> &pos_list, vector<Vector3d> &vel_list, vector<Vector3d> &acc_list, vector<double> &time_list)
  {
      Vector3d pos, vel, acc;

      pos_list.clear();
      vel_list.clear();
      acc_list.clear();
      time_list.clear();

      for(double cur_t = 0; cur_t < t; cur_t+=0.01)
      {
          double cur_t_5 = pow(cur_t, 5);
          double cur_t_4 = pow(cur_t, 4);
          double cur_t_3 = pow(cur_t, 3);
          double cur_t_2 = pow(cur_t, 2);
          double cur_t_1 = pow(cur_t, 1);

          for(int i = 0; i < 3; i++)
          {
              pos[i] = p[i](0) * cur_t_5 + p[i](1) * cur_t_4 + p[i](2) * cur_t_3 + p[i](3) * cur_t_2 + p[i](4) * cur_t_1 + p[i](5);
              vel[i] = p[i](0) * cur_t_4 * 5 + p[i](1) * cur_t_3 * 4 + p[i](2) * cur_t_2 * 3 + p[i](3) * cur_t_1 * 2 + p[i](4);
              acc[i] = p[i](0) * cur_t_3 * 20 + p[i](1) * cur_t_2 * 12 +  p[i](2) * cur_t_1 * 6 + p[i](3) * 2;
          }
          pos_list.push_back(pos);
          vel_list.push_back(vel);
          acc_list.push_back(acc);
          time_list.push_back(cur_t);
      }
  }

  void FrontierFinder::jerk_traj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc, Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                double t, vector<Eigen::MatrixXd> &p, vector<double> &jerk_integrals)
  {
    vector<Eigen::MatrixXd> A(3), b(3), Q(3);
    p.clear();
    p.resize(3);
    jerk_integrals.clear();

    for(int i = 0; i < 3; i++)
    {
        A[i] = Eigen::MatrixXd::Zero(6, 6); 
        b[i] = Eigen::MatrixXd::Zero(6, 1);
        Q[i] = Eigen::MatrixXd::Zero(6, 6); 

        A[i](0, 5) = 1;
        A[i](1, 4) = 1;
        A[i](2, 3) = 2;
        A[i](3, 0) = pow(t, 5), A[i](3, 1) = pow(t, 4), A[i](3, 2) = pow(t, 3), A[i](3, 3) = pow(t, 2), A[i](3, 4) = t, A[i](3, 5) = 1;
        A[i](4, 0) = pow(t, 4)*5, A[i](4, 1) = pow(t, 3)*4, A[i](4, 2) = pow(t, 2)*3, A[i](4, 3) = t*2, A[i](4, 4) = 1;
        A[i](5, 0) = pow(t, 3)*20, A[i](5, 1) = pow(t, 2)*12, A[i](5, 2) = t*6, A[i](5, 3) = 2;

        b[i](0) = start_pos[i];
        b[i](1) = start_vel[i];
        b[i](2) = start_acc[i];
        b[i](3) = end_pos[i];
        b[i](4) = end_vel[i];
        b[i](5) = end_acc[i];

        Q[i](0, 0) = 720*pow(t, 5);
        Q[i](0, 1) = 360*pow(t, 4);
        Q[i](0, 2) = 120*pow(t, 3);

        Q[i](1, 0) = 360*pow(t, 4);
        Q[i](1, 1) = 192*pow(t, 3);
        Q[i](1, 2) = 72*pow(t, 2);

        Q[i](2, 0) = 120*pow(t, 3);
        Q[i](2, 1) = 72*pow(t, 2);
        Q[i](2, 2) = 36*t;

        p[i] = A[i].colPivHouseholderQr().solve(b[i]);

        double jerk_integral = 720*p[i](0)*p[i](0)*pow(t, 5) + 720*p[i](0)*p[i](1)*pow(t, 4)
                            + (240*p[i](0)*p[i](2)+192*p[i](1)*p[i](1))*pow(t, 3)
                            + 144*p[i](1)*p[i](2)*pow(t, 2) + 36*p[i](2)*p[i](2)*t;
        jerk_integrals.push_back(jerk_integral);
        // cout << "p_5:" << p[i](0) << "  p_4:" << p[i](1) << "  p_3:" << p[i](2) << "  p_2:" << p[i](3) << "  p_1:" << p[i](4) << "  p_0:" << p[i](5) << endl;
    }
  }

  double FrontierFinder::calculateMotionTime(double vel, double acc_max, double vel_max, double x) 
  {
      if(vel > vel_max)
        vel = vel_max;
      if(vel < -vel_max)
        vel = -vel_max;

      double t = 0;
      if(x < 0)
      {
          x = -x;
          vel = -vel;
      }

      if(vel < 0)
      {
        t += abs(vel) / acc_max;
        x += vel*vel/acc_max/2;
        vel = 0;
      }
      

      if(vel*vel/acc_max/2 > x)
      {
          double t1 = vel/acc_max;
          double x1 = vel*vel/acc_max/2;

          double x2 = x1 - x;
          double t2 = sqrt(x2/acc_max*2);
          t = t+ t1 + t2;
      }
      else
      {
          double t1 = (vel_max - vel) / acc_max;  // 加速阶段时间 t1
          double x1 = vel * t1 + 0.5 * acc_max * t1 * t1;    // 加速阶段距离 x1

          double t3 = vel_max / acc_max;  // 减速阶段时间 t3
          double x3 = 0.5 * vel_max * t3;    // 减速阶段距离 x3

          if(x1 + x3 < x)
          {
              // 匀速阶段距离 x2
              double x2 = x - x1 - x3;
              // 匀速阶段时间 t2
              double t2 = x2 / vel_max;
              t = t + t1 + t2 + t3;
          }
          else
          {
              // 如果没有足够的距离达到最大速度
              double v_mid = sqrt((2*acc_max*x+vel*vel)/2);
              t1 = (v_mid - vel)/acc_max;
              t3 = v_mid/acc_max;
              t = t + t1 + t3;
          }
      }
      return t;
  }

  bool FrontierFinder::getTourPoints(vector<Vector3d> &tour_points, vector<vector<int>> &frontiers)
  {
    if(global_planning_update_)
    {
      tour_points = ordered_tour_points_;
      frontiers = ordered_tour_points_frontiers_;
      global_planning_update_ = false;
      return true;
    }
    else
      return false;
  }

  bool FrontierFinder::getTourPoint(Vector3d &tour_point, vector<int> &frontiers)
  {
      tour_point = ordered_tour_points_[0];
      frontiers = ordered_tour_points_frontiers_[0];
      return true;
  }

  bool FrontierFinder::getNextViewpoint(Vector3d &viewpoint, vector<int> &frontiers)
  {
    if(!tsp_index_.empty())
    {
      int cluster_id = index_map_[tsp_index_[0]];
      viewpoint = surface_frontier_clusters_[cluster_id].max_gain_viewpoint_;
      
      for(auto cell:surface_frontier_clusters_[cluster_id].cells_)
      {
        frontiers.push_back(cell.first);
      } 
      return true;
    }
    return false;
  }

  void FrontierFinder::selectViewPoints()
  {
    vector<Vector3d> selected_viewpoint;
    vector<vector<int>> condidtae_viewpoints_covered_frontier;
    vector<Vector3d> condidtae_viewpoints;
    vector<pair<int, int>> queue;
    int idx = 0;
    for (auto frontier_viewpoint:surface_frontier_viewpoint_)
    {
      for(auto viewpoint:frontier_viewpoint.second) // 遍历所有vp
      {
        vector<int> covered_cells;

        for(auto cluster:surface_frontier_clusters_)  // 遍历所有surface_frontier
        {
          if(cluster.second.cells_.size() < surface_frontier_cluster_min_)
            continue;
          for(auto frontier:cluster.second.cells_)
          {
            Vector3d frontier_pos = sdf_map_->AddressToPos(frontier.first);
            if((frontier_pos - viewpoint).norm() < sdf_map_->getMaxRayLength()
             && sdf_map_->isCollisionFreeStraight(frontier_pos, viewpoint))
            {
              covered_cells.push_back(frontier.first);
            }
          }
        }
        condidtae_viewpoints.push_back(viewpoint);
        condidtae_viewpoints_covered_frontier.push_back(covered_cells);
              
        queue.push_back(pair<int, int>(covered_cells.size(), idx));
        ROS_WARN("viewpoint id:%d covers num:%d", idx, covered_cells.size());
        idx++;
      }
    }

    std::vector<std::pair<int, int>> queue_copy;
    for (int i = 0; i < queue.size(); i++)
    {
      queue_copy.push_back(queue[i]);
    }
    sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);
    
    ROS_WARN("Sort queue finished");

    int sample_range;
    for (int i = 0; i < queue_copy.size(); i++)
    {
      if (queue_copy[i].first >= 1)
      {
        sample_range++;
      }
    }

    // sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
    sample_range = min(3, sample_range);
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
    
    int queue_idx = gen_next_queue_idx(gen);
    int cur_ind = queue_copy[queue_idx].second;
    ROS_WARN("Random index:%d   Queue size:%d   cur_ind:%d", queue_idx, queue_copy.size(), cur_ind);
    
    unordered_set<int> covered_frontier;

    while(true)
    {
      // 将该vp能覆盖的所有frontier标记为已覆盖
      for(auto& frontier_ind:condidtae_viewpoints_covered_frontier[cur_ind])
      {
        covered_frontier.insert(frontier_ind);
      }
      selected_viewpoint.push_back(condidtae_viewpoints[cur_ind]);
      queue_copy.erase(queue_copy.begin() + queue_idx);

      // 更新优先级队列
      for (int i = 0; i < queue_copy.size(); i++)
      {
        int covered_frontier_num = 0;
        int ind = queue_copy[i].second;
        
        for (auto& frontier_ind : condidtae_viewpoints_covered_frontier[ind])
        {
          if (covered_frontier.find(frontier_ind) == covered_frontier.end()) // 该frontier没被cover
          {
            covered_frontier_num++;
          }
        }
        queue_copy[i].first = covered_frontier_num;
      }

      sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);

      // 若能覆盖的最大frontier数小于阈值,则结束
      if (queue_copy.empty() || queue_copy[0].first < 3)
      {
        break;
      }

      // 随机选择下一个vp
      sample_range = 0;
      for (int i = 0; i < queue.size(); i++)
      {
        if (queue[i].first >= 1)
        {
          sample_range++;
        }
      }
      sample_range = min(3, sample_range);
      std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
      queue_idx = gen_next_queue_idx(gen);
      cur_ind = queue_copy[queue_idx].second;
    }

    selected_viewpoint_ = selected_viewpoint;
    //..................
  }

  void FrontierFinder::setTargetViewpoint(Vector3d target_viewpoint, std::vector<int> attached_frontiers)
  {
    target_viewpoint_ = target_viewpoint;
    target_viewpoint_attached_frontiers_ = attached_frontiers;
  }

  bool FrontierFinder::SortPairInRev(const std::pair<int, int>& a, const std::pair<int, int>& b)
  {
    return (a.first > b.first);
  }


  int FrontierFinder::getCoveredFrontierNum(unordered_set<int> &covered_cells, vector<int> &can_covered_cells)
  {

  }

  void FrontierFinder::updateForwardDirectory(Vector3d forward_directory)
  {
    forward_directory_ = forward_directory;
  }

  void FrontierFinder::updateObstacleCloud()
  {
    // 找occupied cell
    // for (const auto &change_cell : changed_cells_)
    // {
    //   if(sdf_map_->getOccupancy(change_cell) == SDFMap::OCCUPIED)
    //   {
    //     if(obstacle_cells_.find(change_cell)==obstacle_cells_.end())
    //     {
    //       obstacle_cells_.insert(change_cell);
    //       Vector3d pos = sdf_map_->AddressToPos(change_cell);
    //       if(sdf_map_->isInFrontierBox(pos))
    //       {
    //         pcl::PointXYZ point(pos.x(), pos.y(), pos.z());
    //         obstacle_cloud_.points.push_back(point);
    //       }
    //     }
    //   }
    // }

    // Search new frontier within box slightly inflated from updated box
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max, true);
    Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
    Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
    Vector3d box_min, box_max;

    sdf_map_->getBox(box_min, box_max);
    for (int k = 0; k < 3; ++k) 
    {
      search_min[k] = max(search_min[k], box_min[k]);
      search_max[k] = min(search_max[k], box_max[k]);
    }
    Eigen::Vector3i min_id, max_id;
    sdf_map_->posToIndex(search_min, min_id);
    sdf_map_->posToIndex(search_max, max_id);

    #pragma omp parallel                                                                               \
    shared(min_id, max_id)
    {
      vector<int> need_add;

      #pragma omp for nowait

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z) 
          {
            Eigen::Vector3i cur(x, y, z);
            int addr = sdf_map_->toAddress(cur);
            if(sdf_map_->getOccupancy(addr) == SDFMap::OCCUPIED)
            {
              if(obstacle_cells_.find(addr)==obstacle_cells_.end())
              {
                need_add.push_back(addr);
              }
            }
          }

      #pragma omp barrier

      #pragma omp critical
      {
        obstacle_cells_.insert(need_add.begin(), need_add.end());

        for(auto &adr:need_add)
        {
          Vector3d pos = sdf_map_->AddressToPos(adr);
          if(sdf_map_->isInFrontierBox(pos))
          {
            pcl::PointXYZ point(pos.x(), pos.y(), pos.z());
            obstacle_cloud_.points.push_back(point);
          }
        }
      }
    }
  }

  void FrontierFinder::eraseFrontier(const int &addr)
  {
    global_frontier_cells_.erase(addr);
  }

  void FrontierFinder::eraseFrontier(const Eigen::Vector3i &index)
  {
    global_frontier_cells_.erase(sdf_map_->toAddress(index));
  }

  void FrontierFinder::eraseFrontier(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i index;
    sdf_map_->posToIndex(pos, index);
    global_frontier_cells_.erase(sdf_map_->toAddress(index));
  }

  void FrontierFinder::pubFrontiersMarkers()
  {
    std_msgs::ColorRGBA global_rgba;
    global_rgba.a = 0.8;
    global_rgba.r = 0;
    global_rgba.g = 1;
    global_rgba.b = 0;
    visualization_msgs::Marker global_frontier_cells_marker;
    generateMarkerArray("world", global_frontier_cells_marker, global_frontier_cells_, global_rgba);
    global_frontiers_pub_.publish(global_frontier_cells_marker);
  }

  void FrontierFinder::pubOccupancyCloud()
  {
    // 将PCL点云转换为ROS消息
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*local_occupancy_cloud_, msg);
    msg.header.frame_id = "world";  // 设置合适的坐标系
    msg.header.stamp = ros::Time::now();
    // 发布点云
    local_occupancy_cloud_pub_.publish(msg);
    // ROS_WARN_THROTTLE(0.5, "local_occupancy_cloud size:%d", local_occupancy_cloud_->size());
  }

  void FrontierFinder::pubViewpointWithFrontiers()
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.id = 0;
    // 设置 Marker 的比例（线条宽度）
    line_list.scale.x = 0.1;
    // 设置 Marker 的颜色（绿色）
    line_list.color.r = 0.0;
    line_list.color.g = 0.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    for(auto cluster:surface_frontier_clusters_)
    {
      // if(cluster.second.cells_.size() < surface_frontier_cluster_min_)
      //   continue;
      // if(cluster.second.viewpoints_.empty())
      //   continue;
      if(!cluster.second.can_be_observed_)  
        continue;

      geometry_msgs::Point start;
      start.x = cluster.second.max_gain_viewpoint_.x();
      start.y = cluster.second.max_gain_viewpoint_.y();
      start.z = cluster.second.max_gain_viewpoint_.z();

      for(auto cell:cluster.second.covered_cells_)
      {
          Vector3d pos = sdf_map_->AddressToPos(cell);
          geometry_msgs::Point end;
          end.x = pos.x();
          end.y = pos.y();
          end.z = pos.z();
          line_list.points.push_back(start);
          line_list.points.push_back(end);
      }
    }
    viewpoint_with_frontiers_pub_.publish(line_list);
    
  }

  void FrontierFinder::pubViewPoints()
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "labels";

    text_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(text_marker);

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.scale.z = 1;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> max_gain_cloud;

    int i = 1;
    for(auto &iter:surface_frontier_clusters_)
    {
      auto &cluster = iter.second;
      // ROS_WARN("Cluster id:%d viewpoint num:%d", cluster.second.id_, cluster.second.viewpoints_.size());
      if(cluster.cells_.size() < surface_frontier_cluster_min_)
        continue;
      if(cluster.viewpoints_.empty())
        continue;

      std::ostringstream cover_range;
      cover_range << std::fixed << std::setprecision(2) << cluster.cover_range_;
      // string cover_range = to_string(cluster.second.cover_range_);
      text_marker.id = i++;
      text_marker.text = cover_range.str();
      text_marker.pose.position.x = cluster.max_gain_viewpoint_.x();
      text_marker.pose.position.y = cluster.max_gain_viewpoint_.y();
      text_marker.pose.position.z = cluster.max_gain_viewpoint_.z() + 4.0;
      marker_array.markers.push_back(text_marker);
      
      for(auto &viewpoint:cluster.viewpoints_)
      {
        pcl::PointXYZ point(viewpoint.x(), viewpoint.y(), viewpoint.z());
        cloud.points.push_back(point);
      }
      Vector3d &max_gain_viewpoin = cluster.max_gain_viewpoint_;
      pcl::PointXYZ point(max_gain_viewpoin.x(), max_gain_viewpoin.y(), max_gain_viewpoin.z());
      max_gain_cloud.points.push_back(point);
    }

    viewpoint_coverage_pub_.publish(marker_array);

    if(!cloud.points.empty())
    {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);  
      msg.header.frame_id = "world";  
      viewpoints_pub_.publish(msg);  
    }

    if(!max_gain_cloud.points.empty())
    {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(max_gain_cloud, msg);  
      msg.header.frame_id = "world";  
      max_gain_viewpoint_pub_.publish(msg);  
    }
    
  }

  void FrontierFinder::pubGlobalPathMarkers()
  {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.id = 0;
    marker.ns = "path";
    marker.scale.x = 0.5;

    marker.color = getRandomColor(0, 1.0);

    visualization_msgs::Marker text;
    text.header.frame_id = "world";
    text.header.stamp = ros::Time::now();
    text.action = visualization_msgs::Marker::DELETEALL;
    text.ns = "length";
    marker_array.markers.push_back(text);

    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    
    text.scale.z = 2.0;  // 文字大小
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    Vector3d cur_pos, last_pos;
    last_pos = current_pos_;
    marker.points.push_back(toGeometryPoint(current_pos_)); 

    for(int i = 0; i < tsp_index_.size(); i++)
    {
      if(adj_nodes_[tsp_index_[i]].type_ == 2)
        continue;

      cur_pos = adj_nodes_[tsp_index_[i]].pos_;

      vector<Vector3d> path;
      if(getPath(last_pos, cur_pos, 20.0, path))
      {
        shortenPath(path);
        for(int j = 1; j < path.size(); j++)
        {
          marker.points.push_back(toGeometryPoint(path[j])); 
        }
      }
      

      text.id = i;
      if(path.size()==2)
      {
        Vector3d mid = (cur_pos + last_pos) / 2;
        text.pose.position.x = mid.x();
        text.pose.position.y = mid.y();
        text.pose.position.z = mid.z()+2.0; 
      }
      else if(path.size() > 2)
      {
        Vector3d mid = path[path.size()/2];
        text.pose.position.x = mid.x();
        text.pose.position.y = mid.y();
        text.pose.position.z = mid.z()+2.0; 
      }
      std::ostringstream out;
      out << std::fixed << std::setprecision(2) << getPathLength(path);
      text.text = out.str();
      marker_array.markers.push_back(text);

      last_pos = cur_pos;
    }

    marker_array.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.id = 0;
    marker.ns = "point";
    marker.scale.x = marker.scale.y = marker.scale.z = 0.7;

    marker.color = getRandomColor(4, 1.0);
    marker.points.clear();

    for(int i = 0; i < tsp_index_.size(); i++)
    {
      if(adj_nodes_[tsp_index_[i]].type_ == 2)
        continue;

      marker.points.push_back(toGeometryPoint(adj_nodes_[tsp_index_[i]].pos_));
    }

    marker_array.markers.push_back(marker);

    global_path_pub_.publish(marker_array);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    for(auto &point:failed_viewpoints_)
      cloud.points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);  // PCL 转 ROS 消息
    msg.header.frame_id = "world";  // 设置参考坐标系（RViz 中需要）

   
    msg.header.stamp = ros::Time::now();  // 更新时间戳
    find_path_failed_pub_.publish(msg);  // 发布点云消息
    failed_viewpoints_.clear();
  }

  void FrontierFinder::pubObstacleClustersMarkers()
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(obstacle_cloud_, msg);

    // 设置消息的帧 ID 和时间戳
    msg.header.frame_id = "world";  // RViz 中的参考坐标系
    msg.header.stamp = ros::Time::now();

    // 发布点云消息
    following_obstacle_cluster_pub_.publish(msg);
  }

  void FrontierFinder::pubSurfaceFrontierClustersMarkers()
  {
    // 使用surface_frontier_clusters_遍历 
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    // marker.ns = "delete_all";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);

    // ROS_WARN("Surface frontier clusters num:%d", surface_frontier_clusters_.size());
    for (auto it = surface_frontier_clusters_.begin(); it != surface_frontier_clusters_.end(); ++it)
    {
      // ROS_WARN("Obstacle_cluster id:%d  Surface_frontier_cluster num:%d", it->first, it->second.surface_frontier_ids_.size());
      // if(it->second.cells_.empty() || it->second.cells_.size() < surface_frontier_cluster_min_)
      //   continue;
      if(it->second.cells_.empty())
        continue;

      marker.ns = to_string(it->first);
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = marker.scale.y = marker.scale.z = sdf_map_->getResolution();
      marker.color = getRandomColor(it->first, 1.0);
      
      for(auto &cell:it->second.cells_)
      {
        Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);
        marker.points.push_back(toGeometryPoint(cell_pos));  
      }
      marker_array.markers.push_back(marker);  
    }
    surface_frontier_clusters_pub_.publish(marker_array);



    // 增量式发布
    // visualization_msgs::Marker text_marker;
    // text_marker.header.frame_id = "world";
    // text_marker.header.stamp = ros::Time::now();
    
    // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text_marker.action = visualization_msgs::Marker::ADD;
    // text_marker.ns = "label";

    // text_marker.scale.z = 1.0; // Text height
    // text_marker.color.r = 1.0;
    // text_marker.color.g = 1.0;
    // text_marker.color.b = 1.0;
    // text_marker.color.a = 1.0;

    // unordered_set<int> need_add_cluster_ids = new_surface_frontier_clusters_;
    // unordered_set<int> need_erase_cluster_ids = vp_.erased_frontier_cluster_ids_;
    // vp_.erased_frontier_cluster_ids_.clear();
  
    // marker_array.markers.clear();

    // marker.action = visualization_msgs::Marker::DELETE;
    // for(auto &id:need_erase_cluster_ids)
    // {
    //   marker.ns = "cell";
    //   marker.id = id;
    //   marker_array.markers.push_back(marker);

    //   marker.ns = "label";
    //   marker_array.markers.push_back(marker);

    //   marker.ns = "bound";
    //   marker_array.markers.push_back(marker);
    // }
    // following_surface_frontier_cluster_pub_.publish(marker_array);

    // marker_array.markers.clear();
    // marker.action = visualization_msgs::Marker::ADD;
    // for(auto &id:need_add_cluster_ids)
    // {
    //   auto &cluster = surface_frontier_clusters_.at(id);
    //   // if(cluster.cells_.size() < surface_frontier_cluster_min_)
    //   if(!cluster.can_be_observed_)
    //     continue;

    //   marker.ns = "cell";
    //   marker.id = id;
    //   marker.color = getRandomColor(id, 1.0);
    //   marker.points.clear();

    //   for(auto cell: cluster.cells_)
    //   {
    //     Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);
    //     marker.points.push_back(toGeometryPoint(cell_pos));
    //   }
    //   marker_array.markers.push_back(marker);  
      
    //   text_marker.pose.position = toGeometryPoint(cluster.average_pos_);
    //   text_marker.pose.position.z += 1.0; 

    //   text_marker.id = id;
    //   text_marker.text = to_string(id);
    //   marker_array.markers.push_back(text_marker);  
    // }
    // following_surface_frontier_cluster_pub_.publish(marker_array);
  
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.ns = "labels";

    text_marker.scale.z = 1.0; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    marker_array.markers.clear();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);

    marker.action= visualization_msgs::Marker::ADD;

    for(auto iter:surface_frontier_clusters_)
    {
      auto &cluster = iter.second;

      Vector3d average_pos(0, 0, 0);
      if(!cluster.can_be_observed_)
        continue;

      marker.color = getRandomColor(cluster.id_, 1.0);

      marker.points.clear();
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.scale.x = sdf_map_->getResolution();
      marker.id = cluster.id_;
      marker.ns = "cell";

      for(auto cell: cluster.cells_)
      {
        Vector3d cell_pos = sdf_map_->AddressToPos(cell.first);
        marker.points.push_back(toGeometryPoint(cell_pos));
        average_pos+=cell_pos;
      }
      marker_array.markers.push_back(marker);  

      average_pos/=cluster.cells_.size();
      text_marker.pose.position.x = average_pos.x();
      text_marker.pose.position.y = average_pos.y();
      text_marker.pose.position.z = average_pos.z() + 1.0; // Slightly above the point

      text_marker.id = cluster.id_;
      text_marker.text = to_string(cluster.id_);
      marker_array.markers.push_back(text_marker);
    }
    following_surface_frontier_cluster_pub_.publish(marker_array);
  }

  void FrontierFinder::pubSurfaceFrontierNormalsMarkers() 
  {
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.15; // Shaft diameter
    marker.scale.y = 0.2; // Head diameter
    marker.scale.z = 0.4;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    int id = 0;
    for (const auto cluster : surface_frontier_clusters_) 
    {
      if(cluster.second.cells_.size() < surface_frontier_cluster_min_)
        continue;
      for(auto cell : cluster.second.cells_)
      {
        if(cell.second[0] == 404)  // 近邻点太少，未计算法向量
          continue;

        Vector3d pos = sdf_map_->AddressToPos(cell.first);

        geometry_msgs::Point start = toGeometryPoint(pos);
        geometry_msgs::Point end = toGeometryPoint(pos + cell.second * 1.5);

        marker.points.clear();
        marker.id = id;
        marker.points.push_back(start);
        marker.points.push_back(end);
        marker_array.markers.push_back(marker);
        id++;
      }
    }
    surface_frontiers_normals_pub_.publish(marker_array);
  }

  void FrontierFinder::pubLocalSurfaceFrontierMarkers() 
  {    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;

    marker.scale.x = marker.scale.y = marker.scale.z = sdf_map_->getResolution();
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    for (const auto iter : local_surface_frontiers_) 
    {
      Vector3d pos = sdf_map_->AddressToPos(iter.first);
      marker.points.push_back(toGeometryPoint(pos));
    }
    local_surface_frontiers_pub_.publish(marker);
  }

  void FrontierFinder::pubLocalFrontiers() 
  {    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;

    marker.scale.x = marker.scale.y = marker.scale.z = sdf_map_->getResolution();
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    
    for (const auto iter : local_frontiers_) 
    {
      Vector3d pos = sdf_map_->AddressToPos(iter.first);
      marker.points.push_back(toGeometryPoint(pos));
    }
    local_frontiers_pub_.publish(marker);
  }


  void FrontierFinder::pubLocalViewpoints()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for(auto &viewpoint:selected_viewpoints_)
    {
      cloud.points.push_back(pcl::PointXYZ(viewpoint.x(), viewpoint.y(), viewpoint.z()));
    }
    // 将PCL点云转换为ROS消息
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";  // 设置合适的坐标系
    msg.header.stamp = ros::Time::now();
    // 发布点云
    local_viewpoints_pub_.publish(msg);

    // 发布viewpoint与frontier的关系
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; // 坐标系
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color = getRandomColor(5, 1.0);

    // 线条宽度
    marker.scale.x = 0.1;  

    ROS_WARN("selected_viewpoints_.size():%d", selected_viewpoints_.size());
    ROS_WARN("selected_viewpoints_attach_frontiers_.size():%d", selected_viewpoints_attach_frontiers_.size());

    for(int i = 0; i < int(selected_viewpoints_.size())-1; i++)
    {
      auto viewpoint_pos = toGeometryPoint(selected_viewpoints_.at(i));
      auto &frontiers = selected_viewpoints_attach_frontiers_.at(i);
      for(auto &frontier:frontiers)
      {
        marker.points.push_back(viewpoint_pos);
        marker.points.push_back(toGeometryPoint(sdf_map_->AddressToPos(frontier)));
      }
    }
    local_viewpoints_with_frontiers_pub_.publish(marker);

    marker.points.clear();
    marker.ns = "line";
    for(auto &iter:local_viewpoints_)
    {
      auto &viewpoint_iter = iter.second;
      for(auto &frontier:viewpoint_iter.second)
      {
        marker.points.push_back(toGeometryPoint(viewpoint_iter.first));
        marker.points.push_back(toGeometryPoint(sdf_map_->AddressToPos(frontier)));
      }
    }
    local_all_viewpoints_with_frontiers_pub_.publish(marker);

    marker.points.clear();
    marker.ns = "point";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.6;
    for(auto &iter:local_viewpoints_)
    {
      auto &viewpoint_iter = iter.second;
      marker.points.push_back(toGeometryPoint(viewpoint_iter.first));
    }
    local_all_viewpoints_with_frontiers_pub_.publish(marker);
    
  }

  void FrontierFinder::pubLocalPath()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.id = 0;
    marker.scale.x = 0.5;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.points.push_back(toGeometryPoint(current_pos_));  

    ROS_WARN("local_tsp_index_.size():%d  selected_viewpoints_.size():%d", local_tsp_index_.size(), selected_viewpoints_.size());
    for(int i = 0; i < int(local_tsp_index_.size()); i++)
    {
      ROS_WARN("i:%d  local_tsp_index_.at(i):%d", i, local_tsp_index_.at(i));

      Vector3d pos = selected_viewpoints_.at(local_tsp_index_.at(i));
      marker.points.push_back(toGeometryPoint(pos));   
    }
    local_path_pub_.publish(marker);
  }

  void FrontierFinder::pubChangeCellsMarkers()
  {
    std_msgs::ColorRGBA global_rgba;
    global_rgba.a = 0.8;
    global_rgba.r = 1;
    global_rgba.g = 1;
    global_rgba.b = 1;
    visualization_msgs::Marker change_cells_marker;
    generateMarkerArray("world", change_cells_marker, changed_cells_, global_rgba);
    change_cells_pub_.publish(change_cells_marker);
  }

  void FrontierFinder::pubEndVels()
  {
    if(end_vels_.empty())
      return;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.2; // Shaft diameter
    marker.scale.y = 0.3; // Head diameter
    marker.scale.z = 0.8;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for(int i = 0; i < end_vels_.size(); i++)
    {
      end_vels_[i].normalize();
      geometry_msgs::Point start, end;
      start = toGeometryPoint(end_points_[i]);
      end = toGeometryPoint(end_points_[i] + end_vels_[i] * 2.0);

      marker.points.clear();
      marker.id = i;
      marker.points.push_back(start);
      marker.points.push_back(end);
      marker_array.markers.push_back(marker);
    }
    end_vels_pub_.publish(marker_array);
  }

  void FrontierFinder::pubCurrentDirection()
  {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point start(toGeometryPoint(current_pos_));
      forward_directory_.normalize();
      geometry_msgs::Point end(toGeometryPoint(current_pos_ + forward_directory_ * 5.0));

      arrow.points.push_back(start);
      arrow.points.push_back(end);

      arrow.scale.x = 0.2; // Shaft diameter
      arrow.scale.y = 0.2; // Head diameter
      arrow.scale.z = 4.0; // Head length

      arrow.color.r = 0.0;
      arrow.color.g = 0.0;
      arrow.color.b = 1.0;
      arrow.color.a = 1.0;

      arrow.pose.orientation.x = arrow.pose.orientation.y = arrow.pose.orientation.z = 0.0;
      arrow.pose.orientation.w = 1.0;

      currrent_direction_pub_.publish(arrow);
  }

  void FrontierFinder::pubRoadMapPath()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
 
    for(auto &path:road_map_paths_)
    {
      for(int i = 0; i < static_cast<int>(path.size())-1; i++)
      {
        pcl::PointXYZ point_1(path[i].x(), path[i].y(), path[i].z());
        cloud.points.push_back(point_1);

        pcl::PointXYZ point_2(path[i+1].x(), path[i+1].y(), path[i+1].z());
        cloud.points.push_back(point_2);

        double dist = (path[i]-path[i+1]).norm();
        if(dist > 0.2)
        {
          int points_num = static_cast<int>(dist / 0.2);
          for (int j = 1; j <= points_num; ++j) 
          {
            double t = j * 0.2 / dist;
            Vector3d inter_point = path[i] + (path[i+1] -path[i]) * t;
            pcl::PointXYZ point(inter_point.x(), inter_point.y(), inter_point.z());
 
            cloud.points.push_back(point);
          }
        }
      }
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);  
    msg.header.frame_id = "world";  
    msg.header.stamp = ros::Time::now();  
    roadmap_paths_pub_.publish(msg);  
  }

  void FrontierFinder::pubViewpointClusters2()
  {
    // 增量式发布

    // static unordered_map<int, vector<int>> history_ids;

    // visualization_msgs::MarkerArray marker_array;
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "world";
    // marker.header.stamp = ros::Time::now();

    // marker.action = visualization_msgs::Marker::DELETE;
    // for(auto &id:vp_.erased_viewpoint_cluster_ids_)
    // {
    //   marker.ns = "cell";
    //   marker.id = id;
    //   marker_array.markers.push_back(marker);

    //   marker.ns = "vertex";
    //   marker_array.markers.push_back(marker);

    //   marker.ns = "line";
    //   marker_array.markers.push_back(marker);

    //   marker.ns = "label";
    //   for(auto label_id:history_ids.at(id))
    //   {
    //     marker.id = label_id;  
    //     marker_array.markers.push_back(marker);
    //   }
    // }
    // viewpoint_clusters_pub_.publish(marker_array);
    // id:vp_.erased_viewpoint_cluster_ids_.clear();

    // visualization_msgs::Marker text_marker;
    // text_marker.header.frame_id = "world";
    // text_marker.header.stamp = ros::Time::now();
    // text_marker.action = visualization_msgs::Marker::ADD;
    // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // text_marker.scale.z = 1.5; // Text height
    // text_marker.color.r = 1.0;
    // text_marker.color.g = 0.8;
    // text_marker.color.b = 0.8;
    // text_marker.color.a = 1.0;

    // marker.action = visualization_msgs::Marker::ADD;
    // marker.type = visualization_msgs::Marker::POINTS;
    // marker.pose.orientation.w = 1.0;

    // marker.scale.x = 0.5;
    // marker.scale.y = 0.5;
    // marker.scale.z = 0.5;
    
    // for(auto &id:vp_.changed_viewpoint_cluster_ids_)
    // {
    //   auto &cluster = viewpoint_clusters_.at(id);

    //   marker.ns = "cell";
    //   marker.id = id;
    //   marker.color = getRandomColor(id, 1.0);
    //   marker.points.clear();
      
    //   for(auto &cell_id:cluster.cells_)
    //     marker.points.push_back(toGeometryPoint(viewpoints_.at(cell_id).pos_));
        
    //   marker_array.markers.push_back(marker); 
    // }

    // for(auto &id:vp_.changed_viewpoint_cluster_ids_)
    // {
    //   auto &cluster = viewpoint_clusters_.at(id);

    //   text_marker.ns = "label";
    //   vector<int> history_id;
    //   for(auto &cell_id:cluster.cells_)
    //   { 
    //     text_marker.id = cell_id;
    //     history_id.push_back(cell_id);

    //     text_marker.pose.position = toGeometryPoint(viewpoints_.at(cell_id).pos_);
    //     text_marker.pose.position.z += 1.5;    
    //     text_marker.text = to_string(cluster.id_);
    //     marker_array.markers.push_back(text_marker); 
    //   }
    //   history_ids[id] = history_id;
    // }

    // marker.scale.x = 1.0;
    // marker.scale.y = 1.0;
    // marker.scale.z = 1.0;

    // for(auto &id:vp_.changed_viewpoint_cluster_ids_)
    // {
    //   auto &cluster = viewpoint_clusters_.at(id);

    //   marker.ns = "vertex";
    //   marker.id = id;
    //   marker.color = getRandomColor(id, 1.0);
    //   marker.points.clear();
      
    //   for(auto &pos:cluster.vertex_pos_)
    //     marker.points.push_back(toGeometryPoint(pos));
        
    //   marker_array.markers.push_back(marker); 
    // }

    // marker.type = visualization_msgs::Marker::LINE_LIST;
    // marker.scale.x = 0.5;
    // marker.scale.y = 0.0;
    // marker.scale.z = 0.0;
    // for(auto &id:vp_.changed_viewpoint_cluster_ids_)
    // {
    //   auto &cluster = viewpoint_clusters_.at(id);
    //   marker.ns = "line";
    //   marker.id = id;
    //   marker.color = getRandomColor(cluster.id_, 1.0);
    //   marker.points.clear();

    //   Vector3d pos_1 = cluster.avg_pos_ + cluster.dict_.normalized() * sqrt(cluster.eigenvalue_);
    //   Vector3d pos_2 = cluster.avg_pos_ - cluster.dict_.normalized() * sqrt(cluster.eigenvalue_);

    //   geometry_msgs::Point point_1 = toGeometryPoint(pos_1);
    //   geometry_msgs::Point point_2 = toGeometryPoint(pos_2);
    //   marker.points.push_back(point_1);
    //   marker.points.push_back(point_2);
    //   marker_array.markers.push_back(marker);
    // }
    // vp_.changed_viewpoint_cluster_ids_.clear();
    
    // viewpoint_clusters_pub_.publish(marker_array); 

    // 一次性发布
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.pose.orientation.w = 1.0;
    // marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;

    marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
    marker.ns = "viewpoints";

    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    text_marker.scale.z = 1.5; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.8;
    text_marker.color.b = 0.8;
    text_marker.color.a = 1.0;
    text_marker.ns = "cluster_id";

    int id = 0;
    // 聚类中的所有视点
    for(auto &iter:viewpoint_clusters_)
    {
      auto &cluster = iter.second;
      
      marker.color = getRandomColor(cluster.id_, 1.0);

      for(auto &cell_iter:cluster.cells_)
      {
        marker.id = id;
        text_marker.id = id + 2000;
        id++;
        marker.points.clear();
        marker.points.push_back(toGeometryPoint(viewpoints_.at(cell_iter).pos_));
        marker_array.markers.push_back(marker); 

        text_marker.pose.position = toGeometryPoint(viewpoints_.at(cell_iter).pos_);
        text_marker.pose.position.z += 1.5;    

        text_marker.text = to_string(cluster.id_);

        marker_array.markers.push_back(text_marker); 
        id++;
      }
    }

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.ns = "vertices";

    for(auto &iter:viewpoint_clusters_)
    {
      auto &cluster = iter.second;
      if(cluster.vertex_pos_.size() < 2)
        continue;
      marker.color = getRandomColor(cluster.id_, 1.0);
      marker.id = cluster.id_;
      marker.points.clear();
      for(auto &pos:cluster.vertex_pos_)
        marker.points.push_back(toGeometryPoint(pos));

      marker_array.markers.push_back(marker); 
    }

    // viewpoint_clusters_pub_.publish(marker_array);  

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.5;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.ns = "line";

    for(auto &iter:viewpoint_clusters_)
    {
      auto &cluster = iter.second;
      marker.points.clear();
      marker.id = cluster.id_;
      marker.color = getRandomColor(cluster.id_, 1.0);
      Vector3d pos_1 = cluster.avg_pos_ + cluster.dict_.normalized() * sqrt(cluster.eigenvalue_);
      Vector3d pos_2 = cluster.avg_pos_ - cluster.dict_.normalized() * sqrt(cluster.eigenvalue_);

      geometry_msgs::Point point_1 = toGeometryPoint(pos_1);
      geometry_msgs::Point point_2 = toGeometryPoint(pos_2);
      marker.points.push_back(point_1);
      marker.points.push_back(point_2);
      marker_array.markers.push_back(marker);
    }
    
    viewpoint_clusters_pub_.publish(marker_array); 
  }


  // 基于历史TSP的视点聚类
  void FrontierFinder::pubViewpointClusters()
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);   

    marker.action = visualization_msgs::Marker::ADD;

    marker.color.a = 1.0; 
    marker.color.r = 0.0; // Random color
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.pose.orientation.w = 1.0;
   
    marker.action = visualization_msgs::Marker::ADD;


    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();

    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "labels";

    text_marker.scale.z = 1.5; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.8;
    text_marker.color.b = 0.8;
    text_marker.color.a = 1.0;

    ROS_WARN("Viewpoint cluster size:%d", viewpoint_clusters_.size());
    ROS_WARN("Viewpoint size:%d", viewpoints_.size());

    for(auto &iter:viewpoint_clusters_)
    {
      auto &viewpoint_cluster = iter.second;
      if(viewpoint_cluster.is_merged_)
        continue;
      marker.points.clear();
      marker.ns = to_string(viewpoint_cluster.id_);
      if(viewpoint_cluster.ordered_cells_.size() < 2)
      {
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x =  marker.scale.y =  marker.scale.z = 0.5;
        geometry_msgs::Point point;
        point.x = viewpoints_[viewpoint_cluster.ordered_cells_[0]].pos_.x();
        point.y = viewpoints_[viewpoint_cluster.ordered_cells_[0]].pos_.y();
        point.z = viewpoints_[viewpoint_cluster.ordered_cells_[0]].pos_.z();
        marker.points.push_back(point);
      }
      else
      {
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.3;
        for(auto &viewpoint_id:viewpoint_cluster.ordered_cells_)
        {
          geometry_msgs::Point point;
          point.x = viewpoints_[viewpoint_id].pos_.x();
          point.y = viewpoints_[viewpoint_id].pos_.y();
          point.z = viewpoints_[viewpoint_id].pos_.z();

          marker.points.push_back(point);
        }
      }
      marker_array.markers.push_back(marker);   
    }

    for(auto &iter:viewpoint_clusters_)
    {
      auto &viewpoint_cluster = iter.second;
      if(viewpoint_cluster.is_merged_)
        continue;
      text_marker.ns = to_string(viewpoint_cluster.id_+1000);

      for(int i = 0; i < viewpoint_cluster.ordered_cells_.size(); i++)
      {
        text_marker.id = i;
        text_marker.pose.position.x = viewpoints_[viewpoint_cluster.ordered_cells_[i]].pos_.x();
        text_marker.pose.position.y = viewpoints_[viewpoint_cluster.ordered_cells_[i]].pos_.y();
        text_marker.pose.position.z = viewpoints_[viewpoint_cluster.ordered_cells_[i]].pos_.z() + 1.5;

        if(i==0)
        {
          int forward_id = -1, backward_id = -1;
          if(viewpoint_cluster.forward_target_!=-1 && viewpoints_.count(viewpoint_cluster.forward_target_))
            forward_id = viewpoints_[viewpoint_cluster.forward_target_].viewpoint_cluster_id_;
          if(viewpoint_cluster.backward_target_!=-1 && viewpoints_.count(viewpoint_cluster.backward_target_))
            backward_id = viewpoints_[viewpoint_cluster.backward_target_].viewpoint_cluster_id_;

          text_marker.text = to_string(forward_id) + ":" +
                            to_string(viewpoint_cluster.id_) + ":" +
                            to_string(backward_id);
          marker_array.markers.push_back(text_marker);  
        }
        else if(i==static_cast<int>(viewpoint_cluster.ordered_cells_.size())-1)
        {
          int forward_id = -1, backward_id = -1;
          if(viewpoint_cluster.forward_target_!=-1 && viewpoints_.count(viewpoint_cluster.forward_target_))
            forward_id = viewpoints_[viewpoint_cluster.forward_target_].viewpoint_cluster_id_;
          if(viewpoint_cluster.backward_target_!=-1 && viewpoints_.count(viewpoint_cluster.backward_target_))
            backward_id = viewpoints_[viewpoint_cluster.backward_target_].viewpoint_cluster_id_;

          text_marker.text = to_string(forward_id) + ":" +
                            to_string(viewpoint_cluster.id_) + ":" +
                            to_string(backward_id);
          marker_array.markers.push_back(text_marker);  
        }
        else
        {
          text_marker.text = to_string(viewpoint_cluster.id_);
          marker_array.markers.push_back(text_marker);  
        }
      }
      // for(auto &cell_iter:viewpoint_cluster.ordered_cells_)
      // {
      //   int forward_id = -1, backward_id = -1;
      //   if(viewpoint_cluster.forward_target_!=-1 && viewpoints_.count(viewpoint_cluster.forward_target_))
      //     forward_id = viewpoints_[viewpoint_cluster.forward_target_].viewpoint_cluster_id_;
      //   if(viewpoint_cluster.backward_target_!=-1 && viewpoints_.count(viewpoint_cluster.backward_target_))
      //     backward_id = viewpoints_[viewpoint_cluster.backward_target_].viewpoint_cluster_id_;

      //   text_marker.text = to_string(forward_id) + ":" +
      //                     to_string(viewpoint_cluster.id_) + ":" +
      //                     to_string(backward_id) + ":" +
      //                     to_string(viewpoint_cluster.is_merged_);
      //   // text_marker.id = viewpoint_cluster.id_;
      //   text_marker.ns = to_string(viewpoint_cluster.id_);
        
      // }
      // marker_array.markers.push_back(text_marker);  

    }
    viewpoint_clusters_pub_.publish(marker_array);    
  }

  void FrontierFinder::pubStatistics()
  {
    knownCellOutput();

    active_perception::frontierStatistics frontier_statistics_;
    frontier_statistics_.resolution = sdf_map_->getResolution();
    frontier_statistics_.known_cell_num = known_cell_num_;
    frontier_statistics_.known_plane_cell_num = known_plane_cell_num_;

    SDFMap_frontier_statistics_pub_.publish(frontier_statistics_);
  }

  void FrontierFinder::knownCellOutput()
  {
    known_cell_num_ = getKnownNodeNum(simple_known_cells_);
    known_plane_cell_num_ = getKnownPlaneNodeNum(simple_known_cells_);
  }

  std::size_t FrontierFinder::getKnownNodeNum(const std::unordered_set<int> &known_cells)
  {
    return known_cells.size();
  }

  std::size_t FrontierFinder::getKnownPlaneNodeNum(const std::unordered_set<int> &known_cells)
  {
    std::size_t size = 0;
    for (const auto &iter : known_cells)
    {
      Eigen::Vector3d pos_temp(sdf_map_->AddressToPos(iter));
      if (
          pos_temp.z() >= current_pos_.z() &&
          pos_temp.z() < current_pos_.z() + sdf_map_->getResolution())
      {
        size++;
      }
    }
    return size;
  }

  bool FrontierFinder::isFrontier(const Eigen::Vector3i &index)
  {
    if (knownfree(index) && isNeighborUnknown(index)) 
    // if (sdf_map_->getOccupancy(index) == SDFMap::UNKNOWN && isNeighborFree(index))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool FrontierFinder::isSurfaceFrontier(const Eigen::Vector3i &index)
  {
    int addr = sdf_map_->toAddress(index);
    return isFrontier(index) && isNeighborObstacle(index) && (covered_surface_frontier_.find(addr) == covered_surface_frontier_.end());
  }

  bool FrontierFinder::isSurfaceFrontier(const int &addr)
  {
    Vector3i index = sdf_map_->AddressToIndex(addr);
    return isFrontier(index) && isNeighborObstacle(index) && (covered_surface_frontier_.find(addr) == covered_surface_frontier_.end());
  }

  bool FrontierFinder::isPlaneFrontier(const Eigen::Vector3i &index)
  {
    if (knownfree(index) && isPlaneNeighborUnknown(index))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool FrontierFinder::isPlaneFrontier(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i index;
    sdf_map_->posToIndex(pos, index);
    if (knownfree(index) && isPlaneNeighborUnknown(index))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  std::vector<Eigen::Vector3d> FrontierFinder::getSimpleFrontiers()
  {
    std::vector<Eigen::Vector3d> frontiers;
    for (const auto &iter_cell : global_frontier_cells_)
    {
      frontiers.push_back(sdf_map_->AddressToPos(iter_cell));
    }
    return frontiers;
  }

  std::vector<Eigen::Vector3d> FrontierFinder::getSimpleLocalFrontiers(const double &range)
  {
    auto distXY = [](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      Eigen::Vector3d diff = p2 - p1;
      diff.z() = 0.0;
      return diff.norm();
    };

    std::vector<Eigen::Vector3d> frontiers;

    for (const auto &iter_cell : global_frontier_cells_)
    {
      Eigen::Vector3d frontier(sdf_map_->AddressToPos(iter_cell));
      if (distXY(frontier, current_pos_) < range)
        frontiers.push_back(frontier);
    }
    return frontiers;
  }

  std::vector<Eigen::Vector3d> FrontierFinder::getSimpleClearedFrontiers()
  {
    cleared_ft_mutex.lock();
    std::vector<Eigen::Vector3d> frontiers;
    for (const auto &iter_cell : cleared_cells_)
    {
      frontiers.push_back(sdf_map_->AddressToPos(iter_cell));
    }
    cleared_ft_mutex.unlock();
    return frontiers;
  }

  void FrontierFinder::resetSimpleClearedFrontiers()
  {
    cleared_cells_.clear();
  }

  double FrontierFinder::getMaxRayLength()
  {
    return sdf_map_->getMaxRayLength();
  }

  double FrontierFinder::getFrontierUpperBound()
  {
    return frontier_local_ub_;
  }

  double FrontierFinder::getFrontierLowerBound()
  {
    return frontier_local_lb_;
  }

  double FrontierFinder::getSensorHeight()
  {
    return sensor_height_;
  }

  bool FrontierFinder::isCollisionFree(const Eigen::Vector3d &source, const Eigen::Vector3d &target)
  {
    return sdf_map_->isCollisionFreeStraight(source, target);
  }

  void FrontierFinder::searchFrontiers()
  {
    ros::Time t1 = ros::Time::now();
    tmp_frontiers_.clear();

    // Bounding box of updated region
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max, true);

    // Removed changed frontiers in updated map
    auto resetFlag = [&](list<Frontier>::iterator &iter, list<Frontier> &frontiers)
    {
      Eigen::Vector3i idx;
      for (auto cell : iter->cells_)
      {
        sdf_map_->posToIndex(cell, idx);
        frontier_flag_[toadr(idx)] = 0;
      }
      iter = frontiers.erase(iter);
    };

    // std::cout << "Before remove: " << frontiers_.size() << std::endl;

    removed_ids_.clear();
    int rmv_idx = 0;
    for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
    {
      if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
          isFrontierChanged(*iter))
      {
        resetFlag(iter, frontiers_);
        removed_ids_.push_back(rmv_idx);
      }
      else
      {
        ++rmv_idx;
        ++iter;
      }
    }
    // std::cout << "After remove: " << frontiers_.size() << std::endl;
    for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();)
    {
      if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
          isFrontierChanged(*iter))
        resetFlag(iter, dormant_frontiers_);
      else
        ++iter;
    }

    // Search new frontier within box slightly inflated from updated box
    Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
    Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
    Vector3d box_min, box_max;
    sdf_map_->getBox(box_min, box_max);
    for (int k = 0; k < 3; ++k)
    {
      search_min[k] = max(search_min[k], box_min[k]);
      search_max[k] = min(search_max[k], box_max[k]);
    }
    Eigen::Vector3i min_id, max_id;
    sdf_map_->posToIndex(search_min, min_id);
    sdf_map_->posToIndex(search_max, max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y)
        for (int z = min_id(2); z <= max_id(2); ++z)
        {
          // Scanning the updated region to find seeds of frontiers
          Eigen::Vector3i cur(x, y, z);
          if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur))
          {
            // Expand from the seed cell to find a complete frontier cluster
            expandFrontier(cur);
          }
        }
    splitLargeFrontiers(tmp_frontiers_);

    ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
  }

  void FrontierFinder::expandFrontier(
      const Eigen::Vector3i &first /* , const int& depth, const int& parent_id */)
  {
    // std::cout << "depth: " << depth << std::endl;
    auto t1 = ros::Time::now();

    // Data for clustering
    queue<Eigen::Vector3i> cell_queue;
    vector<Eigen::Vector3d> expanded;
    Vector3d pos;

    sdf_map_->indexToPos(first, pos);
    expanded.push_back(pos);
    cell_queue.push(first);
    frontier_flag_[toadr(first)] = 1;

    // Search frontier cluster based on region growing (distance clustering)
    while (!cell_queue.empty())
    {
      auto cur = cell_queue.front();
      cell_queue.pop();
      auto nbrs = allNeighbors(cur);
      for (auto nbr : nbrs)
      {
        // Qualified cell should be inside bounding box and frontier cell not clustered
        int adr = toadr(nbr);
        if (frontier_flag_[adr] == 1 || !sdf_map_->isInBox(nbr) ||
            !(knownfree(nbr) && isNeighborUnknown(nbr)))
          continue;

        sdf_map_->indexToPos(nbr, pos);
        if (pos[2] < 0.4)
          continue; // Remove noise close to ground
        expanded.push_back(pos);
        cell_queue.push(nbr);
        frontier_flag_[adr] = 1;
      }
    }
    if (expanded.size() > cluster_min_)
    {
      // Compute detailed info
      Frontier frontier;
      frontier.cells_ = expanded;
      computeFrontierInfo(frontier);
      tmp_frontiers_.push_back(frontier);
    }
  }

  // 没有大小限制
  void FrontierFinder::expandSurfaceFrontier2(const int &seed)
  {
    auto t1 = ros::Time::now();

    // Data for clustering
    queue<int> cell_queue;              // region-grow队列
    vector<int> expanded;               // 扩展的cell
    unordered_set<int> neighbor_cluster_ids;     // 邻接的聚类id

    vector<int> outrange_cells;

    Vector3d avg_normal(404, 404, 404);  // 聚类的平均法向量
    Vector3d toal_normal(0, 0, 0);
    int normal_num = 0;

    bool is_detect_cluster = false;     // 是否有邻接聚类
    bool is_large = false;

    expanded.push_back(seed);
    cell_queue.push(seed);
    surface_frontier_flag_.insert(seed);
    if(new_surface_frontiers_.at(seed).x()!=404)
    {
      toal_normal+=new_surface_frontiers_.at(seed);
      normal_num++;
      avg_normal = toal_normal / normal_num;
    }

    Vector3d seed_pos = sdf_map_->AddressToPos(seed);
    // Search frontier cluster based on region growing (distance clustering)
    while (!cell_queue.empty())
    {
      auto cur = cell_queue.front();
      cell_queue.pop();
      auto nbrs = allNeighbors(cur);
      for (auto nbr : nbrs)
      {
        int adr = toadr(nbr); 
        if (!sdf_map_->isInBox(nbr))  // Qualified cell should be inside bounding box
          continue;
        // 未扩展过且为surface_frontier
        // if(!isExpandedFrontier(adr) && new_surface_frontiers_.find(adr)!=new_surface_frontiers_.end())
        if(!isExpandedFrontier(adr) && isSurfaceFrontier(adr) && sdf_map_->isInFrontierBox(nbr))
        {  
          // 更新 Bounding Box
          Vector3d nbr_pos = sdf_map_->AddressToPos(adr);

          Vector3d nbr_normal(404, 404, 404);

          if(use_normal_vector_)
          {
            if(new_surface_frontiers_.count(adr))
              nbr_normal = new_surface_frontiers_.at(adr);

            if(nbr_normal.x()!=404 && avg_normal.x()!=404)
            {
              double angle_rad = acos(nbr_normal.dot(avg_normal) / (nbr_normal.norm() * avg_normal.norm()));
              double diff_angle = angle_rad * 180.0 / M_PI;

              if(diff_angle < angle_thres_)
              {
                toal_normal+=nbr_normal;
                normal_num++;
                avg_normal=toal_normal/normal_num;
                
                cell_queue.push(adr);
                expanded.push_back(adr);
                surface_frontier_flag_.insert(adr);
              }
            }
            else if(nbr_normal.x()!=404)
            {
              toal_normal+=nbr_normal;
              normal_num++;
              avg_normal=toal_normal/normal_num;

              cell_queue.push(adr);
              expanded.push_back(adr);
              surface_frontier_flag_.insert(adr);
            }
            else
            {
              cell_queue.push(adr);
              expanded.push_back(adr);
              surface_frontier_flag_.insert(adr);              
            }
          }
          else
          {
            cell_queue.push(adr);
            expanded.push_back(adr);
            surface_frontier_flag_.insert(adr);     
          }
        }
      }
    }
    if (expanded.size() > 0)
    {
      createSurfaceFrontierCluster(expanded, neighbor_cluster_ids);
    }
  }


  // void FrontierFinder::expandSurfaceFrontier(Vector3d &pos, vector<Eigen::Vector3d> &expanded)
  // {
  //   // std::cout << "depth: " << depth << std::endl;
  //   auto t1 = ros::Time::now();

  //   // Data for clustering
  //   queue<int> cell_queue;
  //   // vector<Eigen::Vector3d> expanded;
  //   Vector3i index;
  //   sdf_map_->posToIndex(pos, index);
  //   int addr = sdf_map_->toAddress(index);

  //   expanded.push_back(pos);
  //   cell_queue.push(addr);
  //   local_frontier_cells_[addr] = 1;

  //   while (!cell_queue.empty())
  //   {
  //     auto cur = cell_queue.front();
  //     cell_queue.pop();

  //     Vector3i index = sdf_map_->AddressToIndex(cur);
      
  //     auto nbrs = allNeighbors(index);

  //     for (auto nbr : nbrs)
  //     {
  //       // Qualified cell should be inside bounding box and frontier cell not clustered
  //       int adr = toadr(nbr);
  //       if (!sdf_map_->isInBox(nbr))
  //         continue;

  //       if (surface_frontier_flag_.find(adr) != surface_frontier_flag_.end() 
  //        && new_surface_frontier_cells_.find(adr) != new_surface_frontier_cells_.end()) // 未索引过
  //       {  
  //         expanded.push_back(adr);
  //         cell_queue.push(adr);
  //         obstacle_flag_[adr] = 1;
  //       }
  //       else
  //       { 
  //         if(obstacle_cell_array_.find(adr)==obstacle_cell_array_.end())  // 索引过，但未分类
  //         {
  //           // expanded.push_back(adr);
  //           // cell_queue.push(adr);
  //           // obstacle_flag_[adr] = 1;
  //         }
  //         else  // 索引过，且分过类
  //         {
  //           is_detect_cluster = true;
  //           cluster_ids.insert(obstacle_cell_array_[adr]);
  //         }
          
  //       }
  //     }
  //   }
  // }

  bool FrontierFinder::isDetectObstacle()
  {
    return main_obstacle_id_ > -1;
  }

  int FrontierFinder::generateSurfaceFrontierClusterId()
  {
    int cluster_id;
    if(!surface_frontier_cluster_ids_.empty())
    {
      // 使用回收的聚类编号
      cluster_id = surface_frontier_cluster_ids_.front();
      surface_frontier_cluster_ids_.pop();
    }
    else
    {
      // 使用新的聚类编号
      cluster_id = surface_frontier_cluster_id_;
      surface_frontier_cluster_id_++;
    }
    return cluster_id;
  }

  int FrontierFinder::generateViewpointClusterId()
  {
    int id;
    if(!viewpoint_cluster_ids_.empty())
    {
      // 使用回收的聚类编号
      id = viewpoint_cluster_ids_.front();
      viewpoint_cluster_ids_.pop();
    }
    else
    {
      // 使用新的聚类编号
      id = viewpoint_cluster_id_;
      viewpoint_cluster_id_++;
    }
    return id;
  }

  // void FrontierFinder::expandSurfaceFrontier(const int &seed)
  // {
  //   auto t1 = ros::Time::now();

  //   // Data for clustering
  //   queue<int> cell_queue;              // region-grow队列
  //   vector<int> expanded;               // 扩展的cell
  //   unordered_set<int> neighbor_cluster_ids;     // 邻接的聚类id
  //   vector<int> neighbor_obstacle_ids;  // 邻接的障碍物聚类id
  //   vector<int> outrange_cells;

  //   Vector3d box_min, box_max;  // Bounding Box
  //   double diag_distance; // Bounding Box 的对角线距离

  //   Vector3d avg_normal(404, 404, 404);  // 聚类的平均法向量
  //   Vector3d toal_normal(0, 0, 0);
  //   int normal_num = 0;

  //   bool is_detect_cluster = false;     // 是否有邻接聚类
  //   bool is_large = false;

  //   expanded.push_back(seed);
  //   cell_queue.push(seed);
  //   surface_frontier_flag_.insert(seed);
  //   if(new_surface_frontiers_.at(seed).x()!=404)
  //   {
  //     toal_normal+=new_surface_frontiers_.at(seed);
  //     normal_num++;
  //     avg_normal = toal_normal / normal_num;
  //   }

  //   Vector3d seed_pos = sdf_map_->AddressToPos(seed);
  //   box_min = seed_pos;
  //   box_max = seed_pos;
  //   diag_distance = (box_max-box_min).norm();

  //   // Search frontier cluster based on region growing (distance clustering)
  //   while (!cell_queue.empty() && !is_large)
  //   {
  //     auto cur = cell_queue.front();
  //     cell_queue.pop();
  //     auto nbrs = allNeighbors(cur);
  //     for (auto nbr : nbrs)
  //     {
  //       int adr = toadr(nbr); 
  //       if (!sdf_map_->isInBox(nbr))  // Qualified cell should be inside bounding box
  //         continue;
  //       if(obstacle_cell_array_.find(adr)!=obstacle_cell_array_.end())  // 邻居为障碍物
  //       {
  //         neighbor_obstacle_ids.push_back(obstacle_cell_array_.at(adr));
  //       }
  //       else  // 邻居为surface_frontier
  //       {
  //         // 未扩展过且为surface_frontier
  //         if(!isExpandedFrontier(adr) && new_surface_frontiers_.find(adr)!=new_surface_frontiers_.end())
  //         {  
  //           // 更新 Bounding Box
  //           Vector3d nbr_pos = sdf_map_->AddressToPos(adr);
  //           for(int i=0;i<3;i++)
  //           {
  //             box_min[i] = min(nbr_pos[i], box_min[i]);
  //             box_max[i] = max(nbr_pos[i], box_max[i]);
  //           }
  //           Vector3d nbr_normal = new_surface_frontiers_.at(adr);
  //           double diff_angle = 0;
  //           if(nbr_normal.x()!=404)
  //           {
  //             double angle_rad = acos(nbr_normal.dot(avg_normal) / (nbr_normal.norm() * avg_normal.norm()));
  //             diff_angle = angle_rad * 180.0 / M_PI;
  //           }
  //           diag_distance = (box_max-box_min).norm();
          
  //           if(diag_distance < 20.0 && diff_angle < 90.0)
  //           {
  //             cell_queue.push(adr);
  //             if(nbr_normal.x()!=404)
  //             {
  //               toal_normal+=nbr_normal;
  //               normal_num++;
  //               avg_normal=toal_normal/normal_num;
  //             }
  //             expanded.push_back(adr);
  //           }
  //           else
  //           {
  //             outrange_cells.push_back(adr);
  //             if(diag_distance > 20.0)
  //               is_large = true;
  //           }
  //           surface_frontier_flag_.insert(adr);
  //         }
  //         else  // 访问过且分过类的surface_frontier
  //         { 
  //           if(isClusteredFrontier(adr))  
  //           {
  //             is_detect_cluster = true;
  //             neighbor_cluster_ids.insert(surface_frontier_array_.at(adr));
  //           } 
  //         }
  //       }
  //     }
  //   }
  //   if (expanded.size() > surface_frontier_cluster_min_ || neighbor_cluster_ids.empty())
  //   {
  //     createSurfaceFrontierCluster(expanded, neighbor_cluster_ids);
  //   }
  //   else if(expanded.size() > 0)  // 合并frontier
  //   {
  //     int smallest_cluster_id = -1;
  //     int smallest_num = INT16_MAX;
  //     for(auto &cluster_id:neighbor_cluster_ids)
  //     {
  //       auto &nbr_cluster =  surface_frontier_clusters_.at(cluster_id);
  //       Vector3d nbr_normal = nbr_cluster.average_normal_;

  //       if(nbr_normal.x()!=404 && avg_normal.x()!=404)
  //       {
  //         double angle_rad = acos(nbr_normal.dot(avg_normal) / (nbr_normal.norm() * avg_normal.norm()));
  //         double angle_angle = angle_rad * 180.0 / M_PI;
          
  //         Vector3d new_box_min, new_box_max;
  //         for(int i = 0; i<3; i++)
  //         {
  //           new_box_min[i] = min(nbr_cluster.box_min_[i], box_min[i]);
  //           new_box_max[i] = max(nbr_cluster.box_max_[i], box_max[i]);
  //         }
          
  //         if(angle_angle < 45.0 && (new_box_max-new_box_min).norm() < 12.0)
  //         {
  //           if(nbr_cluster.cells_.size() < smallest_num)
  //           {
  //             smallest_num = nbr_cluster.cells_.size();
  //             smallest_cluster_id = cluster_id;
  //           }
  //         }
  //       } 
  //     }
  //     // 减少约束
  //     if(smallest_cluster_id==-1)
  //     {
  //       for(auto &cluster_id:neighbor_cluster_ids)
  //       {
  //         auto &nbr_cluster =  surface_frontier_clusters_.at(cluster_id);
  //         Vector3d new_box_min, new_box_max;
  //         for(int i = 0; i<3; i++)
  //         {
  //           new_box_min[i] = min(nbr_cluster.box_min_[i], box_min[i]);
  //           new_box_max[i] = max(nbr_cluster.box_max_[i], box_max[i]);
  //         }
          
  //         if((new_box_max-new_box_min).norm() < 12.0)
  //         {
  //           if(nbr_cluster.cells_.size() < smallest_num)
  //           {
  //             smallest_num = nbr_cluster.cells_.size();
  //             smallest_cluster_id = cluster_id;
  //           }
  //         }
  //       }
  //     }
  //     if(smallest_cluster_id!=-1)
  //     {
  //       // 将扩展的cell合并到最小的邻接聚类上
  //       for(auto &cell:expanded)
  //       {
  //         surface_frontier_clusters_.at(smallest_cluster_id).cells_[cell] = new_surface_frontiers_.at(cell);
  //         surface_frontier_array_[cell] = smallest_cluster_id;
  //       }
  //       neighbor_cluster_ids.erase(smallest_cluster_id);
  //       // surface_frontier_clusters_[smallest_cluster_id].neighbor_cluster_ids_.insert(neighbor_cluster_ids.begin(), neighbor_cluster_ids.end());
  //       new_surface_frontier_clusters_.insert(smallest_cluster_id);

  //       computeSurfaceFrontierClusterInfo(surface_frontier_clusters_.at(smallest_cluster_id));  
  //     }
  //     else
  //     {
  //       createSurfaceFrontierCluster(expanded, neighbor_cluster_ids);
  //     }
  //   }
  //   for(auto &cell:outrange_cells)
  //   {
  //     surface_frontier_flag_.erase(cell);
  //   }
  //   // TODO 是否要用递归？会不会递归太深？
  //   for(auto &seed:outrange_cells)
  //   {
  //     if(!isClusteredFrontier(seed))
  //       expandSurfaceFrontier(seed);
  //   }
  //   // if(!outrange_cells.empty())
  //   // {
  //   //   int seed = outrange_cells.front();
  //   //   expandSurfaceFrontier(seed);
  //   // }
  // }

  bool FrontierFinder::isClusteredFrontier(int addr)
  {
    return surface_frontier_array_.find(addr)!=surface_frontier_array_.end();
  }

  bool FrontierFinder::isExpandedFrontier(int addr)
  {
    return surface_frontier_flag_.find(addr)!=surface_frontier_flag_.end();
  }

  void FrontierFinder::createSurfaceFrontierCluster(const vector<int> &expanded, const unordered_set<int> &neighbor_cluster_ids)
  {
    // 获取新聚类的id
    int cluster_id = generateSurfaceFrontierClusterId();
    SurfaceFrontierCluster cluster;
    
    for(auto cell_id:expanded)
    {
      surface_frontier_array_[cell_id] = cluster_id;
      cluster.cells_[cell_id] = new_surface_frontiers_[cell_id];
      cluster.id_ = cluster_id;
    }
    // cluster.neighbor_cluster_ids_ = neighbor_cluster_ids;
    computeSurfaceFrontierClusterInfo(cluster);  
    
    surface_frontier_clusters_[cluster_id] = cluster;

    new_surface_frontier_clusters_.insert(cluster_id);
  }

  void FrontierFinder::computeSurfaceFrontierClusterInfo(SurfaceFrontierCluster &cluster)
  {
    
    Vector3d total_normal(0, 0 ,0);
    int normal_num = 0;

    Vector3d pos = sdf_map_->AddressToPos(cluster.cells_.begin()->first);
    cluster.box_max_ = pos;
    cluster.box_min_ = pos;
    cluster.average_pos_.setZero();
    
    for (auto cell : cluster.cells_)
    {
      pos = sdf_map_->AddressToPos(cell.first);
      for (int i = 0; i < 3; ++i)
      {       
        cluster.box_min_[i] = min(cluster.box_min_[i], pos[i]);
        cluster.box_max_[i] = max(cluster.box_max_[i], pos[i]);
      }

      if(cell.second.x()!=404)
      {
        total_normal+=cell.second;
        normal_num++;
      }
      
      cluster.average_pos_ += pos / cluster.cells_.size();
    }
    
    if(normal_num>0)
        cluster.average_normal_ = total_normal/normal_num;
      else
        cluster.average_normal_ = Vector3d(404, 404, 404);
  }

  void FrontierFinder::eraseSurfaceFrontierCluster(int id)
  {
    // auto neighbor_cluster_ids = surface_frontier_clusters_[id].neighbor_cluster_ids_;
    // // 删除与其他frontier聚类的连接
    // for(auto &neighbor_id:neighbor_cluster_ids)
    // {
    //   surface_frontier_clusters_[neighbor_id].neighbor_cluster_ids_.erase(id);
    // }
    // 回收聚类编号
    surface_frontier_cluster_ids_.push(id);
    // 删除该聚类
    surface_frontier_clusters_.erase(id);
    // 在rviz中删除
    vp_.erased_frontier_cluster_ids_.insert(id);
  }

  void FrontierFinder::splitLargeFrontiers(list<Frontier> &frontiers)
  {
    list<Frontier> splits, tmps;
    for (auto it = frontiers.begin(); it != frontiers.end(); ++it)
    {
      // Check if each frontier needs to be split horizontally
      if (splitHorizontally(*it, splits))
      {
        tmps.insert(tmps.end(), splits.begin(), splits.end());
        splits.clear();
      }
      else
        tmps.push_back(*it);
    }
    frontiers = tmps;
  }

  bool FrontierFinder::splitHorizontally(const Frontier &frontier, list<Frontier> &splits)
  {
    // Split a frontier into small piece if it is too large
    auto mean = frontier.average_.head<2>();
    bool need_split = false;
    for (auto cell : frontier.filtered_cells_)
    {
      if ((cell.head<2>() - mean).norm() > cluster_size_xyz_)
      {
        need_split = true;
        break;
      }
    }
    if (!need_split)
      return false;

    // Compute principal component
    // Covariance matrix of cells
    Eigen::Matrix2d cov;
    cov.setZero();
    for (auto cell : frontier.filtered_cells_)
    {
      Eigen::Vector2d diff = cell.head<2>() - mean;
      cov += diff * diff.transpose();
    }
    cov /= double(frontier.filtered_cells_.size());

    // Find eigenvector corresponds to maximal eigenvector
    Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int max_idx;
    double max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i)
    {
      if (values[i] > max_eigenvalue)
      {
        max_idx = i;
        max_eigenvalue = values[i];
      }
    }
    Eigen::Vector2d first_pc = vectors.col(max_idx);
    // std::cout << "max idx: " << max_idx << std::endl;
    // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

    // Split the frontier into two groups along the first PC
    Frontier ftr1, ftr2;
    for (auto cell : frontier.cells_)
    {
      if ((cell.head<2>() - mean).dot(first_pc) >= 0)
        ftr1.cells_.push_back(cell);
      else
        ftr2.cells_.push_back(cell);
    }
    computeFrontierInfo(ftr1);
    computeFrontierInfo(ftr2);

    // Recursive call to split frontier that is still too large
    list<Frontier> splits2;
    if (splitHorizontally(ftr1, splits2))
    {
      splits.insert(splits.end(), splits2.begin(), splits2.end());
      splits2.clear();
    }
    else
      splits.push_back(ftr1);

    if (splitHorizontally(ftr2, splits2))
      splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
      splits.push_back(ftr2);

    return true;
  }

  bool FrontierFinder::isInBoxes(
      const vector<pair<Vector3d, Vector3d>> &boxes, const Eigen::Vector3i &idx)
  {
    Vector3d pt;
    sdf_map_->indexToPos(idx, pt);
    for (auto box : boxes)
    {
      // Check if contained by a box
      bool inbox = true;
      for (int i = 0; i < 3; ++i)
      {
        inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
        if (!inbox)
          break;
      }
      if (inbox)
        return true;
    }
    return false;
  }

  void FrontierFinder::updateFrontierCostMatrix()
  {
    // std::cout << "cost mat size before remove: " << std::endl;
    for (auto ftr : frontiers_)
      // std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
      // std::cout << "" << std::endl;

      // std::cout << "cost mat size remove: " << std::endl;
      if (!removed_ids_.empty())
      {
        // Delete path and cost for removed clusters
        for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it)
        {
          auto cost_iter = it->costs_.begin();
          auto path_iter = it->paths_.begin();
          int iter_idx = 0;
          for (int i = 0; i < removed_ids_.size(); ++i)
          {
            // Step iterator to the item to be removed
            while (iter_idx < removed_ids_[i])
            {
              ++cost_iter;
              ++path_iter;
              ++iter_idx;
            }
            cost_iter = it->costs_.erase(cost_iter);
            path_iter = it->paths_.erase(path_iter);
          }
          // std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
        }
        removed_ids_.clear();
      }
    // std::cout << "" << std::endl;

    auto updateCost = [](const list<Frontier>::iterator &it1, const list<Frontier>::iterator &it2)
    {
      // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
      // Search path from old cluster's top viewpoint to new cluster'
      Viewpoint &vui = it1->viewpoints_.front();
      Viewpoint &vuj = it2->viewpoints_.front();
      vector<Vector3d> path_ij;
      double cost_ij = ViewNode::computeCost(
          vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
      // Insert item for both old and new clusters
      it1->costs_.push_back(cost_ij);
      it1->paths_.push_back(path_ij);
      reverse(path_ij.begin(), path_ij.end());
      it2->costs_.push_back(cost_ij);
      it2->paths_.push_back(path_ij);
    };

    // std::cout << "cost mat add: " << std::endl;
    // Compute path and cost between old and new clusters
    for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
      for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
        updateCost(it1, it2);

    // Compute path and cost between new clusters
    for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
      for (auto it2 = it1; it2 != frontiers_.end(); ++it2)
      {
        if (it1 == it2)
        {
          // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
          it1->costs_.push_back(0);
          it1->paths_.push_back({});
        }
        else
          updateCost(it1, it2);
      }
    // std::cout << "" << std::endl;
    // std::cout << "cost mat size final: " << std::endl;
    // for (auto ftr : frontiers_)
    //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
    // std::cout << "" << std::endl;
  }

  void FrontierFinder::mergeFrontiers(Frontier &ftr1, const Frontier &ftr2)
  {
    // Merge ftr2 into ftr1
    ftr1.average_ =
        (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
        (double(ftr1.cells_.size() + ftr2.cells_.size()));
    ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
    computeFrontierInfo(ftr1);
  }

  bool FrontierFinder::canBeMerged(const Frontier &ftr1, const Frontier &ftr2)
  {
    Vector3d merged_avg =
        (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
        (double(ftr1.cells_.size() + ftr2.cells_.size()));
    // Check if it can merge two frontier without exceeding size limit
    for (auto c1 : ftr1.cells_)
    {
      auto diff = c1 - merged_avg;
      if (diff.head<2>().norm() > cluster_size_xyz_ || diff[2] > cluster_size_z_)
        return false;
    }
    for (auto c2 : ftr2.cells_)
    {
      auto diff = c2 - merged_avg;
      if (diff.head<2>().norm() > cluster_size_xyz_ || diff[2] > cluster_size_z_)
        return false;
    }
    return true;
  }

  bool FrontierFinder::haveOverlap(
      const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2)
  {
    // Check if two box have overlap part
    Vector3d bmin, bmax;
    for (int i = 0; i < 3; ++i)
    {
      bmin[i] = max(min1[i], min2[i]);
      bmax[i] = min(max1[i], max2[i]);
      if (bmin[i] > bmax[i] + 1e-3)
        return false;
    }
    return true;
  }

  bool FrontierFinder::isFrontierChanged(const Frontier &ft)
  {
    for (auto cell : ft.cells_)
    {
      Eigen::Vector3i idx;
      sdf_map_->posToIndex(cell, idx);
      if (!(knownfree(idx) && isNeighborUnknown(idx)))
        return true;
    }
    return false;
  }

  bool FrontierFinder::isSurfaceFrontierClusterChanged(const SurfaceFrontierCluster &ft)
  {
    for (auto cell : ft.cells_)
    {
      // Eigen::Vector3i idx;
      // idx = sdf_map_->AddressToIndex(cell.first);
      if (!isSurfaceFrontier(cell.first))  // TODO 换成isSurfaceFrontier函数？
        return true;
    }
    return false;
  }

  void FrontierFinder::computeFrontierInfo(Frontier &ftr)
  {
    // Compute average position and bounding box of cluster
    ftr.average_.setZero();
    ftr.box_max_ = ftr.cells_.front();
    ftr.box_min_ = ftr.cells_.front();
    for (auto cell : ftr.cells_)
    {
      ftr.average_ += cell;
      for (int i = 0; i < 3; ++i)
      {
        ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
        ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
      }
    }
    ftr.average_ /= double(ftr.cells_.size());

    // Compute downsampled cluster
    downsample(ftr.cells_, ftr.filtered_cells_);
  }

  void FrontierFinder::computeFrontiersToVisit()
  {
    first_new_ftr_ = frontiers_.end();
    int new_num = 0;
    int new_dormant_num = 0;
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto &tmp_ftr : tmp_frontiers_)
    {
      // Search viewpoints around frontier
      sampleViewpoints(tmp_ftr);
      if (!tmp_ftr.viewpoints_.empty())
      {
        ++new_num;
        list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
        // Sort the viewpoints by coverage fraction, best view in front
        sort(
            inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
            [](const Viewpoint &v1, const Viewpoint &v2)
            { return v1.visib_num_ > v2.visib_num_; });
        if (first_new_ftr_ == frontiers_.end())
          first_new_ftr_ = inserted;
      }
      else
      {
        // Find no viewpoint, move cluster to dormant list
        dormant_frontiers_.push_back(tmp_ftr);
        ++new_dormant_num;
      }
    }
    // Reset indices of frontiers
    int idx = 0;
    for (auto &ft : frontiers_)
    {
      ft.id_ = idx++;
      // std::cout << ft.id_ << ", ";
    }
    // std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
    // std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
    //           << std::endl;
  }

  void FrontierFinder::getTopViewpointsInfo(
      const Vector3d &cur_pos, vector<Eigen::Vector3d> &points, vector<double> &yaws,
      vector<Eigen::Vector3d> &averages)
  {
    points.clear();
    yaws.clear();
    averages.clear();
    for (auto frontier : frontiers_)
    {
      bool no_view = true;
      for (auto view : frontier.viewpoints_)
      {
        // Retrieve the first viewpoint that is far enough and has highest coverage
        if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
          continue;
        points.push_back(view.pos_);
        yaws.push_back(view.yaw_);
        averages.push_back(frontier.average_);
        no_view = false;
        break;
      }
      if (no_view)
      {
        // All viewpoints are very close, just use the first one (with highest coverage).
        auto view = frontier.viewpoints_.front();
        points.push_back(view.pos_);
        yaws.push_back(view.yaw_);
        averages.push_back(frontier.average_);
      }
    }
  }

  void FrontierFinder::getViewpointsInfo(
      const Vector3d &cur_pos, const vector<int> &ids, const int &view_num, const double &max_decay,
      vector<vector<Eigen::Vector3d>> &points, vector<vector<double>> &yaws)
  {
    points.clear();
    yaws.clear();
    for (auto id : ids)
    {
      // Scan all frontiers to find one with the same id
      for (auto frontier : frontiers_)
      {
        if (frontier.id_ == id)
        {
          // Get several top viewpoints that are far enough
          vector<Eigen::Vector3d> pts;
          vector<double> ys;
          int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
          for (auto view : frontier.viewpoints_)
          {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
              break;
            if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
              continue;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
          if (pts.empty())
          {
            // All viewpoints are very close, ignore the distance limit
            for (auto view : frontier.viewpoints_)
            {
              if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
                break;
              pts.push_back(view.pos_);
              ys.push_back(view.yaw_);
            }
          }
          points.push_back(pts);
          yaws.push_back(ys);
        }
      }
    }
  }

  void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>> &clusters)
  {
    clusters.clear();
    for (auto frontier : frontiers_)
      clusters.push_back(frontier.cells_);
    // clusters.push_back(frontier.filtered_cells_);
  }

  void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>> &clusters)
  {
    clusters.clear();
    for (auto ft : dormant_frontiers_)
      clusters.push_back(ft.cells_);
  }

  void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes)
  {
    boxes.clear();
    for (auto frontier : frontiers_)
    {
      Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
      Vector3d scale = frontier.box_max_ - frontier.box_min_;
      boxes.push_back(make_pair(center, scale));
    }
  }

  void FrontierFinder::getPathForTour(
      const Vector3d &pos, const vector<int> &frontier_ids, vector<Vector3d> &path)
  {
    // Make an frontier_indexer to access the frontier list easier
    vector<list<Frontier>::iterator> frontier_indexer;
    for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
      frontier_indexer.push_back(it);

    // Compute the path from current pos to the first frontier
    vector<Vector3d> segment;
    ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
    path.insert(path.end(), segment.begin(), segment.end());

    // Get paths of tour passing all clusters
    for (int i = 0; i < frontier_ids.size() - 1; ++i)
    {
      // Move to path to next cluster
      auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
      int next_idx = frontier_ids[i + 1];
      for (int j = 0; j < next_idx; ++j)
        ++path_iter;
      path.insert(path.end(), path_iter->begin(), path_iter->end());
    }
  }

  void FrontierFinder::getFullCostMatrix(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
      Eigen::MatrixXd &mat)
  {
    if (false)
    {
      // Use symmetric TSP formulation
      int dim = frontiers_.size() + 2;
      mat.resize(dim, dim); // current pose (0), sites, and virtual depot finally

      int i = 1, j = 1;
      for (auto ftr : frontiers_)
      {
        for (auto cs : ftr.costs_)
          mat(i, j++) = cs;
        ++i;
        j = 1;
      }

      // Costs from current pose to sites
      for (auto ftr : frontiers_)
      {
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(0, j) = mat(j, 0) =
            ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
        ++j;
      }
      // Costs from depot to sites, the same large vaule
      for (j = 1; j < dim - 1; ++j)
      {
        mat(dim - 1, j) = mat(j, dim - 1) = 100;
      }
      // Zero cost to depot to ensure connection
      mat(0, dim - 1) = mat(dim - 1, 0) = -10000;
    }
    else
    {
      // Use Asymmetric TSP
      int dimen = frontiers_.size();
      mat.resize(dimen + 1, dimen + 1);
      // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
      // Fill block for clusters
      int i = 1, j = 1;
      for (auto ftr : frontiers_)
      {
        for (auto cs : ftr.costs_)
        {
          // std::cout << "(" << i << ", " << j << ")"
          // << ", ";
          mat(i, j++) = cs;
        }
        ++i;
        j = 1;
      }
      // std::cout << "" << std::endl;

      // Fill block from current state to clusters
      mat.leftCols<1>().setZero();
      for (auto ftr : frontiers_)
      {
        // std::cout << "(0, " << j << ")"
        // << ", ";
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(0, j++) =
            ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      }
      // std::cout << "" << std::endl;
    }
  }

  void FrontierFinder::findViewpoints(
      const Vector3d &sample, const Vector3d &ftr_avg, vector<Viewpoint> &vps)
  {
    if (!sdf_map_->isInBox(sample) ||
        sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
      return;

    double left_angle_, right_angle_, vertical_angle_, ray_length_;

    // Central yaw is determined by frontier's average position and sample
    auto dir = ftr_avg - sample;
    double hc = atan2(dir[1], dir[0]);

    vector<int> slice_gains;
    // Evaluate info gain of different slices
    for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18)
    {
      // Compute gain of one slice
      int gain = 0;
      for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3)
      {
        // Find endpoint of a ray
        Vector3d end;
        end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
        end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
        end[2] = sample[2] + ray_length_ * sin(phi_v);

        // Do raycasting to check info gain
        Vector3i idx;
        raycaster_->input(sample, end);
        while (raycaster_->nextId(idx))
        {
          // Hit obstacle, stop the ray
          if (sdf_map_->getInflateOccupancy(idx) == 1 || !sdf_map_->isInBox(idx))
            break;
          // Count number of unknown cells
          if (sdf_map_->getOccupancy(idx) == SDFMap::FREE)
            gain++;
        }
      }
      slice_gains.push_back(gain);
    }

    // Sum up slices' gain to get different yaw's gain
    vector<pair<double, int>> yaw_gains;
    for (int i = 0; i < 6; ++i) // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
    {
      double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
      int gain = 0;
      for (int j = 2 * i; j < 2 * i + 9; ++j) // 80 degree hFOV, 9 slices
        gain += slice_gains[j];
      yaw_gains.push_back(make_pair(yaw, gain));
    }

    // Get several yaws with highest gain
    vps.clear();
    sort(
        yaw_gains.begin(), yaw_gains.end(),
        [](const pair<double, int> &p1, const pair<double, int> &p2)
        {
          return p1.second > p2.second;
        });
    for (int i = 0; i < 3; ++i)
    {
      if (yaw_gains[i].second < min_visib_num_)
        break;
      Viewpoint vp = {sample, yaw_gains[i].first, yaw_gains[i].second};
      while (vp.yaw_ < -M_PI)
        vp.yaw_ += 2 * M_PI;
      while (vp.yaw_ > M_PI)
        vp.yaw_ -= 2 * M_PI;
      vps.push_back(vp);
    }
  }

  // Sample viewpoints around frontier's average position, check coverage to the frontier cells
  void FrontierFinder::sampleViewpoints(Frontier &frontier)
  {
    // Evaluate sample viewpoints on circles, find ones that cover most cells
    for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
         rc <= candidate_rmax_ + 1e-3; rc += dr)
      for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_)
      {
        const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

        // Qualified viewpoint is in bounding box and in safe region
        if (!sdf_map_->isInBox(sample_pos) ||
            sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
          continue;

        // Compute average yaw
        auto &cells = frontier.filtered_cells_;
        Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
        double avg_yaw = 0.0;
        for (int i = 1; i < cells.size(); ++i)
        {
          Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
          double yaw = acos(dir.dot(ref_dir));
          if (ref_dir.cross(dir)[2] < 0)
            yaw = -yaw;
          avg_yaw += yaw;
        }
        avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
        wrapYaw(avg_yaw);
        // Compute the fraction of covered and visible cells
        int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
        if (visib_num > min_visib_num_)
        {
          Viewpoint vp = {sample_pos, avg_yaw, visib_num};
          frontier.viewpoints_.push_back(vp);
          // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
        }
        // }
      }
  }

  void FrontierFinder::coverSurfaceFrontier()
  {
    for(auto cell:target_viewpoint_attached_frontiers_)
    {
      covered_surface_frontier_.insert(cell);
    }
  }

  bool FrontierFinder::isSurfaceFrontierCovered()
  {
    // 条件1 离vp距离小于阈值
    if((current_pos_-target_viewpoint_).norm() < 3.0)
    {
      for(auto cell:target_viewpoint_attached_frontiers_)
      {
        covered_surface_frontier_.insert(cell);
      }


      // int frontier_id = surface_frontier_array_[*target_viewpoint_attached_frontiers_.begin()];

      // // 删除对应的视点,以及视点聚类
      // if(viewpoints_.count(frontier_id))
      // {
      //   int cluster_id = viewpoints_.at(frontier_id).viewpoint_cluster_id_;
      //   if(cluster_id!=-1)
      //   {
      //     auto cluster = viewpoint_clusters_.at(cluster_id);
      //     for(auto &cell:cluster.cells_)
      //     {
      //       viewpoints_.at(cell).is_clustered_ = false;
      //       viewpoints_.at(cell).viewpoint_cluster_id_ = -1;
      //     }
      //     viewpoint_clusters_.erase(cluster_id);
      //     viewpoint_cluster_ids_.push(cluster_id);     
      //   }
      //   viewpoints_.erase(frontier_id);
      // }
      // eraseSurfaceFrontierCluster(frontier_id);
      
      return true;
    }

    // 条件2 vp对应frontier的被覆盖数百分比大于阈值
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max);
    int change_thresh = target_viewpoint_attached_frontiers_.size() * min_view_finish_fraction_; // 0.6还不错
    int change_num = 0;

    for(auto frontier:target_viewpoint_attached_frontiers_)
    {
      // Vector3i idx;
      // idx = sdf_map_->AddressToIndex(frontier);
      if (!isSurfaceFrontier(frontier))
      {
        change_num++;
        if(change_num > change_thresh)
          return true;
      }
    }
    return false;
  }

  void FrontierFinder::coverSurfaceFrontier(vector<int> &frontiers)
  {
    if(frontiers.empty())
      return;
    
    auto frontier = frontiers.at(0);
    if(surface_frontier_array_.count(frontier))
    {
      int cluster_id = surface_frontier_array_.at(frontier);
      if(surface_frontier_clusters_.count(cluster_id))
      {
        for(auto cell : frontiers)
        {
          covered_surface_frontier_.insert(cell);
          surface_frontier_flag_.erase(cell);
          surface_frontier_array_.erase(cell);
        }
        // 删除对应的视点,以及视点聚类
        if(viewpoints_.count(cluster_id))
        {   
          int viewpoint_cluster_id = viewpoints_.at(cluster_id).viewpoint_cluster_id_;
          if(viewpoint_cluster_id!=-1)
          {
            auto viewpoint_cluster = viewpoint_clusters_.at(viewpoint_cluster_id);
            for(auto &cell:viewpoint_cluster.cells_)
            {
              viewpoints_.at(cell).is_clustered_ = false;
              viewpoints_.at(cell).viewpoint_cluster_id_ = -1;
            }
            viewpoint_clusters_.erase(viewpoint_cluster_id);
            viewpoint_cluster_ids_.push(viewpoint_cluster_id);   
            ROS_WARN("[coverSurfaceFrontier]:erase viewpoint cluster");  
          }
          viewpoints_.erase(cluster_id);
          ROS_WARN("[coverSurfaceFrontier]:erase viewpoint");
        }
        eraseSurfaceFrontierCluster(cluster_id);
        ROS_WARN("[coverSurfaceFrontier]:erase frontier cluster");  
      }
    }
   
    return;
  }

  bool FrontierFinder::isSurfaceFrontierCovered2()
  {
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max);
    for(auto &iter:surface_frontier_clusters_)
    {
      auto &cluster = iter.second;
      if (haveOverlap(cluster.box_min_, cluster.box_max_, update_min, update_max)
        && cluster.can_be_observed_)
      {
        const int change_thresh = min_view_finish_fraction_ * cluster.covered_cells_.size();
        int change_num = 0;
        for (auto cell_iter : cluster.cells_) 
        {
          Eigen::Vector3i idx = sdf_map_->AddressToIndex(cell_iter.first);
          if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
            return true;
        }
      }
    }
    return false;
  }

  bool FrontierFinder::isFrontierCovered()
  {
    Vector3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max);

    auto checkChanges = [&](const list<Frontier> &frontiers)
    {
      for (auto ftr : frontiers)
      {
        if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max))
          continue;
        const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
        int change_num = 0;
        for (auto cell : ftr.cells_)
        {
          Eigen::Vector3i idx;
          sdf_map_->posToIndex(cell, idx);
          if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
            return true;
        }
      }
      return false;
    };

    if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_))
      return true;

    return false;
  }

  bool FrontierFinder::isNearUnknown(const Eigen::Vector3d &pos)
  {
    const int vox_num = floor(min_candidate_clearance_ / resolution_);
    for (int x = -vox_num; x <= vox_num; ++x)
      for (int y = -vox_num; y <= vox_num; ++y)
        for (int z = -1; z <= 1; ++z)
        {
          Eigen::Vector3d vox;
          vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
          if (sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN)
            return true;
        }
    return false;
  }

  bool FrontierFinder::isVisible(const Eigen::Vector3d &viewpoint, const Eigen::Vector3d &frontier)
  {
    // if((frontier - viewpoint).norm() < sdf_map_->getMaxRayLength()
      // && sdf_map_->isCollisionFreeStraight(frontier, viewpoint)) 
    if((frontier - viewpoint).norm() < sdf_map_->getMaxRayLength()) 
    {
      Vector3d diff = frontier - viewpoint;
      // 计算与 Z 轴的夹角（实际上是与 XY 平面的夹角）
      double angle = asin(abs(diff.z() / diff.norm())) / M_PI * 180.0;

      if(angle < 45.0)
      {
        Vector3i idx;
        raycaster_->input(frontier, viewpoint);
        bool visib = true;
        while (raycaster_->nextId(idx))
        {
          // if (sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED ||
          //     sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          if (sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED)
          {
            return false;
          }
        }
        return true;
      }
    }
    return false;

    
    // Vector3i idx;
    // raycaster_->input(frontier, viewpoint);
    // bool visib = true;
    // while (raycaster_->nextId(idx))
    // {
    //   if (sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED ||
    //       sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
    //   {
    //     visib = false;
    //     break;
    //   }
    // }

    // // if((frontier - viewpoint).norm() < sdf_map_->getMaxRayLength()
    //   // && sdf_map_->isCollisionFreeStraight(frontier, viewpoint)) 
    // if((frontier - viewpoint).norm() < sdf_map_->getMaxRayLength() && visib) 
    // {
    //   Vector3d diff = frontier - viewpoint;
    //   // 计算与 Z 轴的夹角（实际上是与 XY 平面的夹角）
    //   double angle = asin(abs(diff.z() / diff.norm())) / M_PI * 180.0;

    //   if(angle < 45.0)
    //     return true;
    // }
    // return false;
  }

  int FrontierFinder::countVisibleCells(
      const Eigen::Vector3d &pos, const double &yaw, const vector<Eigen::Vector3d> &cluster)
  {
    percep_utils_->setPose(pos, yaw);
    int visib_num = 0;
    Eigen::Vector3i idx;
    for (auto cell : cluster)
    {
      // Check if frontier cell is inside FOV
      if (!percep_utils_->insideFOV(cell))
        continue;

      // Check if frontier cell is visible (not occulded by obstacles)
      raycaster_->input(cell, pos);
      bool visib = true;
      while (raycaster_->nextId(idx))
      {
        if (sdf_map_->getInflateOccupancy(idx) == 1 ||
            sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
        {
          visib = false;
          break;
        }
      }
      if (visib)
        visib_num += 1;
    }
    return visib_num;
  }

  void FrontierFinder::downsample(
      const vector<Eigen::Vector3d> &cluster_in, vector<Eigen::Vector3d> &cluster_out)
  {
    // downsamping cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto cell : cluster_in)
      cloud->points.emplace_back(cell[0], cell[1], cell[2]);

    const double leaf_size = sdf_map_->getResolution() * down_sample_;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloudf);

    cluster_out.clear();
    for (auto pt : cloudf->points)
      cluster_out.emplace_back(pt.x, pt.y, pt.z);
  }

  void FrontierFinder::wrapYaw(double &yaw)
  {
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
  }

  Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i &pt)
  {
    queue<Eigen::Vector3i> init_que;
    vector<Eigen::Vector3i> nbrs;
    Eigen::Vector3i cur, start_idx;
    init_que.push(pt);
    // visited_flag_[toadr(pt)] = 1;

    while (!init_que.empty())
    {
      cur = init_que.front();
      init_que.pop();
      if (knownfree(cur))
      {
        start_idx = cur;
        break;
      }

      nbrs = sixNeighbors(cur);
      for (auto nbr : nbrs)
      {
        int adr = toadr(nbr);
        // if (visited_flag_[adr] == 0)
        // {
        //   init_que.push(nbr);
        //   visited_flag_[adr] = 1;
        // }
      }
    }
    return start_idx;
  }

  inline vector<Eigen::Vector3i> FrontierFinder::fourNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(4);
    Eigen::Vector3i tmp;

    tmp = voxel - Eigen::Vector3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = voxel + Eigen::Vector3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 1, 0);
    neighbors[3] = tmp;

    return neighbors;
  }

  inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(6);
    Eigen::Vector3i tmp;

    tmp = voxel - Eigen::Vector3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = voxel + Eigen::Vector3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 1, 0);
    neighbors[3] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 0, 1);
    neighbors[4] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 0, 1);
    neighbors[5] = tmp;

    return neighbors;
  }

  inline vector<Eigen::Vector3i> FrontierFinder::eightNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(8);
    Eigen::Vector3i tmp;

    tmp = voxel - Eigen::Vector3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = voxel + Eigen::Vector3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 1, 0);
    neighbors[3] = tmp;

    tmp = voxel - Eigen::Vector3i(1, 1, 0);
    neighbors[4] = tmp;
    tmp = voxel - Eigen::Vector3i(1, -1, 0);
    neighbors[5] = tmp;
    tmp = voxel - Eigen::Vector3i(-1, 1, 0);
    neighbors[6] = tmp;
    tmp = voxel - Eigen::Vector3i(-1, -1, 0);
    neighbors[7] = tmp;

    return neighbors;
  }

  // inline vector<Eigen::Vector3i> FrontierFinder::fiveFiveNeighbors(const Eigen::Vector3i &voxel)
  // {
  //   vector<Eigen::Vector3i> neighbors(125);
  //   Eigen::Vector3i tmp;

  //   tmp = voxel - Eigen::Vector3i(1, 0, 0);
  //   neighbors[0] = tmp;
  //   tmp = voxel + Eigen::Vector3i(1, 0, 0);
  //   neighbors[1] = tmp;
  //   tmp = voxel - Eigen::Vector3i(0, 1, 0);
  //   neighbors[2] = tmp;
  //   tmp = voxel + Eigen::Vector3i(0, 1, 0);
  //   neighbors[3] = tmp;
  //   tmp = voxel - Eigen::Vector3i(0, 0, 1);
  //   neighbors[4] = tmp;
  //   tmp = voxel + Eigen::Vector3i(0, 0, 1);
  //   neighbors[5] = tmp;

  //   return neighbors;
  // }

  inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(10);
    Eigen::Vector3i tmp;
    int count = 0;

    for (int x = -1; x <= 1; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        if (x == 0 && y == 0)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, 0);
        neighbors[count++] = tmp;
      }
    }
    neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
    neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
    return neighbors;
  }

  inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(26);
    Eigen::Vector3i tmp;
    int count = 0;
    for (int x = -1; x <= 1; ++x)
      for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
        {
          if (x == 0 && y == 0 && z == 0)
            continue;
          tmp = voxel + Eigen::Vector3i(x, y, z);
          neighbors[count++] = tmp;
        }
    return neighbors;
  }

  inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const int &addr)
  {
    Eigen::Vector3i voxel = sdf_map_->AddressToIndex(addr);
    return allNeighbors(voxel);
  }

  inline bool FrontierFinder::isPlaneNeighborUnknown(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = fourNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = sixNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isNeighborFree(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = allNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (sdf_map_->getOccupancy(nbr) == SDFMap::FREE)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isPlaneNeighborOccupied(const Eigen::Vector3i &voxel)
  {
    auto nbrs = eightNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED)
        return true;
    }

    //  if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
            
    return false;
  }

  inline bool FrontierFinder::isNeighborObstacle(const Eigen::Vector3i &voxel)
  {
    bool is_neighbor = false;
    auto nbrs = allNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      int ind = sdf_map_->toAddress(nbr);
      if(obstacle_cells_.find(ind)!=obstacle_cells_.end())
      {
        is_neighbor = true;
      }
    }       
    return is_neighbor;
  }

  inline bool FrontierFinder::isNeighborOccupied(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    // auto nbrs = sixNeighbors(voxel);
    auto nbrs = allNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED)
        return true;
    }
    return false;
  }

  inline int FrontierFinder::toadr(const Eigen::Vector3i &idx)
  {
    return sdf_map_->toAddress(idx);
  }
  

  inline bool FrontierFinder::knownfree(const Eigen::Vector3i &idx)
  {
    return sdf_map_->getOccupancy(idx) == SDFMap::FREE;
  }

  inline bool FrontierFinder::inmap(const Eigen::Vector3i &idx)
  {
    return sdf_map_->isInMap(idx);
  }

  geometry_msgs::Point FrontierFinder::toGeometryPoint(const Vector3d &pos)
  {
    geometry_msgs::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    return point;
  }

  std_msgs::ColorRGBA FrontierFinder::getRandomColor(int id, double alpha)
  {
    static const vector<vector<double>> colors = 
    {
      {1.0, 0.0, 0.0}, // Red
      {1.0, 0.5, 0.0}, // Orange
      {0.0, 1.0, 0.0}, // Green
      {0.0, 1.0, 1.0}, // Cyan
      {0.0, 0.0, 1.0}, // Blue
      {0.8, 0.0, 1.0}  // Purple
    };

    std_msgs::ColorRGBA color;
    color.r = colors[id%6][0]; 
    color.g = colors[id%6][1]; 
    color.b = colors[id%6][2]; 
    color.a = alpha;
    return color;
  }

  void
  FrontierFinder::generateMarkerArray(const std::string &tf_frame, visualization_msgs::Marker &frontier_cells,
                                      std::unordered_set<int> &frontier_cells_set, std_msgs::ColorRGBA &rgba)
  {

    double size = sdf_map_->getResolution();
    frontier_cells.header.frame_id = tf_frame;
    frontier_cells.ns = "frontiers";
    frontier_cells.id = 0;
    frontier_cells.type = visualization_msgs::Marker::CUBE_LIST;
    frontier_cells.scale.x = size;
    frontier_cells.scale.y = size;
    frontier_cells.scale.z = size;
    frontier_cells.color = rgba;
    frontier_cells.pose.orientation.w = 1;

    for (const auto &iter : frontier_cells_set)
    {
      geometry_msgs::Point cube_center;
      Eigen::Vector3d cube_pos = sdf_map_->AddressToPos(iter);
      cube_center.x = cube_pos.x();
      cube_center.y = cube_pos.y();
      cube_center.z = cube_pos.z();

      frontier_cells.points.push_back(cube_center);

      // frontier_cells.colors.push_back(rgba);
    }

    if (!frontier_cells.points.empty())
    {
      frontier_cells.action = visualization_msgs::Marker::ADD;
    }
    else
    {
      frontier_cells.action = visualization_msgs::Marker::DELETE;
    }
  }
} // namespace fast_planner