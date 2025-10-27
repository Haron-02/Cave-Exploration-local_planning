#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <list>
#include <utility>
#include <mutex>
#include <random>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <active_perception/frontierStatistics.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/uniform_sampling.h>

// #include "perception2grids/map_2d_manager.h"
// #include "perception3d/map_3d_manager.h"

#include "graph/plan_graph_ikdtree.h"
#include "active_perception/topo_graph_ikdtree.h" 
#include <active_perception/graph_node.h>

#include "path_searching/astar3.h"
#include "utils/point3d.h"

#include "time_utils/time_utils.h"
#include "file_utils/file_rw.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using Eigen::Vector3d;
using std::list;
using std::mutex;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using namespace utils;

class RayCaster;

namespace preprocess
{
  class IkdTreeTopoGraph;
}

namespace fast_planner
{
  class EDTEnvironment;
  class PerceptionUtils;
  class SDFMap;
  class Map2DManager;
  class Map3DManager;
  
// std::shared_ptr<IkdTreeTopoGraph> Ptr;
  // Viewpoint to cover a frontier cluster
  struct Viewpoint
  {
    // Position and heading
    Vector3d pos_;
    double yaw_;
    // Fraction of the cluster that can be covered
    // double fraction_;
    int visib_num_;
    // Frontier cluster id
    int frontier_cluster_id_;
    int viewpoint_cluster_id_ = -1;
    bool is_vertex_ = true;
    std::unordered_set<int> neighbor_viewpoint_ids_; // 邻居聚类的id
    int id_;
    bool is_clustered_ = false;
    double gain_ = 0;
    double cover_num_ = 0;
    unordered_set<int> covered_cells_;
  };

  struct ViewpointCluster
  {
    int id_;
    std::unordered_set<int> cells_;
    vector<int> ordered_cells_;
    vector<int> vertex_cells_;
    vector<int> child_cluster_ids_;
    vector<Vector3d> vertex_pos_;

    Vector3d dict_;
    double eigenvalue_;

    bool is_circle = true;
    int forward_source_;
    int forward_target_ = -1;

    int backward_source_;
    int backward_target_ = -1;

    int merged_level_ = 0;
    Vector3d avg_pos_;
    bool is_merged_ = false;
    double dist_ = 0;
    double avg_gain_ = 0;
    double avg_cover_num_ = 0;
  };

  // A frontier cluster, the viewpoints to cover it
  struct Frontier
  {
    // Complete voxels belonging to the cluster
    vector<Vector3d> cells_;
    // down-sampled voxels filtered by voxel grid filter
    vector<Vector3d> filtered_cells_;
    // Average position of all voxels
    Vector3d average_;
    // Idx of cluster
    int id_;
    // Viewpoints that can cover the cluster
    vector<Viewpoint> viewpoints_;
    // Bounding box of cluster, center & 1/2 side length
    Vector3d box_min_, box_max_;
    // Path and cost from this cluster to other clusters
    list<vector<Vector3d>> paths_;
    list<double> costs_;
  };

  struct Obstacle
  {
    // Complete voxels belonging to the cluster
    std::unordered_set<int> cells_;
    vector<int> cells_array_; // 与cells_存的东西相同,用于可视化时的并行处理
    // down-sampled voxels filtered by voxel grid filter
    vector<std::unordered_set<int>> surface_frontier_;
    std::unordered_set<int> surface_frontier_ids_;
    std::unordered_map<int, int> surface_frontier_array_;
    // vector<Vector3d> surface_frontier_center_;
    Vector3d selected_cluster_center_;
    int selected_cluster_center_index_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    // Idx of cluster
    int id_;
  };

  struct SurfaceFrontier
  {
    vector<int> neighbor_obstacle_index_;
    // id of map
    int id_;
  };

  struct SurfaceFrontierCluster
  {
    vector<int> neighbor_obstacle_index_;
    int obstacle_cluster_id_;
    std::unordered_map<int, Vector3d> cells_;
    std::unordered_set<int> covered_cells_;
    // id of map
    int id_;
    Vector3d box_min_, box_max_;
    bool is_compute_normal_ = false;
    bool is_sampled_viewpoint_ = false;
    bool can_be_observed_ = false;
    vector<Vector3d> viewpoints_;
    vector<double> viewpoint_gains_;
    Vector3d max_gain_viewpoint_;
    double max_gain_ = 0;
    double cover_range_ = 0;
    Vector3d average_pos_;
    Vector3d average_normal_;
    // for cluster merge
    std::unordered_set<int> neighbor_cluster_ids_; // 邻居聚类的id
    int merged_level_ = 0;
    vector<int> merged_cluster_ids_;
    bool is_merged_ = false;
  };

  struct AdjNode
  {
    int type_ = 0; // 0-viewpoint, 1-vertex, 2-interpoint
    int viewpoint_id_ = -1;
    int viewpoint_cluster_id_ = -1;
    Vector3d pos_;
  };

  struct VisualizeParam
  {
    std::unordered_set<int> pubed_frontier_cluster_ids_;
    std::unordered_set<int> erased_frontier_cluster_ids_;

    std::unordered_set<int> erased_viewpoint_cluster_ids_;
    std::unordered_set<int> changed_viewpoint_cluster_ids_;
  };

  class FrontierFinder
  {
  public:
    typedef shared_ptr<FrontierFinder> Ptr;

    FrontierFinder(const shared_ptr<SDFMap> &sdf_map, ros::NodeHandle &nh);
    ~FrontierFinder();

    void tmp(){int a = 1;}

    void simpleSearchFrontiers();
    void processChangedcells();
    void updateGlobalFrontier();

    bool isPlaneNeighborObstacle(const Eigen::Vector3i &voxel, vector<int> &neighbor_obstacle_index);
    bool isNeighborObstacle(const Eigen::Vector3i &voxel);

    void updateForwardDirectory(Vector3d forward_directory);
   
    bool getSurfaceFrontierCluster(Vector3d &cluster_center);

    void updateObstacleCloud();
    void expandObstacle(const int &first);
    
    void expandSurfaceFrontier(const int &first);
    bool isClusteredFrontier(int addr);
    bool isExpandedFrontier(int addr);
   
    bool isSurfaceFrontier(const int &addr);

    bool isFarFromOccupied(Eigen::Vector3d pos, double dist);
    void computeSurfaceFrontierClusterInfo(SurfaceFrontierCluster &surface_frontier_cluster);
    
    void updateSurfaceFrontierCluster();

    void expandSurfaceFrontier2(const int &first);
   
    bool isFrontier(const Eigen::Vector3i &index);
    bool isSurfaceFrontier(const Eigen::Vector3i &index);

    // void setMap3DManager(const perception::Map3DManager::Ptr &map_3d_manager);
    void setRoadMap(const shared_ptr<preprocess::IkdTreeTopoGraph> &road_map);
  
    int generateSurfaceFrontierClusterId();
    void eraseSurfaceFrontierCluster(int id);
    bool isSurfaceFrontierClusterChanged(const SurfaceFrontierCluster &ft);
    Vector3d computeNormalVector(Vector3d pos, 
                                 pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, 
                                 pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation,
                                 int point_num, 
                                 double radius);
    void sampleSurfaceFrontierViewpoints();
    void sampleSurfaceFrontierViewpoints2();
    void sampleSurfaceFrontierViewpoints(SurfaceFrontierCluster &cluster);
    bool isVisible(const Eigen::Vector3d &viewpoint, const Eigen::Vector3d &frontier);
  
    void selectViewPoints();
    int getCoveredFrontierNum(unordered_set<int> &covered_cells, vector<int> &can_covered_cells);
    bool isDetectObstacle();
    void getViewpointsAndGains(std::vector<Vector3d> &viewpoints, std::vector<int> &gains);
    
    bool isSurfaceFrontierCovered();
    bool isSurfaceFrontierCovered2();
    void coverSurfaceFrontier();
    void coverSurfaceFrontier(vector<int> &frontiers);


    void setTargetViewpoint(Vector3d target_viewpoint, std::vector<int> attached_frontiers);
    static bool SortPairInRev(const std::pair<int, int>& a, const std::pair<int, int>& b);
    void createSurfaceFrontierCluster(const vector<int> &expanded, const unordered_set<int> &neighbor_cluster_ids);
    double collisionFreeDistance(const Vector3d &point, const Vector3d &direction, double check_distance);
    double normalize_angle(double angle);

    bool getPathInGridMap(const Vector3d &start_point, const Vector3d &end_point, vector<Vector3d>& path);
    vector<Vector3d> getPathInRoadMap(const Vector3d &start_point, const Vector3d &end_point);
    
    bool getPath(const Vector3d &start_point, const Vector3d &end_point, double road_map_thres, vector<Vector3d>& path);
    
    void shortenPath(vector<Vector3d> &path);
    void shortenPath(vector<Vector3d> &path, int thread_id);

    double getPathLength(const vector<Vector3d>& path);

    bool addPosToGraph(const Vector3d pos);
    // Minimum jerk traj
    void jerk_traj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc, Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                double t, vector<Eigen::MatrixXd> &p, vector<double> &jerk_integrals);

    double calculateMotionTime(double vel, double acc_max, double vel_max, double x);
  
    void sample_traj(vector<Eigen::MatrixXd> &p, double t, vector<Vector3d> &pos_list, vector<Vector3d> &vel_list, vector<Vector3d> &acc_list, vector<double> &time_list);
    Vector3d get_velocity(vector<Eigen::MatrixXd> &p, double t);
    
    bool getNextViewpoint(Vector3d &viewpoint, vector<int> &frontiers);

    void mergeCluster();
    void mergeFrontierCluster(int seed, queue<int> &seed_queue, int merge_level, double dist);
    void unmergeCluster();

    bool getTourPoints(vector<Vector3d> &tour_points, vector<vector<int>> &frontiers);

    bool getTourPoint(Vector3d &tour_points, vector<int> &frontiers);

    void updateViewpointCluster();
    bool can_be_clustered(ViewpointCluster &viewpoint_cluster,
                          Viewpoint &viewpoint,
                          int &nearest_viewpoint_id,
                          double &min_dist);

    void computeViewpointClusterInfo(ViewpointCluster &cluster);

    void pca(const vector<Vector3d>& points, Vector3d &mean, Eigen::Matrix3d &eigenvectors, Eigen::Vector3d &eigenvalues);
    void findVertex(Vector3d &mean, Vector3d &dict, vector<Vector3d> &points, int &vertex_1, int &vertex_2);

    void global_planning2();

    void global_planning3();

    int generateViewpointClusterId();

    void mergeViewpointCluster();

    void viewpointCluster(int seed, queue<int> &seed_queue, int merge_level, double dist);

    void unmergeViewpointCluster();

    void generateAdjList();

    void generateAdjList2();

    void global_planning5();
    void generateAdjList3();
    void calcPathLength2(vector<vector<double>> &cost_mat);

    void local_planning();

    // double get



    bool getFutureStatus(double time, Vector3d &pos, Vector3d  &vel);
    double distToBox(Vector3d &box_min, Vector3d &box_max, Vector3d &point);

    void splitLargeSurfaceFrontiers(vector<int> &new_frontier_ids);
    bool split(SurfaceFrontierCluster &cluster, vector<SurfaceFrontierCluster> &split_clusters);
    void viewpointRefine();
    void viewpointRefine2();
  
    void eraseFrontier(const int &addr);
    void eraseFrontier(const Eigen::Vector3i &index);
    void eraseFrontier(const Eigen::Vector3d &pos);
    bool isPlaneFrontier(const Eigen::Vector3i &index);
    bool isPlaneFrontier(const Eigen::Vector3d &pos);

    std::vector<Eigen::Vector3d> getSimpleFrontiers();
    std::vector<Eigen::Vector3d> getSimpleLocalFrontiers(const double &range);
    std::vector<Eigen::Vector3d> getSimpleClearedFrontiers();
    void resetSimpleClearedFrontiers();
    void setCurrentPose(const geometry_msgs::Pose &current_pos);
    void setCurrentPose(const Vector3d current_pos);
    double getMaxRayLength();
    double getFrontierUpperBound();
    double getFrontierLowerBound();
    double getSensorHeight();
    bool isCollisionFree(const Eigen::Vector3d &source, const Eigen::Vector3d &target);

    void pubStatistics();
    void pubFrontiersMarkers();

    void pubChangeCellsMarkers();
    void pubSurfaceFrontierNormalsMarkers();
  
    void pubObstacleClustersMarkers();
    void pubFrontierClustersMarkers();
    void pubViewPoints();
    void pubSurfaceFrontierPathMarkers();
    void pubOccupancyCloud();
    void pubSurfaceFrontierClustersMarkers();
    void pubGlobalPathMarkers();

    void pubViewpointWithFrontiers();

    void pubMergedFrontiers();

    void pubClusterRelation();

    void pubViewpointClusters();  // 基于历史TSP的视点聚类
    void pubViewpointIndexMarkers();

    void pubViewpointClusters2();
    void pubLocalSurfaceFrontierMarkers(); 

    // local planning
    void pubLocalFrontiers(); 
    void pubLocalViewpoints();
    vector<int> sovleTSP(vector<vector<double>> &cost_mat, int &cost);
    void pubLocalPath();

    // DEBUG
    void pubEndVels();
    void pubCurrentDirection();
    void pubRoadMapPath();
    // DEBUG

    void knownCellOutput();
    std::size_t getKnownNodeNum(const std::unordered_set<int> &known_cells);
    std::size_t getKnownPlaneNodeNum(const std::unordered_set<int> &known_cells);

    void searchFrontiers();
    void computeFrontiersToVisit();

    void getFrontiers(vector<vector<Vector3d>> &clusters);
    void getDormantFrontiers(vector<vector<Vector3d>> &clusters);
    void getFrontierBoxes(vector<pair<Vector3d, Vector3d>> &boxes);
    // Get viewpoint with highest coverage for each frontier
    void getTopViewpointsInfo(const Vector3d &cur_pos, vector<Vector3d> &points, vector<double> &yaws,
                              vector<Vector3d> &averages);
    // Get several viewpoints for a subset of frontiers
    void getViewpointsInfo(const Vector3d &cur_pos, const vector<int> &ids, const int &view_num,
                           const double &max_decay, vector<vector<Vector3d>> &points,
                           vector<vector<double>> &yaws);
    void updateFrontierCostMatrix();
    void getFullCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                           Eigen::MatrixXd &mat);
    void getPathForTour(const Vector3d &pos, const vector<int> &frontier_ids, vector<Vector3d> &path);

    void setNextFrontier(const int &id);
    bool isFrontierCovered();
    void wrapYaw(double &yaw);

    shared_ptr<SDFMap> sdf_map_;
    shared_ptr<PerceptionUtils> percep_utils_;
    // perception::Map2DManager::Ptr map_2d_manager_;
    // perception::Map3DManager::Ptr map_3d_manager_;
    // graph::IkdTreePlanGraph::Ptr graph_;
    shared_ptr<preprocess::IkdTreeTopoGraph> road_map_;
    VisualizeParam vp_;
    bool global_planning_update_;
    nav_msgs::Path cur_traj_;

    unique_ptr<Astar3> Astar_;
    unique_ptr<RayCaster> raycaster_;

    vector<unique_ptr<Astar3>> Astars_;
    vector<unique_ptr<RayCaster>> raycasters_;
    int thread_num_ = 8;
    bool trigger_ = false;
    int iter_ = 0;

    ros::Publisher SDFMap_frontier_statistics_pub_, global_frontiers_pub_, surface_frontier_clusters_pub_, change_cells_pub_, surface_frontiers_normals_pub_;
    ros::Publisher frontier_clusters_pub_;
    ros::Publisher surface_frontier_path_pub_, local_occupancy_cloud_pub_, following_obstacle_cluster_pub_, following_surface_frontier_cluster_pub_, following_surface_frontier_normal_pub_;

    ros::Publisher viewpoints_pub_, max_gain_viewpoint_pub_, viewpoint_coverage_pub_;;
    ros::Publisher global_path_pub_, viewpoint_cluster_lines_pub_;
    ros::Publisher viewpoint_with_frontiers_pub_;
    ros::Publisher merged_clusters_pub_;
    ros::Publisher viewpoint_clusters_pub_;
    ros::Publisher local_surface_frontiers_pub_;

    // local planning
    ros::Publisher local_frontiers_pub_;
    ros::Publisher local_viewpoints_pub_;
    ros::Publisher local_viewpoints_with_frontiers_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher local_all_viewpoints_with_frontiers_pub_;

    // DEBUG
    ros::Publisher end_vels_pub_, currrent_direction_pub_, roadmap_paths_pub_, cluster_relation_pub_;
    ros::Publisher find_path_failed_pub_;
    // DEBUG
    
    mutex cleared_ft_mutex;

  private:
    void splitLargeFrontiers(list<Frontier> &frontiers);
    bool splitHorizontally(const Frontier &frontier, list<Frontier> &splits);
    void mergeFrontiers(Frontier &ftr1, const Frontier &ftr2);
    bool isFrontierChanged(const Frontier &ft);
    bool haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2,
                     const Vector3d &max2);
    void computeFrontierInfo(Frontier &frontier);
    void downsample(const vector<Vector3d> &cluster_in, vector<Vector3d> &cluster_out);
    void sampleViewpoints(Frontier &frontier);

    int countVisibleCells(const Vector3d &pos, const double &yaw, const vector<Vector3d> &cluster);
    bool isNearUnknown(const Vector3d &pos);
    vector<Eigen::Vector3i> fourNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> eightNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> allNeighbors(const int &addr);

    // vector<Eigen::Vector3i> fiveFiveNeighbors(const Eigen::Vector3i &voxel);

    bool isPlaneNeighborUnknown(const Eigen::Vector3i &voxel);
    bool isNeighborUnknown(const Eigen::Vector3i &voxel);
    bool isNeighborFree(const Eigen::Vector3i &voxel);

    bool isPlaneNeighborOccupied(const Eigen::Vector3i &voxel);
    bool isNeighborOccupied(const Eigen::Vector3i &voxel);

    void expandFrontier(const Eigen::Vector3i &first /* , const int& depth, const int& parent_id */);

    // Wrapper of sdf map
    int toadr(const Eigen::Vector3i &idx);
    bool knownfree(const Eigen::Vector3i &idx);
    bool inmap(const Eigen::Vector3i &idx);

    void generateMarkerArray(const std::string &tf_frame, visualization_msgs::Marker &frontier_cells,
                             std::unordered_set<int> &frontier_cells_set, std_msgs::ColorRGBA &rgba);
    // Deprecated
    Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i &pt);
    bool isInBoxes(const vector<pair<Vector3d, Vector3d>> &boxes, const Eigen::Vector3i &idx);
    bool canBeMerged(const Frontier &ftr1, const Frontier &ftr2);
    void findViewpoints(const Vector3d &sample, const Vector3d &ftr_avg, vector<Viewpoint> &vps);

    // Tool
    geometry_msgs::Point toGeometryPoint(const Vector3d &pos);
    std_msgs::ColorRGBA getRandomColor(int seed, double alpha);

    // Data
    vector<char> frontier_flag_;
    list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
    vector<int> removed_ids_;
    list<Frontier>::iterator first_new_ftr_;
    Frontier next_frontier_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_occupancy_cloud_;

    geometry_msgs::Pose current_pose_;
    std::unordered_set<int> changed_cells_;
    std::unordered_set<int> simple_known_cells_;
    std::unordered_set<int> cleared_cells_;
    std::unordered_set<int> new_frontier_cells_;
    std::mutex global_frontier_mutex_;
    std::unordered_set<int> global_frontier_cells_;  // 所有frontier，包括变成virtual_obstacle的
    std::unordered_set<int> history_frontier_cells_;


    // obstacle data
    vector<char> obstacle_flag_;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_;
    std::unordered_set<int> obstacle_cells_;

    // surface_frontier data
    std::unordered_map<int, Vector3d> surface_frontiers_;  // 所有frontiers
    std::unordered_map<int, Vector3d> local_surface_frontiers_;
    std::unordered_map<int, int> surface_frontier_array_; // cell2cluster_ind
    std::unordered_set<int> surface_frontier_flag_;
    std::unordered_set<int> covered_surface_frontier_; // 被覆盖的frontier，防止重复采样vp
    std::unordered_map<int, SurfaceFrontierCluster> surface_frontier_clusters_;
    pcl::KdTreeFLANN<pcl::PointXYZ> local_occupancy_cloud_kdtree_;
    std::unordered_map<int, Vector3d> tmp_normal_; // 暂存被删的cell的法向量

    std::unordered_map<int, Vector3d> new_surface_frontiers_; 

    std::unordered_map<int, SurfaceFrontierCluster> merged_surface_frontier_clusters_;

    std::unordered_set<int> new_surface_frontier_clusters_;

    std::unordered_set<int> merged_cluster_ids_;

    // viewpoint data
    std::unordered_map<int, vector<Vector3d>> surface_frontier_viewpoint_;
    std::vector<Vector3d> selected_viewpoint_;
    Vector3d target_viewpoint_;
    vector<int> target_viewpoint_attached_frontiers_;
    std::unordered_map<int, Viewpoint> viewpoints_;
    std::unordered_map<int, ViewpointCluster> viewpoint_clusters_;
    queue<int> viewpoint_cluster_ids_;
    int viewpoint_cluster_id_;

    // TSP data
    vector<AdjNode> adj_nodes_;
    vector<int> index_map_; // map tsp index to viewpoint id
    vector<int> last_index_map_;  
    vector<int> tsp_index_; // tsp sequence
    vector<int> last_tsp_index_;
    vector<Vector3d> ordered_tour_points_;
    vector<vector<int>> ordered_tour_points_frontiers_;
    unordered_map<int, unordered_map<int, double>> adj_list_;
    unordered_map<int, vector<Vector3d>> viewpoint_paths_; // 当前点到视点的路径
    Point3DMap<Point3D> node_pos_sequence_;
    
    // local planning data
    unordered_map<int, unordered_set<int>> local_frontiers_;
    unordered_set<int> local_frontier_clusters_;
    unordered_map<int, pair<Vector3d, unordered_set<int>>> local_viewpoints_;
    vector<int> local_tsp_index_;
    vector<Vector3d> selected_viewpoints_;
    vector<vector<int>> selected_viewpoints_attach_frontiers_;
    vector<pair<Vector3d, vector<int>>> viewpoints_attach_frontiers_;
    bool use_local_planning_ = false;

    // DEBUG
    vector<Vector3d> end_points_;
    vector<Vector3d> end_vels_;
    vector<Vector3d> failed_viewpoints_;
    vector<vector<Vector3d>> road_map_paths_;
    // DEBUG

    // Time data
    double total_time_;
    int global_planning_iteration_num_;
    double start_time_ms_;

    double total_tsp_solve_time_;
    std::string tsp_solve_time_txt_name_;

    double total_cost_mat_time_;
    std::string cost_mat_time_txt_name_;

    std::string exploration_txt_name_;

    std::string update_frontier_cluster_txt_name_;

    int min_angle_index_;

    std::unordered_set<int> internal_frontier_cells_; // 当前圈的frontier
    int obstacle_cluster_id_;
    int surface_frontier_cluster_id_;
    int main_obstacle_id_;  // 当前正在探索的obstacle cluster的id
    queue<int> surface_frontier_cluster_ids_;

    Vector3d last_surface_frontier_cluster_center_;

    std::size_t known_cell_num_;
    std::size_t known_plane_cell_num_;

    Vector3d forward_directory_;
    Vector3d current_pos_;
    Vector3d current_vel_;

    // Params
    int dim_;
    int cluster_min_;
    int surface_frontier_cluster_min_, obstacle_cluster_min_;
    double surface_frontier_cluster_distance_;
    double surface_frontier_obstacle_dist_;
    double frontier_local_ub_, frontier_local_lb_, frontier_global_ub_, frontier_global_lb_, sensor_height_;
    double cluster_size_xyz_, cluster_size_z_;
    double angle_thres_; // 聚类法向量角度阈值
    double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_, candidate_theta_,
        w_dir_, min_candidate_clearance_;
    int down_sample_;
    double min_view_finish_fraction_, resolution_;
    int min_visib_num_, candidate_rnum_;
    double vm_;
    bool use_sequence_cost_, use_normal_vector_, find_path_twice_, use_velocity_estimate_, use_local_refine_;
    bool viewpoint_check_;

    double total_sequence_cost_;
    double single_sequence_cost_;

    bool disable_simple_finder_;
    string tsp_dir_;
  };

} // namespace fast_planner
#endif