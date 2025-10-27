//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_TOPO_GRAPH_H
#define ROBO_PLANNER_WS_TOPO_GRAPH_H

#include <ros/package.h>
#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include "graph/plan_graph_ikdtree.h"
#include "plan_env/sdf_map.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include "preprocess/viewpoint_manager.h"
// #include "perception2grids/map_2d_manager.h"
// #include "perception3d/map_3d_manager.h"

namespace utils
{
    class Point3D;
}

namespace preprocess
{
    using namespace utils;

    using namespace fast_planner;

    // class ViewpointManager;

    struct TopoGraphParams
    {
        std::string frame_id_;
        float init_x_, init_y_, init_z_;
        float sample_dist_h_;
        float sample_dist_v_;
        float sample_size_;
        float min_interval_h_;
        float min_interval_v_;
        float bound_margin_h_;
        float bound_margin_v_;
        double points_dist_to_occupied_;
        double points_dist_to_unknown_;
        float connectable_range_; // the maximum connectable distance of newly added points during the graph exhibition
        int connectable_num_;     // maximum number of points that every new viewpoint can connect
    };

    class IkdTreeTopoGraph
    {
    public:
        typedef std::shared_ptr<IkdTreeTopoGraph> Ptr;

        Eigen::Vector3d samplepoint_max_bound_, samplepoint_min_bound_;

        bool is_graph_initialized_;

        TopoGraphParams tgp_;

        graph::IkdTreePlanGraph graph_;

        mutable std::shared_timed_mutex road_graph_mutex_;

        explicit IkdTreeTopoGraph(const SDFMap::Ptr &sdf_map, const ros::NodeHandle &nh_private);

        // void setMap2DManager(const Map2DManager::Ptr &map_2d_manager);
        // void setMap3DManager(const Map3DManager::Ptr &map_3d_manager);
        // void setViewpointManager(const std::shared_ptr<ViewpointManager> &viewpoint_manager);

        void setCurrentPosition(const utils::Point3D &current_position);
        void setCurrentPosition(const Eigen::Vector3d current_pos);
        void setSamplePointsBound(const Eigen::Vector3d &max_bound, const Eigen::Vector3d &min_bound);
        void getParamsFromRos();
        double getSampleDistance();

        bool addTwoWayEdge(const int &a_id, const int &b_id);
        bool getPointId(const utils::Point3D &point, int &point_id);
        bool isPointExisted(const utils::Point3D &point) const;
        int addVertex(const utils::Point3D &point, bool update_kdtree = false);
        void updateKdtree();
        bool findShortestPath(const int &start_v_id, const int &end_v_id, vector<Eigen::Vector3d> &shortest_path) const;

        bool nearestSearch(const utils::Point3D &point, utils::Point3D &nearest_point, int &nearest_point_id);
        bool nearRangeSearch(const utils::Point3D &point, const double &range,
                             utils::Point3DQueue &neighbor_vertexs, std::vector<int> &neighbor_vertex_ids);

        // void connectPointToGraph2D(const utils::Point3D &point);
        void connectPointToGraph3D(const utils::Point3D &point);

        void updateTopoGraphByMapAndViewpoints();
        void initTopoGraphByCurrentPositionAndSamplePoints3D(const Point3D &origin_position, const Point3DSet &sample_points);

        void growingByMap3dAndSamplePoints(const Point3DSet &sample_points);

        int addPointToGraph(const Eigen::Vector3d &current_pos);

        template <typename PointListType>
        visualization_msgs::MarkerArray generatePointsMarkers(const PointListType &points,
                                                              const std_msgs::ColorRGBA &color,
                                                              const geometry_msgs::Vector3 &scale) const;
        visualization_msgs::MarkerArray generateTopoGraphMarkers() const;
        void pubGraphMarkers() const;

    private:
        ros::NodeHandle nh_, nh_private_;

        ros::Publisher graph_pub_, graph_pub_2_, graph_pub_3_, graph_pub_4_;
        
        shared_ptr<SDFMap> sdf_map_;
        // Map2DManager::Ptr map_2d_manager_;
        // Map3DManager::Ptr map_3d_manager_;
        // std::shared_ptr<ViewpointManager> viewpoint_manager_;

        
        // std::shared_ptr<graph::IkdTreePlanGraph> graph_ptr(graph_);

        utils::Point3D current_position_;

        Point3DQueue suitable_points_;
        Point3DQueue local_points_;
        Point3DSet sample_points_;
    };
}

#endif // ROBO_PLANNER_WS_TOPO_GRAPH_H
