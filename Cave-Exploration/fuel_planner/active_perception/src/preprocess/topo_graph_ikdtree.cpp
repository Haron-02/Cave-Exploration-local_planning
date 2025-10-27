//
// Created by hjl on 2022/1/10.
//

#include "active_perception/topo_graph_ikdtree.h"
#include "time_utils/time_utils.h"
#include <map>

namespace preprocess
{

    IkdTreeTopoGraph::IkdTreeTopoGraph(
        const ros::NodeHandle &nh_private) : nh_private_(nh_private),
                                             is_graph_initialized_(false)
    {
        getParamsFromRos();
        graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/topo_graph_markers", 1);
        // graph_pub_2_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/suitable_points", 1);
        // graph_pub_3_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/sample_points", 1);
        // graph_pub_4_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/topo_planner/local_points", 1);
    }

    // void IkdTreeTopoGraph::setMap2DManager(const Map2DManager::Ptr &map_2d_manager)
    // {
    //     this->map_2d_manager_ = map_2d_manager;
    // }

    // void IkdTreeTopoGraph::setMap3DManager(const Map3DManager::Ptr &map_3d_manager)
    // {
    //     this->map_3d_manager_ = map_3d_manager;
    // }

    // void IkdTreeTopoGraph::setViewpointManager(const ViewpointManager::Ptr &viewpoint_manager)
    // {
    //     this->viewpoint_manager_ = viewpoint_manager;
    // }

    void IkdTreeTopoGraph::setSamplePointsBound(const Eigen::Vector3d &max_bound, const Eigen::Vector3d &min_bound)
    {
        this->samplepoint_max_bound_ = max_bound;
        this->samplepoint_min_bound_ = min_bound;
    }

    void IkdTreeTopoGraph::getParamsFromRos()
    {
        std::string ns = ros::this_node::getName() + "/Roadmap";
        tgp_.frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", tgp_.frame_id_))
        {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        tgp_.init_x_ = 0.0;
        if (!ros::param::get(ns + "/init_x", tgp_.init_x_))
        {
            ROS_WARN("No init_x specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_x").c_str());
        }

        tgp_.init_y_ = 0.0;
        if (!ros::param::get(ns + "/init_y", tgp_.init_y_))
        {
            ROS_WARN("No init_y specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_y").c_str());
        }

        tgp_.init_z_ = 2.0;
        if (!ros::param::get(ns + "/init_z", tgp_.init_z_))
        {
            ROS_WARN("No init_z specified. Looking for %s. Default is '2.0'.",
                     (ns + "/init_z").c_str());
        }

        tgp_.sample_dist_h_ = 2;
        if (!ros::param::get(ns + "/sample_dist_h", tgp_.sample_dist_h_))
        {
            ROS_WARN("No sample_dist_h specified. Looking for %s. Default is '0.5'.",
                     (ns + "/sample_dist_h").c_str());
        }

        tgp_.points_dist_to_occupied_ = 0.5;
        if (!ros::param::get(ns + "/points_dist_to_occupied", tgp_.points_dist_to_occupied_))
        {
            ROS_WARN("No points_dist_to_occupied specified. Looking for %s. Default is '0.5'.",
                     (ns + "/points_dist_to_occupied").c_str());
        }

        tgp_.points_dist_to_unknown_ = 0.5;
        if (!ros::param::get(ns + "/points_dist_to_unknown", tgp_.points_dist_to_unknown_))
        {
            ROS_WARN("No points_dist_to_unknown specified. Looking for %s. Default is '0.5'.",
                     (ns + "/points_dist_to_unknown").c_str());
        }

        tgp_.sample_dist_v_ = 2;
        if (!ros::param::get(ns + "/sample_dist_v", tgp_.sample_dist_v_))
        {
            ROS_WARN("No sample_dist_v specified. Looking for %s. Default is '0.5'.",
                     (ns + "/sample_dist_v").c_str());
        }

        tgp_.min_interval_h_ = 1.5;
        if (!ros::param::get(ns + "/min_interval_h", tgp_.min_interval_h_))
        {
            ROS_WARN("No min_interval_h specified. Looking for %s. Default is '0.8'.",
                     (ns + "/min_interval_h").c_str());
        }

        tgp_.min_interval_v_ = 1.5;
        if (!ros::param::get(ns + "/min_interval_v", tgp_.min_interval_v_))
        {
            ROS_WARN("No min_interval_v specified. Looking for %s. Default is '0.8'.",
                     (ns + "/min_interval_v").c_str());
        }

        tgp_.bound_margin_h_ = 1.0;
        if (!ros::param::get(ns + "/bound_margin_h", tgp_.bound_margin_h_))
        {
            ROS_WARN("No bound_margin_h specified. Looking for %s. Default is '1.0'.",
                     (ns + "/bound_margin_h").c_str());
        }

        tgp_.bound_margin_v_ = 1.0;
        if (!ros::param::get(ns + "/bound_margin_v", tgp_.bound_margin_v_))
        {
            ROS_WARN("No bound_margin_v specified. Looking for %s. Default is '1.0'.",
                     (ns + "/bound_margin_v").c_str());
        }

        tgp_.connectable_range_ = 5;
        if (!ros::param::get(ns + "/connectable_range", tgp_.connectable_range_))
        {
            ROS_WARN("No connectable_range specified. Looking for %s. Default is 5.",
                     (ns + "/connectable_range").c_str());
        }

        tgp_.connectable_num_ = 5;
        if (!ros::param::get(ns + "/connectable_num", tgp_.connectable_num_))
        {
            ROS_WARN("No connectable_num specified. Looking for %s. Default is 5.",
                     (ns + "/connectable_num").c_str());
        }
    }

    double IkdTreeTopoGraph::getSampleDistance()
    {
        return tgp_.sample_dist_h_;
    }

    void IkdTreeTopoGraph::setCurrentPosition(const Point3D &current_position)
    {
        current_position_ = current_position;
    }

    bool IkdTreeTopoGraph::addTwoWayEdge(const int &a_id, const int &b_id)
    {
        return graph_.addTwoWayEdge(a_id, b_id);
    }

    bool IkdTreeTopoGraph::getPointId(const utils::Point3D &point, int &point_id)
    {
        return graph_.getPointId(point, point_id);
    }

    bool IkdTreeTopoGraph::isPointExisted(const utils::Point3D &point) const
    {
        return graph_.isPoint3DExisted(point);
    }

    int IkdTreeTopoGraph::addVertex(const utils::Point3D &point, bool update_kdtree)
    {
        int id = graph_.addVertex(point);
        if (update_kdtree)
            graph_.addPointsToKdtree();
        return id;
    }

    void IkdTreeTopoGraph::updateKdtree()
    {
        graph_.addPointsToKdtree();
    }

    bool IkdTreeTopoGraph::findShortestPath(const int &start_v_id, const int &end_v_id, utils::Point3DQueue &shortest_path) const
    {
        std::vector<int> waypoint_ids;
        bool flag = graph_.getShortestPath(start_v_id, end_v_id, waypoint_ids, shortest_path);
        return flag;
    }

    bool IkdTreeTopoGraph::nearestSearch(const utils::Point3D &point, utils::Point3D &nearest_point, int &nearest_point_id)
    {
        nearest_point_id = graph_.getNearestVertexId(point);
        if (nearest_point_id == -1)
        {
            nearest_point = utils::Point3D();
            return false;
        }
        else
        {
            nearest_point = graph_.getVertex(nearest_point_id);
            return true;
        }
    }

    bool IkdTreeTopoGraph::nearRangeSearch(const utils::Point3D &point, const double &range,
                                           utils::Point3DQueue &neighbor_vertexs, std::vector<int> &neighbor_vertex_ids)
    {
        neighbor_vertex_ids = graph_.getNeighborVertexsIDs(point, range, neighbor_vertexs);

        if (neighbor_vertex_ids.empty())
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    // void IkdTreeTopoGraph::connectPointToGraph2D(const utils::Point3D &point)
    // {
    //     // 对每个selected_points中的point找邻居
    //     int point_id = -1;
    //     if (graph_.getPointId(point, point_id))
    //     {
    //         if (graph_.getLinkNums(point_id) > tgp_.connectable_num_)
    //             return;

    //         Point3DQueue neighbors_vertexs;
    //         std::vector<int> neighbors_ids = graph_.getNeighborVertexsIDs(point, tgp_.connectable_range_, neighbors_vertexs);

    //         // 对范围搜索的结果进行排序
    //         std::multimap<double, std::pair<int, Point3D>> sorted_results;
    //         for (size_t i = 0; i < neighbors_ids.size(); i++)
    //         {
    //             Point3D neighbor_vertex = neighbors_vertexs[i];
    //             int neighbor_id = neighbors_ids[i];
    //             sorted_results.insert(std::make_pair(
    //                 point.distance(neighbor_vertex), std::make_pair(neighbor_id, neighbor_vertex)));
    //         }

    //         // 每个selected_points中的point都和邻居连connected_num个
    //         int connected_num = 0;
    //         for (const auto &result : sorted_results)
    //         {
    //             auto index_point_pair = result.second;
    //             int neighbor_id = index_point_pair.first;
    //             Point3D neighbor_vertex = index_point_pair.second;

    //             if (point_id != neighbor_id && map_2d_manager_->isCollisionFreeStraight(
    //                                                Point2D(point.x(), point.y()),
    //                                                Point2D(neighbor_vertex.x(), neighbor_vertex.y())))
    //             {
    //                 graph_.addTwoWayEdge(point_id, neighbor_id);
    //                 connected_num++;
    //                 if (connected_num > tgp_.connectable_num_)
    //                     break;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         std::cerr << "selected_points"
    //                   << Eigen::Vector3d(point.x(), point.y(), point.z())
    //                   << " not in graph!" << std::endl;
    //     }
    // }

    void IkdTreeTopoGraph::connectPointToGraph3D(const utils::Point3D &point)
    {
        // connect each local point
        int point_id = -1;
        // 对每个selected_points中的point找邻居
        if (graph_.getPointId(point, point_id))
        {
            if (graph_.getLinkNums(point_id) >= tgp_.connectable_num_)
                return;

            Point3DQueue neighbors_vertexs;
            std::vector<int> neighbors_ids = graph_.getNeighborVertexsIDs(point, tgp_.connectable_range_, neighbors_vertexs);

            // 对范围搜索的结果排序
            std::multimap<double, std::pair<int, Point3D>> sorted_results;
            for (size_t i = 0; i < neighbors_ids.size(); i++)
            {
                Point3D neighbor_vertex = neighbors_vertexs[i];
                int neighbor_id = neighbors_ids[i];
                sorted_results.insert(std::make_pair(
                    point.distance(neighbor_vertex), std::make_pair(neighbor_id, neighbor_vertex)));
            }

            // 每个selected_points中的point都和邻居连若干个
            int connected_num = 0;
            for (const auto &result : sorted_results)
            {
                auto index_point_pair = result.second;
                int neighbor_id = index_point_pair.first;
                Point3D neighbor_vertex = index_point_pair.second;

                if (point_id != neighbor_id &&
                    sdf_map_->isCollisionFreeStraight(
                        Eigen::Vector3d(point.x(), point.y(), point.z()), 
                        Eigen::Vector3d(neighbor_vertex.x(), neighbor_vertex.y(), neighbor_vertex.z())))
                {
                    graph_.addTwoWayEdge(point_id, neighbor_id);
                    connected_num++;
                    if (connected_num > tgp_.connectable_num_)
                        break;
                }
            }
        }
        else
        {
            std::cerr << "selected_points not in graph!" << std::endl;
        }
    }

    void IkdTreeTopoGraph::updateTopoGraphByMapAndViewpoints(const int &planner_dim)
    {
        if (planner_dim == 2)
        {
            if (map_2d_manager_->is_map_updated_)
            {
                Point3DSet sample_points; // points meeting sampling requirements
                for (double x = -map_2d_manager_->m2dp_.map_length_[0] / 2;
                     x <= map_2d_manager_->m2dp_.map_length_[0] / 2; x += tgp_.sample_dist_h_)
                {
                    for (double y = -map_2d_manager_->m2dp_.map_length_[1] / 2;
                         y <= map_2d_manager_->m2dp_.map_length_[1] / 2; y += tgp_.sample_dist_h_)
                    {
                        Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);
                        if (map_2d_manager_->isFree(sample_2d) &&
                            !map_2d_manager_->isNearOccupied(sample_2d, tgp_.points_dist_to_occupied_) &&
                            !map_2d_manager_->isNearUnknown(sample_2d, tgp_.points_dist_to_unknown_))
                        {
                            Point3D sample_point(sample_2d.x(), sample_2d.y(), tgp_.init_z_);
                            sample_points.insert(sample_point);
                        }
                    }
                }

                if (is_graph_initialized_)
                {
                    // growing
                    growingByMap2dAndSamplePoints(sample_points);
                }
                else
                {
                    Point3D init_point(tgp_.init_x_, tgp_.init_y_, tgp_.init_z_);
                    initTopoGraphByCurrentPositionAndSamplePoints2D(init_point, sample_points);
                }
            }
        }
        else if (planner_dim == 3)
        {
            if (map_3d_manager_->is_map_updated_)
            {
                Point3DSet sample_points; // points meeting sampling requirements
                for (double x = -map_3d_manager_->m3dp_.map_length_[0] / 2;
                     x <= map_3d_manager_->m3dp_.map_length_[0] / 2; x += tgp_.sample_dist_h_)
                {
                    for (double y = -map_3d_manager_->m3dp_.map_length_[1] / 2;
                         y <= map_3d_manager_->m3dp_.map_length_[1] / 2; y += tgp_.sample_dist_h_)
                    {
                        for (double z = -map_3d_manager_->m3dp_.map_length_[2] / 2;
                             z <= map_3d_manager_->m3dp_.map_length_[2] / 2; z += tgp_.sample_dist_v_)
                        {
                            GridPoint3D sample_3d(current_position_.x() + x, current_position_.y() + y, current_position_.z() + z);
                            if (map_3d_manager_->isFree(sample_3d) &&
                                !map_3d_manager_->isNearOccupied(sample_3d, tgp_.points_dist_to_occupied_) &&
                                !map_3d_manager_->isNearUnknown(sample_3d, tgp_.points_dist_to_unknown_))
                            {
                                Point3D sample_point(sample_3d.x(), sample_3d.y(), sample_3d.z());
                                sample_points.insert(sample_point);
                            }
                        }
                    }
                }

                if (is_graph_initialized_)
                {
                    // growing
                    growingByMap3dAndSamplePoints(sample_points);
                }
                else
                {
                    Point3D init_point(tgp_.init_x_, tgp_.init_y_, tgp_.init_z_);
                    initTopoGraphByCurrentPositionAndSamplePoints3D(init_point, sample_points);
                }
            }
        }
        else
        {
            std::cerr << "planner_dim is error!" << std::endl;
        }
    }

    void IkdTreeTopoGraph::initTopoGraphByCurrentPositionAndSamplePoints2D(const Point3D &origin_position,
                                                                           const Point3DSet &sample_points)
    {
        graph_.resetGraph();
        if (!sample_points.empty())
        {
            // first add the origin
            graph_.addVertex(origin_position);
            graph_.buildKdtree();
            // use sample points to grow
            growingByMap2dAndSamplePoints(sample_points);

            is_graph_initialized_ = true;
        }
    }

    void IkdTreeTopoGraph::initTopoGraphByCurrentPositionAndSamplePoints3D(const Point3D &origin_position,
                                                                           const Point3DSet &sample_points)
    {
        graph_.resetGraph();
        if (!sample_points.empty())
        {
            // first add the origin
            graph_.addVertex(origin_position);
            graph_.buildKdtree();
            // use sample points to grow
            growingByMap3dAndSamplePoints(sample_points);

            is_graph_initialized_ = true;
        }
    }

    void IkdTreeTopoGraph::growingByMap2dAndSamplePoints(const Point3DSet &sample_points)
    {
        Point3DQueue local_points; // 目前graph中在一定范围内的points

        graph_.getNeighborVertexsIDs(
            current_position_, map_2d_manager_->m2dp_.map_length_[0] * 1.5 + tgp_.connectable_range_, local_points);

        Point3DQueue suitable_points;                // sample_points中可用的points
        Point3DQueue selected_points = local_points; // store points that need to be searched edges' connection relationship

        // for (const auto &point : local_points)
        // {
        //     if (!map_2d_manager_->isFree(Point2D(point.x(), point.y())))
        //     {
        //         graph_.deleteVertex(point);
        //     }
        // }

        for (const auto &point : sample_points)
        {
            bool suitable = true;

            if (!((samplepoint_max_bound_.x() - point.x()) >= 0.5 &&
                  (samplepoint_max_bound_.y() - point.y()) >= 0.5 &&
                  (samplepoint_max_bound_.z() - point.z()) >= 0.5 &&
                  (point.x() - samplepoint_min_bound_.x()) >= 0.5 &&
                  (point.y() - samplepoint_min_bound_.y()) >= 0.5 &&
                  (point.z() - samplepoint_min_bound_.z()) >= 0.5))
            {
                suitable = false;
            }

            if (suitable)
            {
                for (const auto &vertex : selected_points)
                {
                    if (point.distance(vertex) < tgp_.min_interval_h_)
                    { // make sure the new point is not too close to the local graph node
                        suitable = false;
                        break;
                    }
                }
            }
            if (suitable)
            {
                suitable_points.push_back(point);
                selected_points.push_back(point); // suitable_points+local_points
                graph_.addVertex(point);
            }
        }
        graph_.addPointsToKdtree();

        for (const auto &point : selected_points)
        {
            connectPointToGraph2D(point);
        }
    }

    void IkdTreeTopoGraph::growingByMap3dAndSamplePoints(const Point3DSet &sample_points)
    {
        Point3DQueue local_points;

        graph_.getNeighborVertexsIDs(
            current_position_, map_3d_manager_->m3dp_.map_length_[0] * 1.5 + tgp_.connectable_range_, local_points);

        Point3DQueue suitable_points;
        Point3DQueue selected_points = local_points; // store points that need to be searched edges' connection relationship
        for (const auto &point : sample_points)
        {
            bool suitable = true;

            if (!((samplepoint_max_bound_.x() - point.x()) >= tgp_.bound_margin_h_ &&
                  (samplepoint_max_bound_.y() - point.y()) >= tgp_.bound_margin_h_ &&
                  (samplepoint_max_bound_.z() - point.z()) >= tgp_.bound_margin_v_ &&
                  (point.x() - samplepoint_min_bound_.x()) >= tgp_.bound_margin_h_ &&
                  (point.y() - samplepoint_min_bound_.y()) >= tgp_.bound_margin_h_ &&
                  (point.z() - samplepoint_min_bound_.z()) >= tgp_.bound_margin_v_))
            {
                suitable = false;
            }

            if (suitable)
            {
                for (const auto &vertex : selected_points)
                {
                    if (point.distanceXY(vertex) < tgp_.min_interval_h_ &&
                        fabs(point.z() - vertex.z() < tgp_.min_interval_v_))
                    { // make sure the new point is not too close to the local graph node
                        suitable = false;
                        break;
                    }
                }
            }
            if (suitable)
            {
                suitable_points.push_back(point);
                selected_points.push_back(point); // suitable_points + local_points
                graph_.addVertex(point);
            }
        }
        graph_.addPointsToKdtree();

        for (const auto &point : selected_points)
        {
            connectPointToGraph3D(point);
        }
    }

    visualization_msgs::MarkerArray IkdTreeTopoGraph::generateTopoGraphMarkers() const
    {
        visualization_msgs::Marker vertices;
        visualization_msgs::Marker edges;

        // init headers
        vertices.header.frame_id = edges.header.frame_id = tgp_.frame_id_;
        vertices.header.stamp = edges.header.stamp = ros::Time::now();
        vertices.ns = edges.ns = "graph";
        vertices.action = edges.action = visualization_msgs::Marker::ADD;
        vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

        // setting id for each marker
        vertices.id = 0;
        edges.id = 1;

        // defining types
        edges.type = visualization_msgs::Marker::LINE_LIST;
        vertices.type = visualization_msgs::Marker::POINTS;

        // setting scale
        edges.scale.x = 0.05;
        edges.scale.y = 0.05;
        edges.scale.z = 0.05;

        vertices.scale.x = 0.1;
        vertices.scale.y = 0.1;
        vertices.scale.z = 0.1;

        // assigning colors
        vertices.color.r = 1.0f;
        vertices.color.g = 0.0f;
        vertices.color.b = 0.0f;

        edges.color.r = 0.1f;
        edges.color.g = 0.5f;
        edges.color.b = 1.0f;

        vertices.color.a = 1.0f;
        edges.color.a = 0.3f;

        // assignment
        int num = 0;
        geometry_msgs::Point point;
        for (const auto &node : graph_.getAllVertices())
        {   
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            vertices.points.push_back(point);
            num++;
        }
        ROS_INFO("the graph vertices num is %i", num);

        int number = 0;
        for (const auto &items : graph_.getAllEdges())
        {
            for (const auto &item : items.second)
            {
                auto s_vp = graph_.getVertex(items.first);
                auto t_vp = graph_.getVertex(item.first);
                point.x = s_vp.x();
                point.y = s_vp.y();
                point.z = s_vp.z();
                edges.points.push_back(point);
                point.x = t_vp.x();
                point.y = t_vp.y();
                point.z = t_vp.z();
                edges.points.push_back(point);
                number++;
            }
        }
        ROS_INFO("the graph edges num is %i ", number);

        visualization_msgs::MarkerArray graph_markers;
        graph_markers.markers.resize(2);
        graph_markers.markers[0] = vertices;
        graph_markers.markers[1] = edges;

        return graph_markers;
    }

    template <typename PointListType>
    visualization_msgs::MarkerArray IkdTreeTopoGraph::generatePointsMarkers(const PointListType &points,
                                                                            const std_msgs::ColorRGBA &color,
                                                                            const geometry_msgs::Vector3 &scale) const
    {
        visualization_msgs::Marker vertices;

        // init headers
        vertices.header.frame_id = tgp_.frame_id_;
        vertices.header.stamp = ros::Time::now();
        vertices.ns = "graph";
        vertices.action = visualization_msgs::Marker::ADD;
        vertices.pose.orientation.w = 1.0;

        // setting id for each marker
        vertices.id = 0;

        // defining types
        vertices.type = visualization_msgs::Marker::POINTS;

        // setting scale
        vertices.scale.x = scale.x; // 0.1
        vertices.scale.y = scale.y;
        vertices.scale.z = scale.z;

        // assigning colors
        vertices.color.r = color.r; // 1.0f
        vertices.color.g = color.g;
        vertices.color.b = color.b;
        vertices.color.a = color.a;

        // assignment
        int num = 0;
        geometry_msgs::Point point;
        for (const auto &node : points)
        {
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            vertices.points.push_back(point);
            num++;
        }
        ROS_INFO("the graph vertices num is %i", num);

        visualization_msgs::MarkerArray graph_markers;
        graph_markers.markers.resize(2);
        graph_markers.markers[0] = vertices;

        return graph_markers;
    }

    void IkdTreeTopoGraph::pubGraphMarkers() const
    {
        graph_pub_.publish(generateTopoGraphMarkers());

        // std_msgs::ColorRGBA points_color;
        // points_color.r = 1.0f;
        // points_color.g = 1.0f;
        // points_color.b = 1.0f;
        // points_color.a = 1.0f;
        // geometry_msgs::Vector3 points_scale;
        // points_scale.x = 0.1;
        // points_scale.y = 0.1;
        // points_scale.z = 0.1;
        // graph_pub_2_.publish(generatePointsMarkers(suitable_points_, points_color, points_scale));
        // points_color.r = 0.0f;
        // points_color.g = 1.0f;
        // points_color.b = 0.0f;
        // points_color.a = 1.0f;
        // graph_pub_3_.publish(generatePointsMarkers(sample_points_, points_color, points_scale));
        // points_color.r = 1.0f;
        // points_color.g = 1.0f;
        // points_color.b = 0.0f;
        // points_color.a = 1.0f;
        // graph_pub_4_.publish(generatePointsMarkers(local_points_, points_color, points_scale));
    }
}
