//
// Created by hjl on 2022/6/13.
//

#include "graph/plan_graph_ikdtree.h"

namespace graph
{
    IkdTreePlanGraph::IkdTreePlanGraph()
    {
        is_tree_built_ = false;
        graph_ikdtree_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.2));

        vertex_index_ = 0;
        points_ids_.clear();
        vertices_.clear();
        edges_.clear();
    }

    void IkdTreePlanGraph::resetGraph()
    {
        is_tree_built_ = false;
        graph_ikdtree_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.2));

        vertex_index_ = 0;
        points_ids_.clear();
        vertices_.clear();
        edges_.clear();
    }

    int IkdTreePlanGraph::addVertex(const utils::Point3D &point)
    {
        std::unique_lock<std::shared_timed_mutex> lock_vertex(graph_mutex_);
        int point_id = -1;
        if (points_ids_.count(point) == 0)
        {
            vertices_.insert(std::make_pair(vertex_index_, point));
            points_ids_.insert(std::make_pair(point, vertex_index_));
            edges_.insert(std::make_pair(vertex_index_, std::unordered_map<int, double>()));

            std::unique_lock<std::shared_timed_mutex> lock_point(point_cloud_mutex_);
            point_cloud_.emplace_back(point.x(), point.y(), point.z());

            // if (is_tree_built_)
            // {
            //     graph_ikdtree_->Add_Points(point_cloud_, false);
            // }
            // else
            // {
            //     graph_ikdtree_->Build(point_cloud_);
            //     is_tree_built_ = true;
            // }
            // point_cloud_.clear();

            point_id = vertex_index_;
            vertex_index_++;
        }
        else
        {
            point_id = points_ids_.at(point);
        }

        return point_id;
    }

    bool IkdTreePlanGraph::addOneWayEdge(const int &a_id, const int &b_id, const double &distance)
    {
        std::unique_lock<std::shared_timed_mutex> lock_vertex(graph_mutex_);

        if (a_id != b_id && vertexIndexInRange(a_id) && vertexIndexInRange(b_id))
        {
            auto &edges_a_id = edges_.at(a_id);
            if (edges_a_id.count(b_id) == 0)
            {
                edges_a_id.insert(std::make_pair(b_id, distance));
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    void IkdTreePlanGraph::deleteVertex(const utils::Point3D &point, bool update_kdtree)
    {
        std::unique_lock<std::shared_timed_mutex> lock_vertex(graph_mutex_);

        if (points_ids_.count(point) == 0)
            return;
        else
        {
            std::cerr << "delete: " << Eigen::Vector3d(point.x(), point.y(), point.z()) << std::endl;
            if (update_kdtree)
            {
                std::unique_lock<std::shared_timed_mutex> lock_delete_point(delete_point_cloud_mutex_);
                delete_point_cloud_.emplace_back(point.x(), point.y(), point.z());
                graph_ikdtree_->Delete_Points(delete_point_cloud_);
                delete_point_cloud_.clear();
            }

            int delete_point_id = points_ids_.at(point);

            // 删除边
            std::vector<int> delete_link_ids;
            std::transform(edges_.at(delete_point_id).begin(), edges_.at(delete_point_id).end(),
                           std::back_inserter(delete_link_ids), [](const auto &pair)
                           { return pair.first; });
            for (const auto &id : delete_link_ids)
            {
                edges_.at(id).erase(delete_point_id);
            }
            edges_.erase(delete_point_id);

            // 删除点
            vertices_.erase(delete_point_id);
            points_ids_.erase(point);
        }
    }

    bool IkdTreePlanGraph::addTwoWayEdge(const int &a_id, const int &b_id)
    {
        std::unique_lock<std::shared_timed_mutex> lock_vertex(graph_mutex_);

        if (a_id != b_id && vertexIndexInRange(a_id) && vertexIndexInRange((b_id)))
        {
            double distance = vertices_.at(a_id).distance(vertices_.at(b_id));

            auto &edges_a_id = edges_.at(a_id);
            if (edges_a_id.count(b_id) == 0)
            {
                edges_a_id.insert(std::make_pair(b_id, distance));
            }

            auto &edges_b_id = edges_.at(b_id);
            if (edges_b_id.count(a_id) == 0)
            {
                edges_b_id.insert(std::make_pair(a_id, distance));
            }

            return true;
        }

        return false;
    }

    bool IkdTreePlanGraph::isPoint3DExisted(const utils::Point3D &point) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (points_ids_.count(point) != 0)
            return true;
        else
            return false;
    }

    utils::Point3D IkdTreePlanGraph::getVertex(const int &id) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (vertexIndexInRange(id))
            return vertices_.at(id);
        else
            return utils::Point3D();
    }

    const std::vector<utils::Point3D> IkdTreePlanGraph::getAllVertices() const
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);
        std::vector<utils::Point3D> vertices;
        std::transform(vertices_.begin(), vertices_.end(),
                       std::back_inserter(vertices), [](const auto &pair)
                       { return pair.second; });
        return vertices;
    }

    const std::unordered_map<int, std::unordered_map<int, double>>
    IkdTreePlanGraph::getAllEdges() const
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        return edges_;
    }

    bool IkdTreePlanGraph::getPointId(const utils::Point3D &point, int &point_id)
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (points_ids_.count(point) != 0)
        {
            point_id = points_ids_.at(point);
            return true;
        }
        else
        {
            point_id = -1;
            return false;
        }
    }

    int IkdTreePlanGraph::getLinkNums(const int &point_id) const
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (edges_.count(point_id) != 0)
            return edges_.at(point_id).size();
        else
            return 0;
    }

    void IkdTreePlanGraph::buildKdtree()
    {
        std::unique_lock<std::shared_timed_mutex> lock_point(point_cloud_mutex_);

        graph_ikdtree_->Build(point_cloud_);
        point_cloud_.clear();
    }

    void IkdTreePlanGraph::addPointsToKdtree()
    {
        std::unique_lock<std::shared_timed_mutex> lock_point(point_cloud_mutex_);

        graph_ikdtree_->Add_Points(point_cloud_, false);
        point_cloud_.clear();
    }

    int IkdTreePlanGraph::getNearestVertexId(const utils::Point3D &point)
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (graph_ikdtree_->size() != 0)
        {
            int k = 2;
            PointVector neighbors_vector;
            std::vector<float> distance_vector;
            PointType pt(point.x(), point.y(), point.z());

            graph_ikdtree_->Nearest_Search(pt, k, neighbors_vector, distance_vector);

            utils::Point3D nearest_point(neighbors_vector.front().x,
                                         neighbors_vector.front().y,
                                         neighbors_vector.front().z);
            int nearest_point_id = -1;
            // points_ids_.count(point) != 0;
            if (points_ids_.count(nearest_point) != 0)
            {
                nearest_point_id = points_ids_.at(nearest_point);
                return nearest_point_id;
            }
            else
            {
                std::cerr << "IkdTreePlanGraph getNearestVertexId Fail!" << std::endl;
                return -1;
            }
        }
        else
        {
            std::cerr << "graph is empty!" << std::endl;
            return -1;
        }
    }

    std::vector<int> IkdTreePlanGraph::getNeighborVertexsIDs(const utils::Point3D &point, const double &range,
                                                             std::vector<utils::Point3D> &neighbor_vertexs)
    {
        std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        std::vector<int> vertexs_ids;
        neighbor_vertexs.clear();
        if (graph_ikdtree_->size() != 0)
        {
            PointVector neighbors_vector;
            graph_ikdtree_->Radius_Search(PointType(point.x(), point.y(), point.z()), range, neighbors_vector);

            int id = -1;
            for (const auto &item : neighbors_vector)
            {
                utils::Point3D neigbor_point(item.x, item.y, item.z);
                if (points_ids_.count(neigbor_point) != 0)
                {
                    id = points_ids_.at(neigbor_point);
                    vertexs_ids.push_back(id);
                    neighbor_vertexs.push_back(neigbor_point);
                }
                else
                {
                    std::cerr << "IkdTreePlanGraph getNeighborVertexsIDs Fail!" << std::endl;
                }
            }
        }
        else
        {
            vertexs_ids.clear();
            std::cerr << "graph is empty!" << std::endl;
        }
        return vertexs_ids;
    }

    bool IkdTreePlanGraph::getShortestPath(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids,
                                           utils::Point3DQueue &shortest_path) const
    {
        waypoint_ids.clear();
        shortest_path.clear();

        // std::shared_lock<std::shared_timed_mutex> lock(graph_mutex_);

        if (vertexIndexInRange(start_v_id) && vertexIndexInRange(end_v_id))
        {
            if (start_v_id == end_v_id)
            {
                std::cerr << "start point == end point, return the start point" << std::endl;
                waypoint_ids.push_back(start_v_id);
                shortest_path.push_back(vertices_.at(start_v_id));

                return false;
            }
            else
            {
                if (A_star_search(start_v_id, end_v_id, waypoint_ids))
                {
                    for (const auto &id : waypoint_ids)
                    {
                        shortest_path.push_back(vertices_.at(id));
                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            std::cerr << "start or end point is out of range" << std::endl;
            return false;
        }
    }

    bool IkdTreePlanGraph::A_star_search(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids) const
    {
        enum class status : int8_t
        {
            OPEN,
            CLOSED,
            INIT
        };

        typedef std::pair<double, int> iPair;                                   // Vertices are represented by their index in the graph.vertices list
        std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq; // Priority queue of vertices
        std::vector<double> dist(vertex_index_, INFINITY);                      // Vector of distances
        std::vector<double> estimation(vertex_index_, INFINITY);
        const int INF = 0x3f3f3f3f;                        // integer infinity
        std::vector<int> backpointers(vertex_index_, INF); // Vector of backpointers
        std::vector<status> in_pq(vertex_index_, status::INIT);

        // Add the start vertex
        dist[start_v_id] = 0;
        estimation[start_v_id] = vertices_.at(start_v_id).distance(vertices_.at(end_v_id));
        pq.push(std::make_pair(estimation[start_v_id], start_v_id));
        in_pq[start_v_id] = status::OPEN;

        int u;
        while (!pq.empty())
        { // Loop until priority queue is empty

            auto [current_estimation, u] = pq.top();
            // u = pq.top().second;
            pq.pop(); // Pop the minimum distance vertex

            if (std::abs(current_estimation - estimation[u]) > 1e-9)
                continue;

            // in_pq[u] = status::CLOSED;
            if (u == end_v_id)
            { // Early termination
                std::cerr << "u == end_v_id" << endl;
                break;
                
            }
            for (const auto &edge : edges_.at(u))
            { // Get all adjacent vertices
                // Get vertex label and weight of current adjacent edge of u
                auto v = edge.first;
                auto u_v_dist = edge.second;

                // if (in_pq[v] == status::CLOSED)
                // {
                //     continue;
                // }

                // If there is a shorter path to v through u
                if (dist[v] > dist[u] + u_v_dist)
                {
                    // Updating distance of v
                    dist[v] = dist[u] + u_v_dist;
                    estimation[v] = dist[v] + vertices_.at(v).distance(vertices_.at(end_v_id));
                    backpointers[v] = u;
                    // if (in_pq[v] == status::INIT)
                    // {
                        pq.push(std::make_pair(estimation[v], v));
                    //     in_pq[v] = status::OPEN;
                    // }
                }
            }
        }
        // Backtrack to find path
        waypoint_ids.clear();
        int current = end_v_id;
        if (backpointers[current] == INF)
        { // no path found
            // std::cerr << "no path found, return empty path" << std::endl;
            return false;
        }
        else
        {
            // path found
            while (current != INF)
            {
                waypoint_ids.push_back(current);
                current = backpointers[current];
            }
            // Reverse the path (constructing it this way since vector is more efficient
            // at push_back than insert[0])
            std::reverse(waypoint_ids.begin(), waypoint_ids.end());
            return true;
        }
    }
}
